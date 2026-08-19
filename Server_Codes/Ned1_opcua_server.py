import os
import re
import json
import time
import zipfile
import threading
from opcua import Server, ua
from pyniryo import NiryoRobot,PoseObject,ConveyorID, ConveyorDirection, PinState, PinID,ObjectShape,ObjectColor
#from pyniryo import *

# ─────────────────────────────────────────────
# ROBOT ACCESS LOCK
# ─────────────────────────────────────────────
# pyniryo's TCP client has no internal thread-safety 
ROBOT_LOCK = threading.Lock()


# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

AAS_PATH  = "/home/niryo/niryo_robot_saved_files/NedRobot1.aasx"
ROBOT_IP  = "127.0.0.1"
UPDATE_HZ = 1.0

# Submodel idShort that contains capability containers (PickObject,
# ConveyorCapabilities, ...)
CAPABILITY_SUBMODEL = "CapabilityDescription"

# pyniryo PoseObject attribute names 
POSE_ATTR_ALIASES = {
    "x": "x", "posex": "x",
    "y": "y", "posey": "y",
    "z": "z", "posez": "z",
    "roll": "roll", "rollx": "roll",
    "pitch": "pitch", "pitchy": "pitch",
    "yaw": "yaw", "yawz": "yaw",
}


# ─────────────────────────────────────────────
# 1. LOAD / SAVE AAS FILE
# ─────────────────────────────────────────────

def load_aas_data(path: str) -> dict:
    """
    Accepts a plain .json or an .aasx (ZIP) file.
    """
    if path.endswith(".aasx"):
        try:
            with zipfile.ZipFile(path, "r") as z:
                candidates = [n for n in z.namelist() if n.endswith(".json")]
                if not candidates:
                    raise FileNotFoundError(f"No .json found inside {path}. Contents: {z.namelist()}"                    )
                target = candidates[0]
                print(f"[AASX] Reading from ZIP: {target}")
                with z.open(target) as f:
                    return json.load(f)
        except zipfile.BadZipFile:
            print(f"[AASX] Not a ZIP (was previously saved as JSON) — "
                  f"reading as plain JSON: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

#  save_aas_data() writes back as plain JSON regardless of  extension 
def save_aas_data(path: str, aas_data: dict):
    """Atomic write — avoids a corrupted file if the Pi loses power mid-write."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(aas_data, f, indent=2)
    os.replace(tmp, path)


# ─────────────────────────────────────────────
# 2. EXTRACT OPCUAServerAccess CONFIGURATION
# ─────────────────────────────────────────────

def extract_opcua_config(aas_data: dict) -> dict:
    for sm in aas_data.get("submodels", []):
        if sm.get("idShort") == "OPCUAServerAccess":
            config = {}
            for elem in sm.get("submodelElements", []):
                id_short  = elem.get("idShort")
                raw_value = elem.get("value")
                if id_short == "endpointURL":
                    config["endpoint"] = raw_value
                elif id_short == "NamespaceURI":
                    config["namespace_uri"] = raw_value
                elif id_short == "ServerObjectName":
                    config["server_name"] = raw_value
            missing = [k for k in ("endpoint", "namespace_uri", "server_name") if k not in config]
            if missing:
                raise ValueError(f"OPCUAServerAccess missing fields: {missing}")
            return config
    raise ValueError("OPCUAServerAccess submodel not found in AAS JSON.")


# ─────────────────────────────────────────────
# 3. VALUE TYPE MAPPING 
# ─────────────────────────────────────────────

XS_TYPE_MAP = {
    "xs:boolean": bool, "xs:integer": int, "xs:int": int,
    "xs:float": float, "xs:double": float, "xs:string": str,
}

def coerce_value(raw_val, xs_type: str):
    if raw_val is None:
        return ""
    target_type = XS_TYPE_MAP.get(xs_type, str)
    try:
        if target_type == bool:
            return raw_val.lower() == "true"
        return target_type(raw_val)
    except (ValueError, TypeError):
        return str(raw_val)


# ─────────────────────────────────────────────
# 4. CAPABILITY METHOD REGISTRY
# ─────────────────────────────────────────────

def _method_move_pose(robot, args):
    """
    args: [x, y, z, roll, pitch, yaw] as floats.
    """

    x, y, z, roll, pitch, yaw = [_get_arg(args, i, 0.0) for i in range(6)]
    print(f"  [MovePose] Received: x={x}, y={y}, z={z}, "
          f"roll={roll}, pitch={pitch}, yaw={yaw}")

    with ROBOT_LOCK:
        before = robot.get_pose()
        print(f"  [MovePose] Pose BEFORE: x={before.x}, y={before.y}, z={before.z}, "
              f"roll={before.roll}, pitch={before.pitch}, yaw={before.yaw}")

        robot.move_pose(x, y, z, roll, pitch, yaw)

        after = robot.get_pose()
        print(f"  [MovePose] Pose AFTER:  x={after.x}, y={after.y}, z={after.z},"
              f"roll={after.roll}, pitch={after.pitch}, yaw={after.yaw}")

    moved = abs(after.x - before.x) > 1e-4 or abs(after.y - before.y) > 1e-4 \
            or abs(after.z - before.z) > 1e-4
    if not moved:
        print("  [MovePose] WARNING: pose unchanged after move() call — "
              "target was likely unreachable and was silently ignored.")

    return [ua.Variant(bool(moved), ua.VariantType.Boolean)]

def _method_get_pose(robot, args):
    with ROBOT_LOCK:
        pose = robot.get_pose()
    result = f"{pose.x},{pose.y},{pose.z},{pose.roll},{pose.pitch},{pose.yaw}"
    return [ua.Variant(result, ua.VariantType.String)]

def _method_grasp(robot, args):
    with ROBOT_LOCK:
        robot.grasp_with_tool()
    return [ua.Variant(True, ua.VariantType.Boolean)]

def _method_release(robot, args):
    with ROBOT_LOCK:
        robot.release_with_tool()
    return [ua.Variant(True, ua.VariantType.Boolean)]

def _get_arg(args, index, default):
    """
    Safely extracts a value from an OPC UA method args list.
    """
    if index >= len(args):
        return default
    arg = args[index]
    # ua.Variant has a .Value attribute; plain Python types do not
    return arg.Value if hasattr(arg, "Value") else arg


def _method_run_conveyor(robot, args):
    """
    Starts the conveyor, polls the digital sensor until an object is detected,
    then stops the conveyor. Returns True if object was detected, False on timeout.
    """
    raw_id  = int(_get_arg(args, 0, 1))
    speed   = int(_get_arg(args, 1, 50))
    raw_dir = int(_get_arg(args, 2, 1))

    conveyor_id = {1: ConveyorID.ID_1, 2: ConveyorID.ID_2}.get(raw_id, ConveyorID.ID_1)
    direction   = ConveyorDirection.FORWARD if raw_dir >= 0 else ConveyorDirection.BACKWARD
    sensor_pin  = PinID.DI5
    POLL_INTERVAL = 0.1   # seconds between sensor reads
    TIMEOUT       = 180.0  # max seconds to wait before giving up

    print(f"[RunConveyor] id={conveyor_id}, speed={speed}, direction={direction}")

    # Start the conveyor — short lock acquisition, then release immediately
    with ROBOT_LOCK:
        robot.run_conveyor(conveyor_id, speed=speed, direction=direction)

    detected = False
    deadline = time.time() + TIMEOUT

    # Poll sensor in a tight loop; each read acquires then releases the lock
    # so other OPC-UA method calls are not starved
    while time.time() < deadline:
        with ROBOT_LOCK:
            stat = robot.digital_read(sensor_pin)

        if stat == PinState.LOW:   # LOW = object in front of sensor
            detected = True
            print(f"[RunConveyor] Object detected on DI5 as {detected}.")
            break

        time.sleep(POLL_INTERVAL)

    else:
        print(f"[RunConveyor] Timeout after {TIMEOUT}s — stopping conveyor anyway.")

    # Stop the conveyor
    try:
        with ROBOT_LOCK:
            if detected == True:
                robot.stop_conveyor(conveyor_id)
                print("[RunConveyor] Conveyor stopped.")
    except Exception as e:
        print(f"[RunConveyor] Could not stop conveyor: {e}")

    return [ua.Variant(detected, ua.VariantType.Boolean)]



def _method_stop_conveyor(robot, args):    
    #Stops the conveyor.  
    raw_id      = int(_get_arg(args, 0, 1))
    conveyor_id = {1: ConveyorID.ID_1, 2: ConveyorID.ID_2}.get(raw_id, ConveyorID.ID_1)    
    print(f"  [StopConveyor] conveyor_id={conveyor_id}")
    with ROBOT_LOCK:
        robot.stop_conveyor(conveyor_id)
    return [ua.Variant(True, ua.VariantType.Boolean)]


def _method_vision_pick(robot, args):
    workspace_name = _get_arg(args, 0, "")
    height_offset  = float(_get_arg(args, 1, 0.0))
    shape_str      = str(_get_arg(args, 2, "ANY")).upper()
    color_str      = str(_get_arg(args, 3, "ANY")).upper()
    

    shape = getattr(ObjectShape, shape_str, ObjectShape.ANY)
    color = getattr(ObjectColor, color_str, ObjectColor.ANY)
    print(f"  [VisionPick DEBUG] raw shape_str={shape_str!r}, raw color_str={color_str!r}")
    print(f"  [VisionPick DEBUG] resolved shape={shape}, resolved color={color}"
)

    with ROBOT_LOCK:
        obj_found, shape_ret, color_ret = robot.vision_pick(
            workspace_name, height_offset=height_offset, shape=shape, color=color
        )


    print(f"  [VisionPick] obj_found={obj_found}, shape={shape_ret.value}, color={color_ret.value}")

    return [
        ua.Variant(bool(obj_found), ua.VariantType.Boolean),
        ua.Variant(shape_ret.value, ua.VariantType.String),
        ua.Variant(color_ret.value, ua.VariantType.String)
    ]




# Function name (parsed from the AAS string) → (wrapper, input arg names).
# Reason: still a registry, because "what does move_pose() need as
# arguments" is genuine domain knowledge that can't be auto-discovered
# from a string like "robot.move_pose()" alone.
CAPABILITY_REGISTRY = {
    "move_pose": (
        _method_move_pose,
        [("x", ua.ObjectIds.Double), ("y", ua.ObjectIds.Double), ("z", ua.ObjectIds.Double),
         ("roll", ua.ObjectIds.Double), ("pitch", ua.ObjectIds.Double), ("yaw", 
ua.ObjectIds.Double)],
        [("Result", ua.ObjectIds.Boolean)],
    ),
    "get_pose": (
        _method_get_pose,
        [],
        [("Result", ua.ObjectIds.String)],
    ),
    "grasp_with_tool": (
        _method_grasp,
        [],
        [("Result", ua.ObjectIds.Boolean)],
    ),
    "release_with_tool": (
        _method_release,
        [],
        [("Result", ua.ObjectIds.Boolean)],
    ),
    "run_conveyor": (
        _method_run_conveyor,
        [("conveyor_id", ua.ObjectIds.Double), ("speed", ua.ObjectIds.Double), (
"direction", ua.ObjectIds.Double)],
        [("Result", ua.ObjectIds.Boolean)],
    ),
    "stop_conveyor": (
        _method_stop_conveyor,
        [("conveyor_id", ua.ObjectIds.Double)],
        [("Result", ua.ObjectIds.Boolean)],
    ),
    "vision_pick": (
        _method_vision_pick,
        [("workspace_name", ua.ObjectIds.String), ("height_offset", ua.ObjectIds
.Double),
         ("shape", ua.ObjectIds.String), ("color", ua.ObjectIds.String)],
        [("ObjectFound", ua.ObjectIds.Boolean), ("Shape", ua.ObjectIds.String), 
("Color", ua.ObjectIds.String)],
    ),
}

def parse_function_name(method_string: str) -> str:
    """Extracts 'move_pose' from 'robot.move_pose()'."""
    match = re.search(r"\.(\w+)\s*\(", method_string)
    return match.group(1) if match else ""


def make_argument(name: str, type_id) -> ua.Argument:
    """
    Builds a ua.Argument for OPC UA method signatures.
    """
    arg = ua.Argument()
    arg.Name = name
    arg.DataType = ua.NodeId(type_id)
    arg.ValueRank = -1
    arg.ArrayDimensions = []
    arg.Description = ua.LocalizedText(name)
    return arg


# ─────────────────────────────────────────────
# 5. DISCOVERY REGISTRY
# ─────────────────────────────────────────────
# Built once while walking the AAS

class Discovery:
    def __init__(self):
        self.pose_fields  = {}   # pyniryo attr name -> (opcua_node, aas_path tuple)
        self.bool_fields  = []   # list of (opcua_node, aas_path tuple)
        self.methods      = []   # list of (idShort, bound_function_name)
        self.all_nodes    = []   # list of (aas_path tuple, modelType) — for the report

    def report(self):
        print("\n" + "═" * 60)
        print(" DISCOVERY REPORT")
        print("═" * 60)

        print(f"\nLive pose fields ({len(self.pose_fields)}):")
        for attr, (node, path) in self.pose_fields.items():
            print(f"{node}  • {' / '.join(path):<45} → pyniryo .{attr}")

        print(f"\nBoolean fields watched ({len(self.bool_fields)}):")
        for node, path in self.bool_fields:
            print(f"{node}  • {' / '.join(path)}")

        print(f"\nExecutable methods bound ({len(self.methods)}):")
        for id_short, func_name in self.methods:
            print(f"  • {id_short:<20} → robot.{func_name}()")

        print(f"\nTotal nodes mapped: {len(self.all_nodes)}")
        print("═" * 60 + "\n")


# ─────────────────────────────────────────────
# 6. RECURSIVE AAS → OPC UA NODE MAPPER 
# ─────────────────────────────────────────────

def add_elements_to_node(parent_node, elements: list, idx: int, robot_ref: dict,
                          discovery: Discovery, path: tuple, inside_capability: 
bool = False):
    """
    Recursively maps AAS submodel elements to OPC UA nodes, and records
    everything it finds into `discovery` as it goes.
    """
    for elem in elements:
        model_type = elem.get("modelType", "")
        id_short   = elem.get("idShort", "Unknown")
        xs_type    = elem.get("valueType", "xs:string")
        elem_path  = path + (id_short,)

        if model_type == "Property" and inside_capability:
            method_string = elem.get("value", "")
            func_name     = parse_function_name(method_string)
            entry         = CAPABILITY_REGISTRY.get(func_name)

            if entry is None:
                print(f"  [SKIP-METHOD] {id_short} — unknown function '{func_name}'")
                continue

            wrapper, arg_names, out_arg_specs = entry

            def make_callback(wrapper_fn=wrapper):
                def callback(parent, *input_args):
                    robot = robot_ref.get("robot")
                    if robot is None:
                        return [ua.Variant(False, ua.VariantType.Boolean)]
                    try:
                        # DEBUG — print exactly what opcua delivers
                        print(f"  [DEBUG] input_args type: {type(input_args)}")
                        print(f"  [DEBUG] input_args value: {input_args}")
                        for i, a in enumerate(input_args):
                            print(f"  [DEBUG] input_args[{i}] type={type(a)} value={a}")
                            if isinstance(a, list):
                                for j, item in enumerate(a):
                                    print(f"  [DEBUG]   [{i}][{j}] type={type(item)} value={item}")

                        flat = []
                        for a in input_args:
                            if isinstance(a, list):
                                for item in a:
                                    flat.append(item.Value if hasattr(item, "Value") else item)
                            else:
                                flat.append(a.Value if hasattr(a, "Value") else 
a)

                        print(f"  [DEBUG] flat result: {flat}")
                        return wrapper_fn(robot, flat)
                    except Exception as e:
                        print(f"  [METHOD ERROR] {e}")
                        return [ua.Variant(False, ua.VariantType.Boolean)]
                return callback

            in_args  = [make_argument(name, dtype) for name, dtype in arg_names]
            out_args = [make_argument(name, dtype) for name, dtype in out_arg_specs]
            meth = parent_node.add_method(idx, id_short, make_callback(), in_args, out_args)

            print(f"HERE!!!:{func_name} {meth}")
            discovery.methods.append((id_short, func_name))
            discovery.all_nodes.append((elem_path, "Method"))

        elif model_type == "Property":
            typed_val = coerce_value(elem.get("value"), xs_type)
            var = parent_node.add_variable(idx, id_short, typed_val)
            var.set_writable()
            discovery.all_nodes.append((elem_path, "Property"))

            # ── Auto-discover pose-like fields by name match ──
            attr = POSE_ATTR_ALIASES.get(id_short.lower())
            if attr is not None and xs_type in ("xs:float", "xs:double", "xs:integer", "xs:int"):
                discovery.pose_fields[attr] = (var, elem_path)

            # ── Auto-discover boolean fields ──
            if xs_type == "xs:boolean":
                discovery.bool_fields.append((var, elem_path))

        elif model_type == "MultiLanguageProperty":
            lang_vals = elem.get("value", [])
            text = next((lv.get("text", "") for lv in lang_vals if lv.get("language") == "en"), "")
            if not text and lang_vals:
                text = lang_vals[0].get("text", "")
            var = parent_node.add_variable(idx, id_short, str(text))
            var.set_writable()
            discovery.all_nodes.append((elem_path, "MultiLanguageProperty"))

        elif model_type in ("SubmodelElementCollection", "SubmodelElementList"):
            child_node = parent_node.add_object(idx, id_short)
            print(f"[CHILD NODES for SMCs]:{child_node}---{id_short}")
            discovery.all_nodes.append((elem_path, model_type))
            children = elem.get("value", [])
            if children:
                add_elements_to_node(child_node, children, idx, robot_ref, discovery,
                                      elem_path, inside_capability=inside_capability)

        elif model_type == "Range":
            min_var = parent_node.add_variable(idx, f"{id_short}_min", str(elem.
get("min", "")))
            max_var = parent_node.add_variable(idx, f"{id_short}_max", str(elem.
get("max", "")))
            min_var.set_writable()
            max_var.set_writable()
            discovery.all_nodes.append((elem_path, "Range"))

        elif model_type == "File":
            var = parent_node.add_variable(idx, id_short, str(elem.get("value", 
"")))
            var.set_writable()
            discovery.all_nodes.append((elem_path, "File"))

        else:
            print(f"  [SKIP] {id_short} — unsupported modelType: {model_type}")


# ─────────────────────────────────────────────
# 7. NODE FINDER (browse-name walk — still handy for one-off lookups)
# ─────────────────────────────────────────────

def find_node(parent, *browse_names):
    node = parent
    for name in browse_names:
        found = next((c for c in node.get_children()
                      if c.get_browse_name().Name == name), None)
        if found is None:
            raise LookupError(f"Node '{name}' not found under {node}")
        node = found
    return node


# ─────────────────────────────────────────────
# 8. UPDATE AAS DICT IN MEMORY 
# ─────────────────────────────────────────────

def update_aas_property(aas_data: dict, submodel_id: str, path: tuple, new_value
):
    """path is the full idShort chain INCLUDING the property name."""
    for sm in aas_data.get("submodels", []):
        if sm.get("idShort") == submodel_id:
            elements = sm.get("submodelElements", [])
            for step in path[:-1]:
                for elem in elements:
                    if elem.get("idShort") == step:
                        elements = elem.get("value", [])
                        break
            for elem in elements:
                if elem.get("idShort") == path[-1]:
                    elem["value"] = str(new_value)
                    return


# ─────────────────────────────────────────────
# 9. CONVEYOR SUBSCRIPTION HANDLER 
# ─────────────────────────────────────────────

class ConveyorCommandHandler:
    """
    Watches every auto-discovered boolean field for client writes.
    """
    def __init__(self, robot_ref):
        self.robot_ref = robot_ref

    def datachange_notification(self, node, val, data):
        name = node.get_browse_name().Name
        robot = self.robot_ref.get("robot")
        if robot is None or "conveyor" not in name.lower():
            return
        try:
            with ROBOT_LOCK:
                if val is True:
                    robot.run_conveyor(ConveyorID.ID_1, speed=50,
                                       direction=ConveyorDirection.FORWARD)
                else:
                    robot.stop_conveyor(ConveyorID.ID_1)
            print(f"[Subscription] {name} → {val} → conveyor "
                  f"{'started' if val else 'stopped'}")
        except Exception as e:
            print(f"[Subscription] Conveyor command error: {e}")


# ─────────────────────────────────────────────
# 10. LIVE UPDATE LOOP (discovery-driven)
# ─────────────────────────────────────────────

def live_update_loop(robot_ref, discovery, aas_data, aas_path, interval):
    """
    Polls the robot and updates EVERY discovered pose field and the
    first discovered sensor-like boolean 
    """
    if not discovery.pose_fields:
        print("[LiveUpdate] No pose fields discovered — live pose update disabled.")
    else:
        print(f"[LiveUpdate] Tracking {len(discovery.pose_fields)} pose field(s).")

    # Heuristic: a "sensor" boolean is one whose path contains "sensor"
    # or "infrared" — still discovered, just filtered by name at use-time
    # rather than hardcoded as a separate lookup path.
    sensor_entries = [(node, path) for node, path in discovery.bool_fields
                       if "sensor" in path[-1].lower() or "infrared" in " ".join
(path).lower()]

    save_counter = 0
    print("[LiveUpdate] Polling started.\n")

    while True:
        robot = robot_ref.get("robot")
        if robot is None:
            time.sleep(interval)
            continue
        try:
            with ROBOT_LOCK:
                pose = robot.get_pose()
            for attr, (node, path) in discovery.pose_fields.items():
                live_val = round(getattr(pose, attr), 4)
                node.set_value(live_val)
                # path[0] is the submodel idShort; rest is the element chain
                update_aas_property(aas_data, path[0], path[1:], live_val)

            if sensor_entries:
                try:
                    with ROBOT_LOCK:
                        hw = robot.get_hardware_status()
                    if hw.digital_input_states:
                        sensor_val = bool(hw.digital_input_states[0])
                        for node, path in sensor_entries:
                            node.set_value(sensor_val)
                            update_aas_property(aas_data, path[0], path[1:],
                                                 str(sensor_val).lower())
                except Exception:
                    pass

            save_counter += 1
            if save_counter >= 10:
                save_aas_data(aas_path, aas_data)
                save_counter = 0

        except Exception as e:
            print(f"[LiveUpdate] Error: {e}")

        time.sleep(interval)


# ─────────────────────────────────────────────
# 11. MAIN
# ─────────────────────────────────────────────

def main():
    aas_data = load_aas_data(AAS_PATH)
    cfg      = extract_opcua_config(aas_data)
    print(f"\n[Config] Endpoint     : {cfg['endpoint']}")
    print(f"[Config] Namespace URI: {cfg['namespace_uri']}")
    print(f"[Config] Server name  : {cfg['server_name']}\n")

    server = Server()
    server.set_endpoint(cfg["endpoint"])
    server.set_server_name(cfg["server_name"])
    idx     = server.register_namespace(cfg["namespace_uri"])
    objects = server.get_objects_node()
    print(f"[OPC UA] Namespace '{cfg['namespace_uri']}' registered as ns={idx}")

    robot_ref = {"robot": None}
    discovery = Discovery()

    for submodel in aas_data.get("submodels", []):
        sm_id   = submodel.get("idShort", "Unknown")
        sm_node = objects.add_object(idx, sm_id)
        print(f"[AAS] Mapping submodel: {sm_id}")
        # inside_capability starts True only for the CapabilityDescription
        # submodel — its Properties (however deeply nested) become Methods.
        add_elements_to_node(sm_node, submodel.get("submodelElements", []), idx,
                              robot_ref, discovery, path=(sm_id,),
                              inside_capability=(sm_id == CAPABILITY_SUBMODEL))

    discovery.report()
    
    def list_clients():
        try:
            clients = server.bserver.clients
            if not clients:
                print("[Clients] No clients currently connected")
            else:
                print(f"[Clients] {len(clients)} connected:")
                for c in clients:
                    peer = getattr(c, "peername","unknown")
                    print(f" .{peer}")
        except Exception as e:
            print(f"[Clients] Could not retrieve client list: {e}")

    server.start()
    print(f"OPC UA Server running at {cfg['endpoint']}")

    try:
        conn_node = find_node(objects, "OPCUAServerAccess", "ConnectionStatus")
        conn_node.set_value(True)
    except Exception as e:
        conn_node = None
        print(f"[AAS] ConnectionStatus update skipped: {e}")

    robot = None
    try:
        robot = NiryoRobot(ROBOT_IP)
        with ROBOT_LOCK:
            robot.calibrate_auto()

            conveyor_id =robot.set_conveyor()
            print(f"CONVEYOR REGISTERED AS: {conveyor_id}")

        robot_ref["robot"] = robot
        print(f"[Robot] Connected at {ROBOT_IP}")


        # Subscribe to every discovered boolean field — the handler itself
        # filters for "conveyor" in the name at notification time.
        handler      = ConveyorCommandHandler(robot_ref)
        subscription = server.create_subscription(200, handler)
        for node, path in discovery.bool_fields:
            subscription.subscribe_data_change(node)
        print(f"[Server] Subscribed to {len(discovery.bool_fields)} boolean field(s).")

        t = threading.Thread(target=live_update_loop,
                             args=(robot_ref, discovery, aas_data, AAS_PATH, UPDATE_HZ),
                              daemon=True)
        t.start()
        print(f"[LiveUpdate] Started — polling every {UPDATE_HZ}s\n")

    except Exception as e:
        print(f"[Robot] Could not connect: {e}")
        print("        Server running with static values. Methods will no-op.\n"
)

    try:
        tick = 0
        while True:
            time.sleep(1)
            tick +=1
            if tick %30 ==0:
                list_clients()
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")
    finally:
        if conn_node:
            try:
                conn_node.set_value(False)
            except Exception:
                pass
        if robot:
            try:
                with ROBOT_LOCK:
                    robot.end()
            except Exception:
                pass
        server.stop()
        save_aas_data(AAS_PATH, aas_data)
        print("[Server] Stopped. AAS saved.")


if __name__ == "__main__":
    main()

