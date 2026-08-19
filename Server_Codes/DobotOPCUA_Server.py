import os
import re
import json
import time
import zipfile
import threading
from opcua import Server, ua
from dobot_api import DobotApi, DobotApiDashboard, DobotApiMove

ROBOT_LOCK = threading.Lock()

#----------Configuration-------------------
AAS_PATH        = "/home/chaikmat/RunChain_devel/OPCUA_AAS/AAS_Files/Dobot_V1.aasx"
ROBOT_IP        = "192.168.1.6"      # default IP
DASHBOARD_PORT  = 29999
MOVE_PORT       = 30003
UPDATE_HZ       = 1.0

# Capabilities description
CAPABILITY_SUBMODEL = "CapabilityDescription"

# Pose attribute names
POSE_ATTR_ALIASES = {
    "PoseX": "x",
    "PoseY": "y",
    "PoseZ": "z",
    "RotEndEffector": "r"
}


# ---------1. LOAD / SAVE AAS FILE-----------

def load_aas_data(path: str) -> dict:
    """
    Accepts a plain .json or an .aasx (ZIP) file.
    """
    if path.endswith(".aasx"):
        try:
            with zipfile.ZipFile(path, "r") as z:
                candidates = [n for n in z.namelist() if n.endswith(".json")]
                if not candidates:
                    raise FileNotFoundError(f"No .json found inside {path}. Contents: {z.namelist()}")
                target = candidates[0]
                print(f"[AASX] Reading from ZIP: {target}")
                with z.open(target) as f:
                    return json.load(f)
        except zipfile.BadZipFile:
            print(f"[AASX] Not a ZIP (was previously saved as JSON) — "
                  f"reading as plain JSON: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_aas_data(path: str, aas_data: dict):
    """Atomic write — avoids a corrupted file if the Pi loses power mid-write."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(aas_data, f, indent=2)
    os.replace(tmp, path)


# ---------2. EXTRACT OPCUAServerAccess CONFIGURATION-----------
def extract_opcua_config(aas_data: dict) -> dict:
    for sm in aas_data.get("submodels", []):
        if sm.get("idShort") == "OPCUAServerAccess":
            config = {}
            for elem in sm.get("submodelElements", []):
                id_short = elem.get("idShort")
                raw_value = elem.get("value")
                if id_short == "endpointURL":
                    config["endpoint"] = raw_value
                elif id_short == "NamespaceURI":
                    config["namespace_uri"] = raw_value
                elif id_short == "ServerName":
                    config["server_name"] = raw_value
            missing = [k for k in ("endpoint", "namespace_uri", "server_name") if k not in config]
            if missing:
                raise ValueError(f"OPCUAServerAccess missing fields: {missing}")
            return config
    raise ValueError("OPCUAServerAccess submodel not found in AAS JSON.")


# ----------- 3. VALUE TYPE MAPPING -------------------
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


def _get_arg(args, index, default):
    if index >= len(args):
        return default
    arg = args[index]
    return arg.Value if hasattr(arg, "Value") else arg


# ----------- Dobot pose-string parser -----------
POSE_BLOCK_RE = re.compile(r"\{([-\d.,\s]+)\}")


def parse_pose_string(raw: str) -> dict:
    match = POSE_BLOCK_RE.search(raw)
    if not match:
        raise ValueError(f"Could not parse Dobot pose string: {raw!r}")
    values = [float(v) for v in match.group(1).split(",") if v.strip() != ""]
    keys = ["x", "y", "z", "r"]
    return dict(zip(keys, values))



def _is_success_reply(raw: str) -> bool:
    """'0' means success; else,an error code."""
    if raw is None:
        return False
    return raw.strip().startswith("0")



# ----------- 4. MOVE CAPABILITY  -------------------
def _method_move_pose(robot_ref, args):
    """args: [x, y, z, r] as floats."""
    userparam = 1  # Integer ID for user parameter (check if this is the correct value)
    toolparam = 1  # Integer ID for tool parameter (check if this is the correct value)
    speedlparam = 20  # Speed (ensure this is in the correct units and range)
    acclparam = 20  # Acceleration (ensure this is in the correct units and range)
    cpparam = 0  # Check if CP is a boolean or integer flag

    x, y, z, r = [_get_arg(args, i, 0.0) for i in range(4)]
    print(f"  [MovePose] Received: x={x}, y={y}, z={z}, Rot={r}")
 
    dashboard = robot_ref.get("dashboard")
    move = robot_ref.get("move")
    if dashboard is None or move is None:
        print("  [MovePose] dashboard/move not connected yet.")
        return [ua.Variant(False, ua.VariantType.Boolean)]
 
    with ROBOT_LOCK:
        before = dashboard.GetPose()
        print(f"  [MovePose] Pose BEFORE: {before}")
        reply = move.MovJ(x, y, z, r,userparam, toolparam, speedlparam, acclparam, cpparam)
        success_move = _is_success_reply(reply)
 
    return [ua.Variant(bool(success_move), ua.VariantType.Boolean)]
 
 
def _method_move_linear(robot_ref, args):
    """Linear move. Same argument shape as MovJ, but follows a linear trajectory"""
    x, y, z, r = [_get_arg(args, i, 0.0) for i in range(4)]
    dashboard = robot_ref.get("dashboard")
    move = robot_ref.get("move")
    if dashboard is None or move is None:
        return [ua.Variant(False, ua.VariantType.Boolean)]
 
    with ROBOT_LOCK:
        print(f"  [MovL] Target: x={x}, y={y}, z={z}, Rot={r}")
        reply = move.MovL(x, y, z, r)
        success_move = _is_success_reply(reply)
 
    return [ua.Variant(bool(success_move), ua.VariantType.Boolean)]
 
 
def _method_digital_output(robot_ref, args):
    """Digital output control (e.g. vacuum gripper on/off)."""
    index = int(_get_arg(args, 0, 1))
    status = int(bool(_get_arg(args, 1, False)))
    dashboard = robot_ref.get("dashboard")
    if dashboard is None:
        return [ua.Variant(False, ua.VariantType.Boolean)]
 
    with ROBOT_LOCK:
        print(f"  [DO] index={index}, status={status}")
        reply = dashboard.DO(index, status)
        success = _is_success_reply(reply)
 
    return [ua.Variant(bool(success), ua.VariantType.Boolean)]
 
 
def _method_get_pose(robot_ref, args):
    """On-demand pose read, Returns x, y, z, r """
    dashboard = robot_ref.get("dashboard")
    if dashboard is None:
        return [ua.Variant(0.0, ua.VariantType.Double) for _ in range(4)]
 
    with ROBOT_LOCK:
        raw = dashboard.GetPose()
    pose = parse_pose_string(raw)
    return [ua.Variant(pose.get(k, 0.0), ua.VariantType.Double) for k in ("x", "y", "z", "r")]
 
 
CAPABILITY_REGISTRY = {
    "move_to_pose": (
        _method_move_pose,
        [("x", ua.ObjectIds.Double), ("y", ua.ObjectIds.Double), ("z", ua.ObjectIds.Double), ("r", ua.ObjectIds.Double)],
        [("success_move", ua.ObjectIds.Boolean)],
    ),
    "MovJ": (
        _method_move_pose,
        [("x", ua.ObjectIds.Double), ("y", ua.ObjectIds.Double), ("z", ua.ObjectIds.Double), ("r", ua.ObjectIds.Double)],
        [("success_move", ua.ObjectIds.Boolean)],
    ),
    "MovL": (
        _method_move_linear,
        [("x", ua.ObjectIds.Double), ("y", ua.ObjectIds.Double), ("z", ua.ObjectIds.Double), ("r", ua.ObjectIds.Double)],
        [("success_move", ua.ObjectIds.Boolean)],
    ),
    "DO": (
        _method_digital_output,
        [("index", ua.ObjectIds.Int32), ("status", ua.ObjectIds.Boolean)],
        [("success", ua.ObjectIds.Boolean)],
    ),
    "GetPose": (
        _method_get_pose,
        [],
        [("x", ua.ObjectIds.Double), ("y", ua.ObjectIds.Double), ("z", ua.ObjectIds.Double), ("r", ua.ObjectIds.Double)],
    ),
}



def parse_function_name(method_string: str) -> str:
    match = re.search(r"(?:^|\.)(\w+)\s*(?:\(|$)", method_string.strip())
    return match.group(1) if match else ""



def make_argument(name: str, type_id) -> ua.Argument:
    arg = ua.Argument()
    arg.Name = name
    arg.DataType = ua.NodeId(type_id)
    arg.ValueRank = -1
    arg.ArrayDimensions = []
    arg.Description = ua.LocalizedText(name)
    return arg


# ------------ 5. DISCOVERY REGISTRY ----------------
class Discovery:
    def __init__(self):
        self.pose_fields = {}
        self.bool_fields = []
        self.methods = []
        self.all_nodes = []

    def report(self):
        print("\n" + "═" * 60)
        print(" DISCOVERY REPORT")
        print("═" * 60)
        print(f"\nLive pose fields ({len(self.pose_fields)}):")
        for attr, (node, path) in self.pose_fields.items():
            print(f"{node}  • {' / '.join(path):<45} → parsed_pose['{attr}']")
        print(f"\nBoolean fields watched ({len(self.bool_fields)}):")
        for node, path in self.bool_fields:
            print(f"{node}  • {' / '.join(path)}")
        print(f"\nExecutable methods bound ({len(self.methods)}):")
        for id_short, func_name in self.methods:
            print(f"  • {id_short:<20} → dobot.{func_name}()")
        print(f"\nTotal nodes mapped: {len(self.all_nodes)}")
        print("═" * 60 + "\n")


# ------------ 6. RECURSIVE AAS to OPC UA NODE MAPPER ----------------
def add_elements_to_node(parent_node, elements: list, idx: int, robot_ref: dict,
                          discovery: Discovery, path: tuple, inside_capability: bool = False):
    for elem in elements:
        model_type = elem.get("modelType", "")
        id_short = elem.get("idShort", "Unknown")
        xs_type = elem.get("valueType", "xs:string")
        elem_path = path + (id_short,)

        if model_type == "Property" and inside_capability:
            method_string = elem.get("value", "")
            func_name = parse_function_name(method_string)
            entry = CAPABILITY_REGISTRY.get(func_name)

            if entry is None:
                print(f"  [SKIP-METHOD] {id_short} — unknown function '{func_name}'")
                continue

            wrapper, arg_names, out_arg_specs = entry

            def make_callback(wrapper_fn=wrapper):
                def callback(parent, *input_args):
                    # Changed: pass the whole robot_ref dict through instead of
                    # robot_ref.get("robot") — MG400 wrapper functions need
                    # both "dashboard" and "move" separately, not one object.
                    if robot_ref.get("dashboard") is None or robot_ref.get("move") is None:
                        return [ua.Variant(False, ua.VariantType.Boolean)]
                    try:
                        flat = []
                        for a in input_args:
                            if isinstance(a, list):
                                for item in a:
                                    flat.append(item.Value if hasattr(item, "Value") else item)
                            else:
                                flat.append(a.Value if hasattr(a, "Value") else a)
                        return wrapper_fn(robot_ref, flat)
                    except Exception as e:
                        print(f"  [METHOD ERROR] {e}")
                        return [ua.Variant(False, ua.VariantType.Boolean)]
                return callback

            in_args = [make_argument(name, dtype) for name, dtype in arg_names]
            out_args = [make_argument(name, dtype) for name, dtype in out_arg_specs]
            meth = parent_node.add_method(idx, id_short, make_callback(), in_args, out_args)
            
            discovery.methods.append((id_short, func_name))
            discovery.all_nodes.append((elem_path, "Method"))

        elif model_type == "Property":
            typed_val = coerce_value(elem.get("value"), xs_type)
            var = parent_node.add_variable(idx, id_short, typed_val)
            var.set_writable()
            discovery.all_nodes.append((elem_path, "Property"))

            attr = POSE_ATTR_ALIASES.get(id_short.lower())
            if attr is not None and xs_type in ("xs:float", "xs:double", "xs:integer", "xs:int"):
                discovery.pose_fields[attr] = (var, elem_path)

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
            discovery.all_nodes.append((elem_path, model_type))
            children = elem.get("value", [])
            if children:
                add_elements_to_node(child_node, children, idx, robot_ref, discovery,
                                      elem_path, inside_capability=inside_capability)

        elif model_type == "Range":
            min_var = parent_node.add_variable(idx, f"{id_short}_min", str(elem.get("min", "")))
            max_var = parent_node.add_variable(idx, f"{id_short}_max", str(elem.get("max", "")))
            min_var.set_writable()
            max_var.set_writable()
            discovery.all_nodes.append((elem_path, "Range"))

        elif model_type == "File":
            var = parent_node.add_variable(idx, id_short, str(elem.get("value", "")))
            var.set_writable()
            discovery.all_nodes.append((elem_path, "File"))

        else:
            print(f"  [SKIP] {id_short} — unsupported modelType: {model_type}")


# --------- 7. NODE FINDER---------------
def find_node(parent, *browse_names):
    node = parent
    for name in browse_names:
        found = next((c for c in node.get_children()
                      if c.get_browse_name().Name == name), None)
        if found is None:
            raise LookupError(f"Node '{name}' not found under {node}")
        node = found
    return node


# --------- 8. UPDATE AAS DICT IN MEMORY ---------------
def update_aas_property(aas_data: dict, submodel_id: str, path: tuple, new_value):
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


# ------------9. LIVE UPDATE LOOP (adapted for MG400) -------------
def live_update_loop(robot_ref, discovery, aas_data, aas_path, interval):
    """
    Polls the robot and updates every discovered pose field.
    """
    if not discovery.pose_fields:
        print("[LiveUpdate] No pose fields discovered — live pose update disabled.")
    else:
        print(f"[LiveUpdate] Tracking {len(discovery.pose_fields)} pose field(s).")

    save_counter = 0
    print("[LiveUpdate] Polling started.\n")

    while True:
        dashboard = robot_ref.get("dashboard")
        if dashboard is None:
            time.sleep(interval)
            continue
        try:
            with ROBOT_LOCK:
                raw_pose = dashboard.GetPose()
            pose = parse_pose_string(raw_pose)

            for attr, (node, path) in discovery.pose_fields.items():
                if attr not in pose:
                    continue
                live_val = round(pose[attr], 4)
                node.set_value(live_val)
                update_aas_property(aas_data, path[0], path[1:], live_val)

            save_counter += 1
            if save_counter >= 10:
                save_aas_data(aas_path, aas_data)
                save_counter = 0

        except Exception as e:
            print(f"[LiveUpdate] Error: {e}")

        time.sleep(interval)


# ------------10. ROBOT CONNECTION -------------
def connect_robot(ip: str, dashboard_port: int, move_port: int) -> dict:
    """
    Builds the robot_ref dict for MG400. Two separate sockets are opened
    because the Dobot SDK exposes control/status commands and motion
    commands through two different API classes.
    """
    dashboard = DobotApiDashboard(ip, dashboard_port)
    move = DobotApiMove(ip, move_port)
    dashboard.EnableRobot()
    return {"dashboard": dashboard, "move": move}


def main():
    aas_data = load_aas_data(AAS_PATH)
    cfg = extract_opcua_config(aas_data)
    print(f"\n[Config] Endpoint     : {cfg['endpoint']}")
    print(f"[Config] Namespace URI: {cfg['namespace_uri']}")
    print(f"[Config] Server name  : {cfg['server_name']}\n")

    server = Server()
    server.set_endpoint(cfg["endpoint"])
    server.set_server_name(cfg["server_name"])
    idx = server.register_namespace(cfg["namespace_uri"])
    objects = server.get_objects_node()
    print(f"[OPC UA] Namespace '{cfg['namespace_uri']}' registered as ns={idx}")

    robot_ref = {"dashboard": None, "move": None}
    discovery = Discovery()

    for sm in aas_data.get("submodels", []):
        sm_node = objects.add_object(idx, sm.get("idShort", "Submodel"))
        inside_cap = sm.get("idShort") == CAPABILITY_SUBMODEL
        add_elements_to_node(sm_node, sm.get("submodelElements", []), idx,
                              robot_ref, discovery, (sm.get("idShort"),),
                              inside_capability=inside_cap)

    discovery.report()

    server.start()
    print(f"OPC UA Server running at {cfg['endpoint']}")

    try:
        robot_ref.update(connect_robot(ROBOT_IP, DASHBOARD_PORT, MOVE_PORT))
        print(f"[Robot] Connected to MG400 at {ROBOT_IP}")
    except Exception as e:
        print(f"[Robot] Connection failed, running with nodes only: {e}")

    updater = threading.Thread(
        target=live_update_loop,
        args=(robot_ref, discovery, aas_data, AAS_PATH, UPDATE_HZ),
        daemon=True
    )
    updater.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Shutdown] Stopping server...")
        server.stop()


if __name__ == "__main__":
    main()
