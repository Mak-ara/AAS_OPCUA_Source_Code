import cv2
import numpy as np
from opcua import Server, ua
from ultralytics import YOLO
import json
import queue
import threading

CAMERA_ID        = "camera1"
AAS_JSON_PATH    = "camera1_operational.json"      # the submodel json you posted
VIDEO_SOURCE     = 0
OPCUA_ENDPOINT   = "opc.tcp://0.0.0.0:4840/camera1/"
YOLO_MODEL_PATH  = "yolo26n.engine"
YOLO_CONF_THRESH = 0.5
FRAME_W, FRAME_H, FPS = 320, 180, 30
SHOW_WINDOW      = True
 
YOLO_CLASS_MAP = {
    0: ("red", "circle"),
    1: ("red", "square"),
    2: ("green", "circle"),
    3: ("green", "square"),
    4: ("blue", "circle"),
    5: ("blue", "square"),
}

HSV_RANGES = {
    "blue":  [(np.array([ 95, 80, 50]), np.array([130, 255, 255]))],
    "green": [(np.array([ 40, 80, 50]), np.array([ 85, 255, 255]))],
    "red":   [(np.array([  0, 80, 50]), np.array([ 10, 255, 255])),
              (np.array([170, 80, 50]), np.array([180, 255, 255]))],
}
KERNEL = np.ones((5, 5), np.uint8)

def mask_for_color(hsv, color_name):
    ranges = HSV_RANGES[color_name]
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, lo, hi)
        mask = m if mask is None else (mask | m)
    return mask

def classify_shape(contour, area_min=800):
    area = cv2.contourArea(contour)
    if area < area_min:
        return None, None
    peri = cv2.arcLength(contour, True)
    if peri <= 1e-6:
        return None, None
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    x, y, w, h = cv2.boundingRect(approx)
    if len(approx) == 4:
        ar = w / float(h)
        if 0.85 < ar < 1.15:
            return "square", (x, y, x + w, y + h)
    circularity = 4.0 * np.pi * area / (peri * peri)
    if circularity > 0.80:
        return "circle", (x, y, x + w, y + h)
    return None, None

def detect_colored_shapes(frame_bgr, area_min=800):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    detections = []
    for color in ("red", "green", "blue"):
        mask = mask_for_color(hsv, color)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            shape, box = classify_shape(c, area_min=area_min)
            if shape is None:
                continue
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            circularity = 4.0 * np.pi * area / (peri * peri + 1e-9)
            detections.append((f"{color}_{shape}", float(circularity), box))
    return detections
 
 
# ---------------------------------------------------------------------
# AAS json -> OPC-UA address space (recursive, mirrors idShort structure)
# ---------------------------------------------------------------------
def build_from_aas(parent_node, idx, elements, node_registry, prefix=""):
    """
    Walks submodelElements and creates matching OPC-UA Object/Variable
    nodes. Fills node_registry with {"dotted.idShort.path": Node} so the
    writer loop can look nodes up by name instead of hardcoded NodeIds.
    """
    for el in elements:
        name = el["idShort"]
        path = f"{prefix}.{name}" if prefix else name
 
        if el["modelType"] == "SubmodelElementList":
            child_obj = parent_node.add_object(idx, name)
            node_registry[path] = child_obj
            build_from_aas(child_obj, idx, el["value"], node_registry, prefix=path)
 
        elif el["modelType"] == "Property":
            value_type = el.get("valueType", "xs:string")
            if "float" in value_type or "double" in value_type:
                initial, vtype = float(el.get("value") or 0.0), ua.VariantType.Float
            else:
                initial, vtype = str(el.get("value") or ""), ua.VariantType.String
 
            var_node = parent_node.add_variable(idx, name, initial, vtype)
            var_node.set_writable()
            node_registry[path] = var_node
 
 
def load_submodel(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
 
 
# ---------------------------------------------------------------------
# Fusion: one winning (color, shape, cx, cy) per frame
# ---------------------------------------------------------------------
def fuse_detections(cv_dets, yolo_result):
    best_yolo = None
    if yolo_result.boxes is not None and len(yolo_result.boxes) > 0:
        confs = yolo_result.boxes.conf.tolist()
        clss = yolo_result.boxes.cls.tolist()
        xyxy = yolo_result.boxes.xyxy.tolist()
        best_i = max(range(len(confs)), key=lambda i: confs[i])
        if confs[best_i] >= YOLO_CONF_THRESH and int(clss[best_i]) in YOLO_CLASS_MAP:
            best_yolo = (confs[best_i], clss[best_i], xyxy[best_i])
 
    if best_yolo is not None:
        _, cls_id, (x1, y1, x2, y2) = best_yolo
        color, shape = YOLO_CLASS_MAP[int(cls_id)]
        return color, shape, float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)
 
    if cv_dets:
        label, _, (x1, y1, x2, y2) = max(cv_dets, key=lambda d: d[1])
        color, shape = label.split("_", 1)
        return color, shape, float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)
 
    return "", "", 0.0, 0.0  # nothing detected -> matches AAS empty defaults
 
 
# ---------------------------------------------------------------------
# Producer (vision thread) / Consumer (OPC-UA writer thread)
# ---------------------------------------------------------------------
write_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()
 
 
def vision_loop(model, cap):
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            break
 
        cv_dets = detect_colored_shapes(frame, area_min=800)
        results = model(frame, verbose=False)
        color, shape, cx, cy = fuse_detections(cv_dets, results[0])
 
        if write_queue.full():
            try:
                write_queue.get_nowait()  # keep only the freshest reading
            except queue.Empty:
                pass
        write_queue.put((color, shape, cx, cy))
 
        if SHOW_WINDOW:
            annotated = results[0].plot()
            for label, _, (x1, y1, x2, y2) in cv_dets:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, label, (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(f"{CAMERA_ID} (YOLO + CV)", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_event.set()
                break
 
 
def opcua_writer_loop(node_registry):
    color_node = node_registry["OperationalData.ObjectColor"]
    shape_node = node_registry["OperationalData.ObjectShape"]
    cx_node = node_registry["OperationalData.ObjectPosition_Center.ObjectPosition_CenterX"]
    cy_node = node_registry["OperationalData.ObjectPosition_Center.ObjectPosition_CenterY"]
 
    while not stop_event.is_set():
        try:
            color, shape, cx, cy = write_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        color_node.set_value(color, ua.VariantType.String)
        shape_node.set_value(shape, ua.VariantType.String)
        cx_node.set_value(float(cx), ua.VariantType.Float)
        cy_node.set_value(float(cy), ua.VariantType.Float)
 
 
def main():
    server = Server()
    server.set_endpoint(OPCUA_ENDPOINT)
    idx = server.register_namespace(f"http://example.com/{CAMERA_ID}")
 
    submodel = load_submodel(AAS_JSON_PATH)
    objects = server.get_objects_node()
    root_obj = objects.add_object(idx, submodel["idShort"])  # "OperationalData"
 
    node_registry = {submodel["idShort"]: root_obj}
    build_from_aas(root_obj, idx, submodel["submodelElements"], node_registry,
                    prefix=submodel["idShort"])
 
    server.start()
    print(f"[{CAMERA_ID}] OPC-UA server started at {OPCUA_ENDPOINT}")
 
    model = YOLO(YOLO_MODEL_PATH)
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FPS)
 
    writer_thread = threading.Thread(target=opcua_writer_loop, args=(node_registry,), daemon=True)
    writer_thread.start()
 
    try:
        vision_loop(model, cap)
    finally:
        stop_event.set()
        writer_thread.join(timeout=2)
        cap.release()
        cv2.destroyAllWindows()
        server.stop()
        print(f"[{CAMERA_ID}] server stopped")
 
 
if __name__ == "__main__":
    main()
 
