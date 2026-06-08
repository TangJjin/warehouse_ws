# K230 / CanMV: CSI1 GC2093 + YOLO11 animal detection + UART2 output.
#
# UART packet format:
#   magic(4) + type(1) + seq(4) + length(4) + payload + crc16(2)
#   type: 1 = detection JSON, 2 = JPEG image bytes

import os
import sys
import gc
import time
import struct

try:
    import ujson as json
except Exception:
    import json

from libs.PipeLine import ScopedTiming
from libs.YOLO import YOLO11
from machine import FPIOA, UART
from media.sensor import *
from media.display import *
from media.media import *
import image


CSI_ID = 1
DISPLAY_MODE = "lcd"  # "lcd" or "hdmi"

DISPLAY_WIDTH = ALIGN_UP(800, 16)
DISPLAY_HEIGHT = 480
RGB888P_WIDTH = ALIGN_UP(640, 16)
RGB888P_HEIGHT = 480
SNAPSHOT_WIDTH = RGB888P_WIDTH
SNAPSHOT_HEIGHT = RGB888P_HEIGHT
CONTROL_WIDTH = DISPLAY_WIDTH
CONTROL_HEIGHT = DISPLAY_HEIGHT
MODEL_INPUT_SIZE = [640, 640]
CAMERA_FPS = 30

KMODEL_PATH = "/sdcard/best.kmodel"
LABELS = ["elephant", "tiger", "wolf", "peacock", "monkey"]

CONF_THRESH = 0.60
NMS_THRESH = 0.50
MAX_BOXES_NUM = 20
DEBUG_MODE = 0

UART_BAUDRATE = 921600
UART_SEND_INTERVAL_MS = 100
UART_LOG_INTERVAL_MS = 1000
RES_DEBUG_INTERVAL_MS = 1000
RES_DEBUG_MAX_ITEMS = 5
UART_PACKET_MAGIC = b"K230"
UART_PACKET_TYPE_DETECTION = 1
UART_PACKET_TYPE_JPEG = 2
SEND_EMPTY_RESULT = True

SNAPSHOT_REQUIRED_FRAMES = 3
SNAPSHOT_CENTER_STABLE_PX = 40
SNAPSHOT_LOST_RESET_FRAMES = 5
SNAPSHOT_MIN_SCORE = 0.60
SNAPSHOT_JPEG_QUALITY = 80
SNAPSHOT_OUTPUT_SIZE = 320
SNAPSHOT_ROI_SCALE = 1.5
SNAPSHOT_MIN_ROI_SIZE = 160

OSD_STATUS_X = 8
OSD_STATUS_Y = 8
OSD_STATUS_W = 520
OSD_STATUS_H = 92
OSD_COLOR_BG = (5, 8, 12)
OSD_COLOR_TEXT = (255, 255, 255)
OSD_COLOR_GREEN = (0, 220, 120)
OSD_COLOR_AMBER = (255, 180, 0)
OSD_COLOR_CYAN = (0, 210, 255)


def print_exception(exc):
    try:
        sys.print_exception(exc)
    except Exception:
        print("Exception:", exc)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def iround(v):
    return int(v + 0.5) if v >= 0 else int(v - 0.5)


def init_uart2():
    fpioa = FPIOA()
    fpioa.set_function(5, FPIOA.UART2_TXD)
    fpioa.set_function(6, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def uart_write_all(uart, data):
    total = 0
    length = len(data)
    while total < length:
        sent = uart.write(data[total:])
        if sent is None:
            sent = length - total
        if sent <= 0:
            raise RuntimeError("uart write returned 0")
        total += sent


def send_uart_packet(uart, packet_type, seq, payload):
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    header = UART_PACKET_MAGIC + struct.pack(">BII", packet_type, seq, len(payload))
    crc = crc16_ccitt(header + payload)
    packet = header + payload + struct.pack(">H", crc)
    uart_write_all(uart, packet)


def make_detection_uart_payload(payload):
    compact = {
        "valid": bool(payload.get("valid", False)),
        "label": str(payload.get("label", "")),
        "score": payload.get("score", 0.0),
        "cx": payload.get("cx", -1),
        "cy": payload.get("cy", -1),
        "err_x": payload.get("err_x", 0),
        "err_y": payload.get("err_y", 0),
        "norm_x": payload.get("norm_x", 0.0),
        "norm_y": payload.get("norm_y", 0.0),
        "confirmed": bool(payload.get("confirmed", False)),
        "stable_frames": payload.get("stable_frames", 0),
    }
    return json.dumps(compact)


def send_detection_uart(uart, seq, payload):
    send_uart_packet(uart, UART_PACKET_TYPE_DETECTION, seq, make_detection_uart_payload(payload))


def send_jpeg_uart(uart, seq, jpeg_bytes):
    send_uart_packet(uart, UART_PACKET_TYPE_JPEG, seq, jpeg_bytes)


def is_jpeg_bytes(data):
    return len(data) >= 4 and data[0] == 0xFF and data[1] == 0xD8 and data[-2] == 0xFF and data[-1] == 0xD9


def encode_image_to_jpeg(img, quality):
    source = None
    frame = None
    try:
        try:
            source = img.copy(copy_to_fb=False)
        except TypeError:
            source = img.copy()
        frame = source

        try:
            compressed = frame.compress(quality=quality)
        except TypeError:
            compressed = frame.compress(quality)
        if compressed is not None:
            frame = compressed

        data = frame.bytearray()
        if is_jpeg_bytes(data):
            return data

        try:
            compressed = frame.compressed(quality=quality)
        except TypeError:
            compressed = frame.compressed(quality)
        data = compressed.bytearray()
        if is_jpeg_bytes(data):
            return data
    except Exception:
        raise
    raise RuntimeError("jpeg encode failed")


def image_width(img):
    try:
        return img.width()
    except Exception:
        return RGB888P_WIDTH


def image_height(img):
    try:
        return img.height()
    except Exception:
        return RGB888P_HEIGHT


def make_snapshot_roi(img, payload):
    img_w = image_width(img)
    img_h = image_height(img)
    max_side = img_w if img_w < img_h else img_h

    scale_x = img_w / CONTROL_WIDTH
    scale_y = img_h / CONTROL_HEIGHT
    cx = iround(payload.get("cx", CONTROL_WIDTH // 2) * scale_x)
    cy = iround(payload.get("cy", CONTROL_HEIGHT // 2) * scale_y)
    bbox_w = iround(payload.get("bbox_w", SNAPSHOT_MIN_ROI_SIZE) * scale_x)
    bbox_h = iround(payload.get("bbox_h", SNAPSHOT_MIN_ROI_SIZE) * scale_y)

    side = iround((bbox_w if bbox_w > bbox_h else bbox_h) * SNAPSHOT_ROI_SCALE)
    side = clamp(side, SNAPSHOT_MIN_ROI_SIZE, max_side)
    cx = clamp(cx, 0, img_w - 1)
    cy = clamp(cy, 0, img_h - 1)

    x = cx - side // 2
    y = cy - side // 2
    x = clamp(x, 0, img_w - side)
    y = clamp(y, 0, img_h - side)
    return [x, y, side, side]


def make_snapshot_image(img, payload):
    roi = make_snapshot_roi(img, payload)
    scale = SNAPSHOT_OUTPUT_SIZE / roi[2]

    try:
        return img.to_rgb888(x_scale=scale, y_scale=scale, roi=roi)
    except TypeError:
        return img.to_rgb888(scale, scale, roi)
    except Exception:
        try:
            cropped = img.copy(roi=roi, copy_to_fb=False)
        except TypeError:
            cropped = img.copy(roi=roi)
        try:
            return cropped.to_rgb888(x_scale=scale, y_scale=scale)
        except TypeError:
            return cropped.to_rgb888(scale, scale)


def make_snapshot_jpeg(img, payload):
    snapshot = make_snapshot_image(img, payload)
    jpeg_data = encode_image_to_jpeg(snapshot, SNAPSHOT_JPEG_QUALITY)
    return snapshot, jpeg_data


def send_snapshot_uart(uart, seq, img, payload):
    snapshot, jpeg_data = make_snapshot_jpeg(img, payload)
    send_jpeg_uart(uart, seq, jpeg_data)
    print(
        "snapshot uart sent:",
        "seq=%d" % seq,
        "label=%s" % payload.get("label", ""),
        "size=%dx%d" % (image_width(snapshot), image_height(snapshot)),
        "bytes=",
        len(jpeg_data),
    )


def draw_status_osd(osd_img, payload, raw_detection, detection_count):
    is_valid = bool(payload.get("valid", False))
    is_confirmed = bool(payload.get("confirmed", False))
    stable_frames = payload.get("stable_frames", 0)
    if detection_count <= 0:
        stable_frames = 0

    status_color = OSD_COLOR_GREEN if is_valid else OSD_COLOR_AMBER
    status_text = "VALID" if is_valid else "WAIT"
    target = payload if is_valid else raw_detection

    if target is None:
        label = "none"
        score = 0.0
        cx = -1
        cy = -1
        norm_x = 0.0
        norm_y = 0.0
    else:
        label = str(target.get("label", ""))
        score = target.get("score", 0.0)
        cx = target.get("cx", -1)
        cy = target.get("cy", -1)
        norm_x = target.get("norm_x", 0.0)
        norm_y = target.get("norm_y", 0.0)

    osd_img.draw_rectangle(
        OSD_STATUS_X,
        OSD_STATUS_Y,
        OSD_STATUS_W,
        OSD_STATUS_H,
        color=OSD_COLOR_BG,
        thickness=1,
        fill=True,
    )
    osd_img.draw_rectangle(
        OSD_STATUS_X,
        OSD_STATUS_Y,
        OSD_STATUS_W,
        OSD_STATUS_H,
        color=status_color,
        thickness=2,
        fill=False,
    )
    osd_img.draw_string_advanced(
        OSD_STATUS_X + 12,
        OSD_STATUS_Y + 8,
        18,
        "%s valid=%d confirmed=%d stable=%d/%d count=%d"
        % (
            status_text,
            1 if is_valid else 0,
            1 if is_confirmed else 0,
            stable_frames,
            SNAPSHOT_REQUIRED_FRAMES,
            detection_count,
        ),
        color=status_color,
    )
    osd_img.draw_string_advanced(
        OSD_STATUS_X + 12,
        OSD_STATUS_Y + 36,
        17,
        "label=%s score=%.3f cx=%d cy=%d" % (label, score, cx, cy),
        color=OSD_COLOR_TEXT,
    )
    osd_img.draw_string_advanced(
        OSD_STATUS_X + 12,
        OSD_STATUS_Y + 62,
        17,
        "norm_x=%.3f norm_y=%.3f" % (norm_x, norm_y),
        color=OSD_COLOR_CYAN,
    )


def print_res_debug(res):
    print("YOLO res type:", type(res))
    try:
        print("YOLO res len:", len(res))
    except Exception:
        print("YOLO res len: n/a")
    print("YOLO res raw:", res)

    try:
        count = len(res)
        limit = RES_DEBUG_MAX_ITEMS if count > RES_DEBUG_MAX_ITEMS else count
        for i in range(limit):
            item = res[i]
            try:
                item_len = len(item)
            except Exception:
                item_len = -1
            print("YOLO res[%d] type=%s len=%d value=%s" % (i, type(item), item_len, item))
    except Exception as e:
        print("YOLO res item debug failed:", e)


def to_float(v):
    try:
        return float(v)
    except Exception:
        return float(str(v))


def class_is_valid(v):
    try:
        cls = int(to_float(v))
        return 0 <= cls < len(LABELS)
    except Exception:
        return False


def score_is_valid(v):
    try:
        score = to_float(v)
        return 0.0 <= score <= 1.0
    except Exception:
        return False


def box_is_valid(x1, y1, x2, y2):
    return x2 > x1 and y2 > y1


def make_detection(cls_id, score, x1, y1, x2, y2):
    x1 = clamp(iround(x1), 0, CONTROL_WIDTH - 1)
    y1 = clamp(iround(y1), 0, CONTROL_HEIGHT - 1)
    x2 = clamp(iround(x2), 0, CONTROL_WIDTH - 1)
    y2 = clamp(iround(y2), 0, CONTROL_HEIGHT - 1)
    if x2 <= x1 or y2 <= y1:
        return None

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    err_x = cx - (CONTROL_WIDTH // 2)
    err_y = cy - (CONTROL_HEIGHT // 2)
    norm_x = err_x / (CONTROL_WIDTH / 2)
    norm_y = err_y / (CONTROL_HEIGHT / 2)
    bbox_w = x2 - x1
    bbox_h = y2 - y1

    cls_id = int(cls_id)
    label = LABELS[cls_id] if 0 <= cls_id < len(LABELS) else "unknown"
    return {
        "valid": True,
        "class_id": cls_id,
        "label": label,
        "score": float("%.3f" % score),
        "cx": cx,
        "cy": cy,
        "err_x": err_x,
        "err_y": err_y,
        "norm_x": float("%.4f" % norm_x),
        "norm_y": float("%.4f" % norm_y),
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "bbox_w": bbox_w,
        "bbox_h": bbox_h,
        "bbox_area": bbox_w * bbox_h,
    }


def make_detection_from_xywh(cls_id, score, x, y, w, h):
    if w <= 0 or h <= 0:
        return None

    x1 = x
    y1 = y
    x2 = x + w
    y2 = y + h
    return make_detection(cls_id, score, x1, y1, x2, y2)


def parse_detection(det):
    try:
        if len(det) < 6:
            return None
    except Exception:
        return None

    try:
        if class_is_valid(det[0]) and score_is_valid(det[1]):
            cls_id = int(to_float(det[0]))
            score = to_float(det[1])
            x1 = to_float(det[2])
            y1 = to_float(det[3])
            x2 = to_float(det[4])
            y2 = to_float(det[5])
            if box_is_valid(x1, y1, x2, y2):
                return make_detection(cls_id, score, x1, y1, x2, y2)
    except Exception:
        pass

    try:
        if score_is_valid(det[4]) and class_is_valid(det[5]):
            x1 = to_float(det[0])
            y1 = to_float(det[1])
            x2 = to_float(det[2])
            y2 = to_float(det[3])
            score = to_float(det[4])
            cls_id = int(to_float(det[5]))
            if box_is_valid(x1, y1, x2, y2):
                return make_detection(cls_id, score, x1, y1, x2, y2)
    except Exception:
        pass

    return None


def parse_grouped_yolo11_detections(res):
    detections = []
    try:
        if len(res) < 3:
            return detections
        boxes = res[0]
        class_ids = res[1]
        scores = res[2]
        count = len(boxes)
        if len(class_ids) < count:
            count = len(class_ids)
        if len(scores) < count:
            count = len(scores)
    except Exception:
        return detections

    for i in range(count):
        try:
            box = boxes[i]
            if len(box) < 4:
                continue
            cls_id = int(to_float(class_ids[i]))
            score = to_float(scores[i])
            if not class_is_valid(cls_id) or not score_is_valid(score):
                continue
            det = make_detection_from_xywh(
                cls_id,
                score,
                to_float(box[0]),
                to_float(box[1]),
                to_float(box[2]),
                to_float(box[3]),
            )
            if det is not None:
                detections.append(det)
        except Exception as e:
            print("parse grouped det failed:", e)
    return detections


def parse_detections(res):
    detections = []
    if res is None:
        return detections
    try:
        count = len(res)
    except Exception:
        return detections

    grouped_detections = parse_grouped_yolo11_detections(res)
    if grouped_detections:
        return grouped_detections

    for i in range(count):
        try:
            det = parse_detection(res[i])
            if det is not None:
                detections.append(det)
        except Exception as e:
            print("parse det failed:", e)
    return detections


def best_detection(detections):
    best_det = None
    best_score = -1.0
    for det in detections:
        score = det.get("score", 0.0)
        if score > best_score:
            best_score = score
            best_det = det
    return best_det


def make_stable_state():
    return {
        "candidate": None,
        "frames": 0,
        "lost_frames": 0,
    }


def reset_stable_state(state):
    state["candidate"] = None
    state["frames"] = 0
    state["lost_frames"] = 0


def stable_candidate_matches(prev, cur):
    if prev is None or cur is None:
        return False
    if int(prev.get("class_id", -1)) != int(cur.get("class_id", -2)):
        return False
    dx = abs(int(cur.get("cx", 0)) - int(prev.get("cx", 0)))
    dy = abs(int(cur.get("cy", 0)) - int(prev.get("cy", 0)))
    return dx <= SNAPSHOT_CENTER_STABLE_PX and dy <= SNAPSHOT_CENTER_STABLE_PX


def best_stable_candidate(detections):
    best = best_detection(detections)
    if best is None:
        return None
    if best.get("score", 0.0) < SNAPSHOT_MIN_SCORE:
        return None
    return best


def update_stable_detection(state, detections):
    candidate = best_stable_candidate(detections)
    if candidate is None:
        state["lost_frames"] += 1
        if state["lost_frames"] >= SNAPSHOT_LOST_RESET_FRAMES:
            reset_stable_state(state)
        return None, state["frames"]

    state["lost_frames"] = 0
    if stable_candidate_matches(state["candidate"], candidate):
        state["frames"] += 1
    else:
        state["candidate"] = candidate
        state["frames"] = 1

    state["candidate"] = candidate
    if state["frames"] >= SNAPSHOT_REQUIRED_FRAMES:
        return candidate, state["frames"]
    return None, state["frames"]


def make_empty_payload(seq):
    return {
        "type": "animals_control",
        "schema": 1,
        "source": "k230_animals_detect_uart",
        "seq": seq,
        "timestamp_ms": time.ticks_ms(),
        "valid": False,
        "img_w": CONTROL_WIDTH,
        "img_h": CONTROL_HEIGHT,
        "count": 0,
        "label": "",
        "score": 0.0,
        "cx": -1,
        "cy": -1,
        "err_x": 0,
        "err_y": 0,
        "norm_x": 0.0,
        "norm_y": 0.0,
        "bbox_w": 0,
        "bbox_h": 0,
        "bbox_area": 0,
    }


def make_payload(seq, detections, stable_detection, stable_frames):
    if stable_detection is None:
        payload = make_empty_payload(seq)
        payload["count"] = len(detections)
        payload["confirmed"] = False
        payload["stable_frames"] = stable_frames if len(detections) > 0 else 0
        return payload
    payload = {
        "type": "animals_control",
        "schema": 1,
        "source": "k230_animals_detect_uart",
        "seq": seq,
        "timestamp_ms": time.ticks_ms(),
        "valid": True,
        "img_w": CONTROL_WIDTH,
        "img_h": CONTROL_HEIGHT,
        "count": len(detections),
        "confirmed": True,
        "stable_frames": stable_frames,
    }
    payload.update(stable_detection)
    return payload


def init_sensor():
    sensor = Sensor(
        id=CSI_ID,
        width=RGB888P_WIDTH,
        height=RGB888P_HEIGHT,
        fps=CAMERA_FPS,
    )
    sensor.reset()

    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420, chn=CAM_CHN_ID_0)

    sensor.set_framesize(width=SNAPSHOT_WIDTH, height=SNAPSHOT_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

    sensor.set_framesize(width=RGB888P_WIDTH, height=RGB888P_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    return sensor


def init_display(sensor):
    bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
    if DISPLAY_MODE == "hdmi":
        Display.init(Display.LT9611, to_ide=True)
        return [1920, 1080]
    Display.init(Display.ST7701, to_ide=True)
    return [DISPLAY_WIDTH, DISPLAY_HEIGHT]


def main():
    sensor = None
    yolo = None
    osd_img = None
    uart = None
    last_send_ms = 0
    last_uart_log_ms = 0
    last_res_debug_ms = 0
    stable_state = make_stable_state()
    last_valid = False
    snapshot_seq = 0
    seq = 0

    try:
        print("CSI1 GC2093 YOLO11 animal detection UART starting")
        uart = init_uart2()
        print("UART2 ready: tx=gpio5 rx=gpio6 baud=%d" % UART_BAUDRATE)

        sensor = init_sensor()
        display_size = init_display(sensor)
        osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

        MediaManager.init()
        sensor.run()

        yolo = YOLO11(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=[RGB888P_WIDTH, RGB888P_HEIGHT],
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONF_THRESH,
            nms_thresh=NMS_THRESH,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=DEBUG_MODE,
        )
        yolo.config_preprocess()

        print("Labels:", LABELS)
        print("Model:", KMODEL_PATH)

        while True:
            os.exitpoint()

            with ScopedTiming("total", DEBUG_MODE > 0):
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
                if rgb888p_img.format() == image.RGBP888:
                    frame = rgb888p_img.to_numpy_ref()
                    res = yolo.run(frame)
                    now_ms = time.ticks_ms()
                    if time.ticks_diff(now_ms, last_res_debug_ms) >= RES_DEBUG_INTERVAL_MS:
                        last_res_debug_ms = now_ms
                        print_res_debug(res)

                    detections = parse_detections(res)
                    stable_detection, stable_frames = update_stable_detection(stable_state, detections)
                    display_payload = make_payload(seq, detections, stable_detection, stable_frames)
                    raw_detection = best_detection(detections)

                    osd_img.clear()
                    yolo.draw_result(res, osd_img)
                    draw_status_osd(osd_img, display_payload, raw_detection, len(detections))
                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

                    current_valid = bool(display_payload.get("valid", False))
                    if current_valid and not last_valid:
                        snapshot_seq += 1
                        try:
                            photo_img = sensor.snapshot(chn=CAM_CHN_ID_1)
                            send_snapshot_uart(uart, snapshot_seq, photo_img, display_payload)
                        except Exception as e:
                            print("snapshot uart send failed:", e)
                    last_valid = current_valid

                    should_send = len(detections) > 0 or SEND_EMPTY_RESULT
                    if should_send:
                        if time.ticks_diff(now_ms, last_send_ms) >= UART_SEND_INTERVAL_MS:
                            last_send_ms = now_ms
                            seq += 1
                            payload = make_payload(seq, detections, stable_detection, stable_frames)
                            try:
                                send_detection_uart(uart, seq, payload)
                                if payload["valid"] and time.ticks_diff(now_ms, last_uart_log_ms) >= UART_LOG_INTERVAL_MS:
                                    last_uart_log_ms = now_ms
                                    print(
                                        "uart sent seq=%d stable=%d %s norm=(%.3f,%.3f) center=(%d,%d)"
                                        % (
                                            seq,
                                            payload["stable_frames"],
                                            payload["label"],
                                            payload["norm_x"],
                                            payload["norm_y"],
                                            payload["cx"],
                                            payload["cy"],
                                        )
                                    )
                            except Exception as e:
                                print("uart send failed:", e)

                gc.collect()

    except KeyboardInterrupt as e:
        print("User stopped:", e)
    except BaseException as e:
        print_exception(e)
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        if yolo:
            yolo.deinit()
        if sensor:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        gc.collect()
        time.sleep_ms(100)
        print("CSI1 GC2093 YOLO11 animal detection UART stopped")


if __name__ == "__main__":
    main()
