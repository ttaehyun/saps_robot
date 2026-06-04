from flask import Flask, Response, render_template_string, jsonify
import cv2
import numpy as np
import threading
import time
import math
import subprocess
import shlex
from ultralytics import YOLO

# =========================================================
# 기본 설정
# =========================================================
app = Flask(__name__)

# 현재 v4l2-ctl 기준
# ABKO CCTV 1 = /dev/video4
# ABKO CCTV 2 = /dev/video0
# HD Webcam   = /dev/video2 -> 사용 안 함
CAM1_INDEX = 2
CAM2_INDEX = 4

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_FPS = 15
JPEG_QUALITY = 70

# =========================================================
# 카메라 원본 방향 보정
# rotate: 0, 90, 180, 270
# flip: None, 0=상하반전, 1=좌우반전, -1=상하+좌우반전
# =========================================================
INPUT_ROTATE_CCTV1 = 270
INPUT_ROTATE_CCTV2 = 90

INPUT_FLIP_CCTV1 = None
INPUT_FLIP_CCTV2 = None

# =========================================================
# 화면 표시 설정
# =========================================================
USE_TRACK_MASK = True
USE_TRACK_CROP = True
TRACK_CROP_MARGIN = 25

# ArUco가 잠깐 끊겨도 마지막 기준점/호모그래피 유지 시간
MARKER_HOLD_SEC = 5.0

# 화면을 깔끔하게 보이게 하기 위한 설정
SHOW_MARKER_ID = False          # True로 바꾸면 ID 글자 표시
SHOW_TRACK_BORDER = True        # 트랙 외곽선 표시
SHOW_STATUS_TEXT = True         # 좌측 상단 Homography 상태 표시
SHOW_TARGET_PANEL = True        # 좌측 하단 Target 패널 표시

TRACK_BORDER_THICKNESS = 1
YOLO_BOX_THICKNESS = 2

# YOLO 모델 경로
YOLO_MODEL_PATH = "/home/a/Desktop/cctv/runs/detect/train-2/weights/best.pt"
YOLO_CONF = 0.05
YOLO_IMGSZ = 640

# 트랙 실제 크기 cm
TRACK_W_CM = 102
TRACK_H_CM = 412

# ArUco 배치
#
# 뒤쪽
# ID 3 -------- ID 2
#
#       트랙
#
# ID 0 -------- ID 1
# 앞쪽
MARKER_WORLD = {
    0: (0, 0),                       # 앞쪽 왼쪽
    1: (TRACK_W_CM, 0),              # 앞쪽 오른쪽
    2: (TRACK_W_CM, TRACK_H_CM),     # 뒤쪽 오른쪽
    3: (0, TRACK_H_CM),              # 뒤쪽 왼쪽
}

# 로버 출발 위치
ROVER_START_X = TRACK_W_CM / 2       # 51
ROVER_START_Y = -20

# ROS2 Docker 설정
DOCKER_CONTAINER = "ros2_humble"

# 로버 Action Server와 반드시 같은 ROS_DOMAIN_ID 사용
ROS_DOMAIN_ID = 7

# ROS2 Action 설정
# Action 파일 예시:
# string command
# float32 x
# float32 y
# ---
# bool success
# string message
# ---
# string status
# float32 progress
ROVER_ACTION_NAME = "/rover_command"
ROVER_ACTION_TYPE = "saps_interfaces/action/RoverCommand"

# =========================================================
# 전역 변수
# =========================================================
raw_frame1 = None
raw_frame2 = None

display_frame1 = None
display_frame2 = None

lock1 = threading.Lock()
lock2 = threading.Lock()

latest_H = {
    "CCTV1": None,
    "CCTV2": None,
}

latest_marker_points = {
    "CCTV1": None,
    "CCTV2": None,
}

latest_marker_time = {
    "CCTV1": 0,
    "CCTV2": 0,
}

latest_target = {
    "detected": False,
    "camera": "-",
    "x": None,
    "y": None,
    "distance": None,
    "angle": None,
    "time": None,
}

mission_state = "대기 중"
system_log = []
running = True

# =========================================================
# YOLO 모델 로드
# =========================================================
print("[INFO] YOLO 모델 로딩 중...")
model = YOLO(YOLO_MODEL_PATH)
print("[OK] YOLO 모델 로드 완료")

# =========================================================
# ArUco 설정
# =========================================================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

try:
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    NEW_ARUCO = True
except AttributeError:
    aruco_detector = None
    NEW_ARUCO = False


# =========================================================
# 유틸 함수
# =========================================================
def add_log(msg):
    global system_log

    now = time.strftime("%H:%M:%S")
    line = f"[{now}] {msg}"
    print(line)

    system_log.append(line)

    if len(system_log) > 30:
        system_log = system_log[-30:]


def open_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"[ERROR] 카메라 {index} 열기 실패")
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"[OK] 카메라 {index} 열림")
    return cap


def apply_input_orientation(frame, rotate_angle=0, flip_code=None):
    if rotate_angle == 90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_angle == 180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    elif rotate_angle == 270:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # 주의: ArUco는 거울반전되면 ID 인식이 깨질 수 있음
    if flip_code is not None:
        frame = cv2.flip(frame, flip_code)

    return frame


def detect_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if NEW_ARUCO:
        corners, ids, _ = aruco_detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            aruco_dict,
            parameters=aruco_params
        )

    marker_points = {}

    if ids is None:
        return marker_points, corners, ids

    ids = ids.flatten()

    for i, marker_id in enumerate(ids):
        pts = corners[i][0]

        # ArUco 꼭짓점 순서
        # 0: 좌상, 1: 우상, 2: 우하, 3: 좌하
        bottom_left = pts[3]
        bottom_right = pts[2]

        # 세워둔 마커라서 하단 중앙점을 바닥 기준점으로 사용
        bottom_cx = float((bottom_left[0] + bottom_right[0]) / 2)
        bottom_cy = float((bottom_left[1] + bottom_right[1]) / 2)

        marker_points[int(marker_id)] = (bottom_cx, bottom_cy)

    return marker_points, corners, ids


def compute_homography(marker_points):
    pixel_pts = []
    world_pts = []

    for marker_id, world_xy in MARKER_WORLD.items():
        if marker_id in marker_points:
            pixel_pts.append(marker_points[marker_id])
            world_pts.append(world_xy)

    if len(pixel_pts) < 4:
        return None

    pixel_pts = np.array(pixel_pts, dtype=np.float32)
    world_pts = np.array(world_pts, dtype=np.float32)

    H, _ = cv2.findHomography(pixel_pts, world_pts)
    return H


def pixel_to_world(H, px, py):
    point = np.array([[[px, py]]], dtype=np.float32)
    world = cv2.perspectiveTransform(point, H)

    x = float(world[0][0][0])
    y = float(world[0][0][1])

    return x, y


def calc_distance_angle(x, y):
    dx = x - ROVER_START_X
    dy = y - ROVER_START_Y

    distance = math.sqrt(dx * dx + dy * dy)
    angle = math.degrees(math.atan2(dy, dx))

    return distance, angle


def is_inside_track_world(x, y):
    return 0 <= x <= TRACK_W_CM and 0 <= y <= TRACK_H_CM


def get_track_polygon(marker_points):
    required_ids = [0, 1, 2, 3]

    if not all(mid in marker_points for mid in required_ids):
        return None

    pts = np.array([
        marker_points[0],
        marker_points[1],
        marker_points[2],
        marker_points[3]
    ], dtype=np.int32)

    return pts.reshape((-1, 1, 2))


def mask_track_region(frame, marker_points, crop_mode=False, margin=10):
    poly = get_track_polygon(marker_points)

    if poly is None:
        return frame, None, (0, 0)

    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [poly], 255)

    masked = cv2.bitwise_and(frame, frame, mask=mask)

    offset_x = 0
    offset_y = 0

    if crop_mode:
        x, y, w, h = cv2.boundingRect(poly)

        x = max(0, x - margin)
        y = max(0, y - margin)
        w = min(frame.shape[1] - x, w + margin * 2)
        h = min(frame.shape[0] - y, h + margin * 2)

        masked = masked[y:y+h, x:x+w]
        offset_x = x
        offset_y = y

    return masked, poly, (offset_x, offset_y)


def run_ros2_in_docker(ros_cmd, timeout=120):
    """
    Flask 서버에서 Docker 컨테이너 내부 ROS2 명령을 실행한다.
    서버 GUI 버튼 클릭 -> Docker 안에서 ros2 action send_goal 실행. 
    """
    try:
        full_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi && "
            "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi && "
            f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
            f"{ros_cmd}"
        )

        result = subprocess.run(
            ["sudo", "docker", "exec", DOCKER_CONTAINER, "bash", "-lc", full_cmd],
            capture_output=True,
            text=True,
            timeout=timeout
        )

        ok = result.returncode == 0
        return ok, result.stdout, result.stderr

    except subprocess.TimeoutExpired:
        return False, "", "ROS2 Action timeout"

    except Exception as e:
        return False, "", str(e)


def send_rover_action(command, x=0.0, y=0.0, timeout=120):
    """
    로버 Action Server로 Goal 전송.

    command:
      start          : x, y 돌 좌표까지 이동 후 집기
      return         : 좌표 없이 원래 위치로 복귀
      emergency_stop : 즉시 정지

    x, y:
      start일 때만 사용.
      return / emergency_stop은 0.0, 0.0 전송.
    """
    goal = (
        "{"
        f"command: '{command}', "
        f"x: {float(x)}, "
        f"y: {float(y)}"
        "}"
    )

    ros_cmd = (
        f"ros2 action send_goal "
        f"{ROVER_ACTION_NAME} "
        f"{ROVER_ACTION_TYPE} "
        f"{shlex.quote(goal)} "
        f"--feedback"
    )

    add_log(f"Action 요청 시작: command={command}, x={x}, y={y}")

    ok, stdout, stderr = run_ros2_in_docker(ros_cmd, timeout=timeout)

    if ok:
        add_log(f"Action 전송 완료: command={command}, x={x}, y={y}")

        # 로그가 너무 길어지는 것을 막기 위해 마지막 몇 줄만 표시
        if stdout.strip():
            lines = stdout.strip().splitlines()
            for line in lines[-5:]:
                add_log(line)

    else:
        add_log(f"Action 전송 실패: command={command}")

        if stderr.strip():
            for line in stderr.strip().splitlines()[-5:]:
                add_log(line)

        if stdout.strip():
            for line in stdout.strip().splitlines()[-5:]:
                add_log(line)

    return ok


def send_rover_action_async(command, x=0.0, y=0.0, timeout=120):
    """
    Flask 화면이 멈추지 않도록 Action 전송을 별도 스레드에서 실행한다.
    """
    thread = threading.Thread(
        target=send_rover_action,
        args=(command, x, y, timeout),
        daemon=True
    )
    thread.start()
    return True

def update_latest_target(cam_name, rock_x, rock_y, distance, angle):
    global latest_target

    latest_target = {
        "detected": True,
        "camera": cam_name,
        "x": round(rock_x, 1),
        "y": round(rock_y, 1),
        "distance": round(distance, 1),
        "angle": round(angle, 1),
        "time": time.strftime("%H:%M:%S"),
    }


def draw_text_with_bg(frame, text, org, font_scale=0.55, color=(255, 255, 255), thickness=1):
    x, y = org
    font = cv2.FONT_HERSHEY_SIMPLEX

    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    pad = 5

    overlay = frame.copy()
    cv2.rectangle(
        overlay,
        (x - pad, y - th - pad),
        (x + tw + pad, y + baseline + pad),
        (0, 0, 0),
        -1
    )
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    cv2.putText(frame, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def draw_status_info(frame, cam_name, H, using_hold=False):
    if not SHOW_STATUS_TEXT:
        return

    if H is not None:
        if using_hold:
            text = f"{cam_name}  HOMOGRAPHY HOLD"
            color = (0, 255, 255)
        else:
            text = f"{cam_name}  HOMOGRAPHY OK"
            color = (0, 255, 0)
    else:
        text = f"{cam_name}  NEED 4 ARUCO"
        color = (0, 0, 255)

    draw_text_with_bg(frame, text, (16, 28), font_scale=0.55, color=color, thickness=1)


def draw_marker_points(frame, marker_points, offset=(0, 0)):
    offset_x, offset_y = offset

    for marker_id, (px, py) in marker_points.items():
        px_i = int(px - offset_x)
        py_i = int(py - offset_y)

        if 0 <= px_i < frame.shape[1] and 0 <= py_i < frame.shape[0]:
            # 작은 빨간 점만 표시
            cv2.circle(frame, (px_i, py_i), 5, (0, 0, 255), -1)

            if SHOW_MARKER_ID:
                cv2.putText(
                    frame,
                    f"ID {marker_id}",
                    (px_i + 6, py_i - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 0),
                    1
                )


def draw_target_panel(frame, target_info):
    if not SHOW_TARGET_PANEL:
        return

    h, w = frame.shape[:2]

    panel_w = 260
    panel_h = 105
    x0 = 14
    y0 = h - panel_h - 14

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.60, frame, 0.40, 0, frame)

    cv2.rectangle(frame, (x0, y0), (x0 + panel_w, y0 + panel_h), (70, 130, 180), 1)

    if target_info is None:
        lines = [
            ("TARGET", (255, 255, 255)),
            ("No target detected", (0, 255, 255)),
        ]
    else:
        lines = [
            ("TARGET DETECTED", (0, 255, 0)),
            (f"x={target_info['x']:.1f}cm   y={target_info['y']:.1f}cm", (255, 255, 255)),
            (f"d={target_info['distance']:.1f}cm   a={target_info['angle']:.1f}deg", (255, 255, 255)),
            (f"conf={target_info['conf']:.2f}", (255, 255, 255)),
        ]

    y = y0 + 24
    for text, color in lines:
        cv2.putText(
            frame,
            text,
            (x0 + 12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            1,
            cv2.LINE_AA
        )
        y += 23


def draw_yolo_and_target(frame_for_display, frame_for_yolo, cam_name, H, offset=(0, 0)):
    global latest_target

    if H is None:
        draw_target_panel(frame_for_display, None)
        return frame_for_display

    offset_x, offset_y = offset
    target_info = None

    try:
        results = model.predict(
            frame_for_yolo,
            conf=YOLO_CONF,
            imgsz=YOLO_IMGSZ,
            verbose=False
        )

        best_det = None
        best_conf = -1.0

        for r in results:
            boxes = r.boxes

            if boxes is None:
                continue

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                x1_i = int(x1)
                y1_i = int(y1)
                x2_i = int(x2)
                y2_i = int(y2)

                cx_local = int((x1_i + x2_i) / 2)
                cy_local = int((y1_i + y2_i) / 2)

                cx_original = cx_local + offset_x
                cy_original = cy_local + offset_y

                rock_x, rock_y = pixel_to_world(H, cx_original, cy_original)

                if not is_inside_track_world(rock_x, rock_y):
                    continue

                if conf > best_conf:
                    best_conf = conf
                    best_det = {
                        "x1": x1_i,
                        "y1": y1_i,
                        "x2": x2_i,
                        "y2": y2_i,
                        "cx": cx_local,
                        "cy": cy_local,
                        "rock_x": rock_x,
                        "rock_y": rock_y,
                        "conf": conf
                    }

        if best_det is not None:
            x1_i = best_det["x1"]
            y1_i = best_det["y1"]
            x2_i = best_det["x2"]
            y2_i = best_det["y2"]
            cx = best_det["cx"]
            cy = best_det["cy"]
            rock_x = best_det["rock_x"]
            rock_y = best_det["rock_y"]
            conf = best_det["conf"]

            distance, angle = calc_distance_angle(rock_x, rock_y)
            update_latest_target(cam_name, rock_x, rock_y, distance, angle)

            target_info = {
                "x": rock_x,
                "y": rock_y,
                "distance": distance,
                "angle": angle,
                "conf": conf,
            }

            # YOLO 박스는 얇게, 라벨만 간단히
            cv2.rectangle(
                frame_for_display,
                (x1_i, y1_i),
                (x2_i, y2_i),
                (0, 255, 255),
                YOLO_BOX_THICKNESS
            )
            cv2.circle(frame_for_display, (cx, cy), 4, (0, 0, 255), -1)

            label = f"rock {conf:.2f}"
            draw_text_with_bg(
                frame_for_display,
                label,
                (x1_i, max(20, y1_i - 8)),
                font_scale=0.48,
                color=(0, 255, 255),
                thickness=1
            )

    except Exception as e:
        draw_text_with_bg(
            frame_for_display,
            f"YOLO ERROR: {e}",
            (20, 65),
            font_scale=0.5,
            color=(0, 0, 255),
            thickness=1
        )

    draw_target_panel(frame_for_display, target_info)
    return frame_for_display


# =========================================================
# 카메라 스레드
# =========================================================
def camera_loop(cam_index, cam_name):
    global raw_frame1, raw_frame2

    cap = open_camera(cam_index)

    if cap is None:
        add_log(f"{cam_name} 열기 실패")
        return

    while running:
        for _ in range(2):
            cap.grab()

        ret, frame = cap.retrieve()

        if not ret:
            time.sleep(0.01)
            continue

        if cam_name == "CCTV1":
            with lock1:
                raw_frame1 = frame.copy()
        else:
            with lock2:
                raw_frame2 = frame.copy()

        time.sleep(0.005)

    cap.release()


# =========================================================
# 영상 처리
# =========================================================
def process_single_frame(frame, cam_name):
    marker_points, corners, ids = detect_markers(frame)
    now = time.time()
    using_hold = False

    has_all_markers = all(mid in marker_points for mid in [0, 1, 2, 3])

    if has_all_markers:
        latest_marker_points[cam_name] = marker_points.copy()
        latest_marker_time[cam_name] = now

        H = compute_homography(marker_points)
        if H is not None:
            latest_H[cam_name] = H
    else:
        H = None

        # ArUco가 잠깐 끊기면 마지막 정상 marker_points와 H 사용
        if latest_marker_points[cam_name] is not None:
            elapsed = now - latest_marker_time[cam_name]

            if elapsed <= MARKER_HOLD_SEC:
                marker_points = latest_marker_points[cam_name].copy()
                H = latest_H[cam_name]
                using_hold = True

    if H is None:
        H = latest_H[cam_name]

    if USE_TRACK_MASK:
        display_frame, poly, offset = mask_track_region(
            frame,
            marker_points,
            crop_mode=USE_TRACK_CROP,
            margin=TRACK_CROP_MARGIN
        )

        yolo_frame = display_frame.copy()

        if poly is not None and SHOW_TRACK_BORDER and not USE_TRACK_CROP:
            cv2.polylines(
                display_frame,
                [poly],
                True,
                (0, 255, 255),
                TRACK_BORDER_THICKNESS
            )
    else:
        display_frame = frame.copy()
        yolo_frame = frame.copy()
        offset = (0, 0)
        poly = None

    # crop 화면에서는 poly 좌표가 원본 기준이라 선은 생략하고, 작은 점만 표시
    draw_marker_points(display_frame, marker_points, offset)
    draw_status_info(display_frame, cam_name, H, using_hold=using_hold)

    display_frame = draw_yolo_and_target(
        display_frame,
        yolo_frame,
        cam_name,
        H,
        offset
    )

    return display_frame


def process_loop():
    global raw_frame1, raw_frame2, display_frame1, display_frame2

    while running:
        frame1 = None
        frame2 = None

        with lock1:
            if raw_frame1 is not None:
                frame1 = raw_frame1.copy()

        with lock2:
            if raw_frame2 is not None:
                frame2 = raw_frame2.copy()

        if frame1 is not None:
            frame1 = apply_input_orientation(
                frame1,
                INPUT_ROTATE_CCTV1,
                INPUT_FLIP_CCTV1
            )

            processed1 = process_single_frame(frame1, "CCTV1")

            with lock1:
                display_frame1 = processed1.copy()

        if frame2 is not None:
            frame2 = apply_input_orientation(
                frame2,
                INPUT_ROTATE_CCTV2,
                INPUT_FLIP_CCTV2
            )

            processed2 = process_single_frame(frame2, "CCTV2")

            with lock2:
                display_frame2 = processed2.copy()

        time.sleep(0.03)


# =========================================================
# 스트리밍
# =========================================================
def generate_frames(cam_name):
    global display_frame1, display_frame2

    while True:
        if cam_name == "CCTV1":
            with lock1:
                frame = None if display_frame1 is None else display_frame1.copy()
        else:
            with lock2:
                frame = None if display_frame2 is None else display_frame2.copy()

        if frame is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                f"{cam_name} Loading...",
                (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2
            )
            frame = blank

        ret, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )

        if not ret:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buffer.tobytes() +
            b"\r\n"
        )


# =========================================================
# HTML
# =========================================================
HTML_PAGE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>ITS Rover Control Server</title>
    <style>
        body {
            margin: 0;
            background: #0d1117;
            color: white;
            font-family: Arial, sans-serif;
        }

        header {
            background: #161b22;
            padding: 14px 24px;
            font-size: 24px;
            font-weight: bold;
            border-bottom: 2px solid #30363d;
        }

        .main {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            padding: 12px;
        }

        .panel {
            background: #161b22;
            border-radius: 10px;
            padding: 10px;
            border: 1px solid #30363d;
            box-shadow: 0 0 10px rgba(0,0,0,0.25);
        }

        .panel h2 {
            margin: 0 0 8px 0;
            font-size: 20px;
            color: #93c5fd;
        }

        .video {
            width: 100%;
            height: 560px;
            object-fit: contain;
            border-radius: 8px;
            border: 1px solid #30363d;
            background: black;
        }

        .bottom {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 12px;
            padding: 0 12px 12px 12px;
        }

        button {
            padding: 12px 16px;
            margin: 5px;
            font-size: 15px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            color: white;
            font-weight: bold;
        }

        .dispatch { background: #16a34a; }
        .stop { background: #dc2626; }
        .grasp { background: #2563eb; }
        .return { background: #7c3aed; }
        .safety { background: #f97316; }
        .target { background: #0891b2; }

        .status-box {
            line-height: 1.8;
            font-size: 15px;
        }

        .log {
            background: #010409;
            height: 160px;
            overflow-y: auto;
            padding: 10px;
            border-radius: 8px;
            font-size: 13px;
            color: #d1d5db;
            border: 1px solid #30363d;
        }

        .good { color: #22c55e; }
        .warn { color: #facc15; }
        .bad { color: #ef4444; }
    </style>
</head>
<body>
    <header>
        ITS 낙하물 감지 및 로버 제어 서버
    </header>

    <div class="main">
        <div class="panel">
            <h2>CCTV 1</h2>
            <img class="video" src="/video_feed1">
        </div>

        <div class="panel">
            <h2>CCTV 2</h2>
            <img class="video" src="/video_feed2">
        </div>
    </div>

    <div class="bottom">
        <div class="panel">
            <h2>로버 제어</h2>
            <button class="dispatch" onclick="sendCommand('start')">출발</button>
            <button class="return" onclick="sendCommand('return')">복귀</button>
            <button class="safety" onclick="sendCommand('emergency_stop')">긴급정지</button>
        </div>

        <div class="panel">
            <h2>현재 Target</h2>
            <div class="status-box" id="targetBox">
                불러오는 중...
            </div>
        </div>

        <div class="panel">
            <h2>시스템 로그</h2>
            <div class="log" id="logBox"></div>
        </div>
    </div>

    <script>
        function sendCommand(cmd) {
            fetch('/cmd/' + cmd)
                .then(res => res.json())
                .then(data => {
                    console.log(data);
                    updateStatus();
                });
        }

        function sendTarget() {
            fetch('/send_target')
                .then(res => res.json())
                .then(data => {
                    console.log(data);
                    updateStatus();
                });
        }

        function updateStatus() {
            fetch('/status')
                .then(res => res.json())
                .then(data => {
                    let t = data.target;
                    let html = "";

                    if (t.detected) {
                        html += "<span class='good'>낙하물 감지됨</span><br>";
                        html += "Camera: " + t.camera + "<br>";
                        html += "X: " + t.x + " cm<br>";
                        html += "Y: " + t.y + " cm<br>";
                        html += "Distance: " + t.distance + " cm<br>";
                        html += "Angle: " + t.angle + " deg<br>";
                        html += "Time: " + t.time + "<br>";
                    } else {
                        html += "<span class='warn'>감지된 Target 없음</span><br>";
                    }

                    html += "<hr>";
                    html += "Mission: " + data.mission_state + "<br>";

                    document.getElementById("targetBox").innerHTML = html;

                    let logs = data.logs;
                    let logHtml = "";
                    for (let i = logs.length - 1; i >= 0; i--) {
                        logHtml += logs[i] + "<br>";
                    }
                    document.getElementById("logBox").innerHTML = logHtml;
                });
        }

        setInterval(updateStatus, 1000);
        updateStatus();
    </script>
</body>
</html>
"""


# =========================================================
# Flask Route
# =========================================================
@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/video_feed1")
def video_feed1():
    return Response(
        generate_frames("CCTV1"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/video_feed2")
def video_feed2():
    return Response(
        generate_frames("CCTV2"),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/status")
def status():
    return jsonify({
        "target": latest_target,
        "mission_state": mission_state,
        "logs": system_log[-20:]
    })


@app.route("/send_target")
def send_target():
    """
    현재 구조에서는 Target 전송 버튼을 사용하지 않는다.
    출발 버튼을 누르면 latest_target의 x, y 좌표가 start Action에 같이 들어간다.
    """
    if not latest_target["detected"]:
        add_log("Target 확인 실패: 감지된 낙하물 없음")
        return jsonify({
            "ok": False,
            "msg": "감지된 낙하물이 없습니다."
        })

    return jsonify({
        "ok": True,
        "msg": "Target은 출발 버튼을 누를 때 자동으로 전송됩니다.",
        "target": latest_target
    })


@app.route("/cmd/<command>")
def cmd(command):
    global mission_state

    valid_commands = ["start", "return", "emergency_stop"]

    if command not in valid_commands:
        return jsonify({
            "ok": False,
            "msg": "잘못된 명령"
        })

    # 1) 출발: 돌 좌표를 같이 전송
    # 로버 쪽에서는 command == "start"를 받으면
    # x, y 좌표까지 이동한 뒤 집게로 집는 동작까지 수행하면 된다.
    if command == "start":
        if not latest_target["detected"]:
            add_log("출발 실패: 감지된 돌 좌표 없음")
            return jsonify({
                "ok": False,
                "msg": "감지된 돌 좌표가 없습니다."
            })

        x = latest_target["x"]
        y = latest_target["y"]

        send_rover_action_async("start", x, y, timeout=120)

        mission_state = "출발: 돌 좌표로 이동 및 집기"
        add_log(f"GUI 출발 요청: start, x={x}, y={y}")

        return jsonify({
            "ok": True,
            "command": "start",
            "x": x,
            "y": y,
            "mission_state": mission_state
        })

    # 2) 복귀: 좌표 없이 원래 위치로 복귀
    elif command == "return":
        send_rover_action_async("return", 0.0, 0.0, timeout=120)

        mission_state = "복귀 중"
        add_log("GUI 복귀 요청: return")

        return jsonify({
            "ok": True,
            "command": "return",
            "mission_state": mission_state
        })

    # 3) 긴급정지: 좌표 없이 즉시 정지
    elif command == "emergency_stop":
        send_rover_action_async("emergency_stop", 0.0, 0.0, timeout=30)

        mission_state = "긴급 정지"
        add_log("GUI 긴급정지 요청: emergency_stop")

        return jsonify({
            "ok": True,
            "command": "emergency_stop",
            "mission_state": mission_state
        })


# =========================================================
# 실행
# =========================================================
if __name__ == "__main__":
    add_log("서버 시작")

    t1 = threading.Thread(target=camera_loop, args=(CAM1_INDEX, "CCTV1"), daemon=True)
    t2 = threading.Thread(target=camera_loop, args=(CAM2_INDEX, "CCTV2"), daemon=True)
    tp = threading.Thread(target=process_loop, daemon=True)

    t1.start()
    time.sleep(0.5)

    t2.start()
    time.sleep(0.5)

    tp.start()

    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
