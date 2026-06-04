import cv2
import numpy as np
import math

# =========================
# 카메라 설정
# =========================
CAM1_INDEX = 4
CAM2_INDEX = 0

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 15

# =========================
# 트랙 실제 크기 cm
# =========================
TRACK_W_CM = 92
TRACK_H_CM = 402

# =========================
# ArUco ID별 실제 좌표 cm
#
# 현재 배치:
#
# 뒤쪽
# ID 3 -------- ID 2
#
#       트랙
#
# ID 0 -------- ID 1
# 앞쪽
# =========================
MARKER_WORLD = {
    0: (0, 0),                       # 앞쪽 왼쪽
    1: (TRACK_W_CM, 0),              # 앞쪽 오른쪽
    2: (TRACK_W_CM, TRACK_H_CM),     # 뒤쪽 오른쪽
    3: (0, TRACK_H_CM),              # 뒤쪽 왼쪽
}

# =========================
# 로버 출발 위치 cm
# 트랙 아래 중앙에서 20cm 밖에서 출발한다고 가정
# =========================
ROVER_START_X = TRACK_W_CM / 2       # 46cm
ROVER_START_Y = -20                  # 트랙 밖이면 음수 가능

# =========================
# ArUco 설정
# =========================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

try:
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    NEW_ARUCO = True
except AttributeError:
    detector = None
    NEW_ARUCO = False


latest_H = {
    "CCTV1": None,
    "CCTV2": None,
}


def open_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"[ERROR] 카메라 {index} 열기 실패")
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"[OK] 카메라 {index} 열림")
    return cap


def detect_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if NEW_ARUCO:
        corners, ids, _ = detector.detectMarkers(gray)
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

        # 세워둔 마커이므로 하단 중앙점을 바닥 기준점으로 사용
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


def draw_info(frame, cam_name, marker_points, H):
    # 마커 하단 중앙점 표시
    for marker_id, (px, py) in marker_points.items():
        px_i = int(px)
        py_i = int(py)

        cv2.circle(frame, (px_i, py_i), 8, (0, 0, 255), -1)
        cv2.putText(
            frame,
            f"ID {marker_id} bottom",
            (px_i + 10, py_i),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

    if H is not None:
        cv2.putText(
            frame,
            f"{cam_name}: Homography OK",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )
    else:
        cv2.putText(
            frame,
            f"{cam_name}: Need 4 markers",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2
        )

    return frame


def mouse_callback_cctv1(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        H = latest_H["CCTV1"]

        if H is None:
            print("[CCTV1 CLICK] Homography 없음. ArUco 4개가 모두 인식되어야 함.")
            return

        wx, wy = pixel_to_world(H, x, y)
        dist, ang = calc_distance_angle(wx, wy)

        print(
            f"[CCTV1 CLICK] pixel=({x},{y}) "
            f"-> world=({wx:.1f}, {wy:.1f}) cm, "
            f"distance={dist:.1f} cm, angle={ang:.1f} deg"
        )


def mouse_callback_cctv2(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        H = latest_H["CCTV2"]

        if H is None:
            print("[CCTV2 CLICK] Homography 없음. ArUco 4개가 모두 인식되어야 함.")
            return

        wx, wy = pixel_to_world(H, x, y)
        dist, ang = calc_distance_angle(wx, wy)

        print(
            f"[CCTV2 CLICK] pixel=({x},{y}) "
            f"-> world=({wx:.1f}, {wy:.1f}) cm, "
            f"distance={dist:.1f} cm, angle={ang:.1f} deg"
        )


def main():
    cap1 = open_camera(CAM1_INDEX)
    cap2 = open_camera(CAM2_INDEX)

    if cap1 is None and cap2 is None:
        print("카메라를 하나도 열 수 없습니다.")
        return

    cv2.namedWindow("CCTV1 Homography Check")
    cv2.namedWindow("CCTV2 Homography Check")

    cv2.setMouseCallback("CCTV1 Homography Check", mouse_callback_cctv1)
    cv2.setMouseCallback("CCTV2 Homography Check", mouse_callback_cctv2)

    print("======================================")
    print("Homography 좌표 확인 시작")
    print("화면을 마우스로 클릭하면 실제 cm 좌표가 출력됩니다.")
    print("종료: q")
    print("======================================")
    print("확인 기준:")
    print("ID 0 근처 클릭 -> world=(0, 0) 근처")
    print("ID 1 근처 클릭 -> world=(92, 0) 근처")
    print("ID 2 근처 클릭 -> world=(92, 402) 근처")
    print("ID 3 근처 클릭 -> world=(0, 402) 근처")
    print("트랙 중앙 클릭 -> world=(46, 201) 근처")
    print("======================================")

    while True:
        if cap1 is not None:
            ret1, frame1 = cap1.read()

            if ret1:
                marker_points1, corners1, ids1 = detect_markers(frame1)

                if ids1 is not None:
                    cv2.aruco.drawDetectedMarkers(frame1, corners1, ids1)

                H1 = compute_homography(marker_points1)
                latest_H["CCTV1"] = H1

                frame1 = draw_info(frame1, "CCTV1", marker_points1, H1)
                cv2.imshow("CCTV1 Homography Check", frame1)

        if cap2 is not None:
            ret2, frame2 = cap2.read()

            if ret2:
                marker_points2, corners2, ids2 = detect_markers(frame2)

                if ids2 is not None:
                    cv2.aruco.drawDetectedMarkers(frame2, corners2, ids2)

                H2 = compute_homography(marker_points2)
                latest_H["CCTV2"] = H2

                frame2 = draw_info(frame2, "CCTV2", marker_points2, H2)
                cv2.imshow("CCTV2 Homography Check", frame2)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    if cap1 is not None:
        cap1.release()

    if cap2 is not None:
        cap2.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
