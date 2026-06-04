import cv2
import numpy as np

# CCTV 카메라 번호
CAM1_INDEX = 2
CAM2_INDEX = 4

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 15

# ArUco 딕셔너리 설정
# 보통 4x4_50 많이 사용
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# OpenCV 버전별 Detector 처리
try:
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    NEW_ARUCO = True
except AttributeError:
    detector = None
    NEW_ARUCO = False


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


def detect_aruco(frame, cam_name):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if NEW_ARUCO:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            aruco_dict,
            parameters=aruco_params
        )

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        ids = ids.flatten()

        for i, marker_id in enumerate(ids):
            pts = corners[i][0]

            # 꼭짓점 좌표
            top_left = pts[0]
            top_right = pts[1]
            bottom_right = pts[2]
            bottom_left = pts[3]

            # 마커 중심
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            # 세운 마커 기준으로 사용할 하단 중심점
            bottom_cx = int((bottom_left[0] + bottom_right[0]) / 2)
            bottom_cy = int((bottom_left[1] + bottom_right[1]) / 2)

            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.circle(frame, (bottom_cx, bottom_cy), 6, (0, 0, 255), -1)

            text1 = f"ID:{marker_id}"
            text2 = f"center=({cx},{cy})"
            text3 = f"bottom=({bottom_cx},{bottom_cy})"

            cv2.putText(frame, text1, (cx + 10, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, text2, (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            cv2.putText(frame, text3, (cx + 10, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

            print(f"[{cam_name}] ID {marker_id} center=({cx},{cy}) bottom=({bottom_cx},{bottom_cy})")

        cv2.putText(frame, f"{cam_name}: ArUco detected {len(ids)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)
    else:
        cv2.putText(frame, f"{cam_name}: No ArUco",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)

    return frame


def main():
    cap1 = open_camera(CAM1_INDEX)
    cap2 = open_camera(CAM2_INDEX)

    if cap1 is None and cap2 is None:
        print("카메라를 하나도 열 수 없습니다.")
        return

    print("실행 중입니다. 종료하려면 q 누르세요.")

    while True:
        frame1 = None
        frame2 = None

        if cap1 is not None:
            ret1, frame1 = cap1.read()
            if ret1:
                frame1 = detect_aruco(frame1, "CCTV1")
                cv2.imshow("CCTV1 ArUco Check", frame1)

        if cap2 is not None:
            ret2, frame2 = cap2.read()
            if ret2:
                frame2 = detect_aruco(frame2, "CCTV2")
                cv2.imshow("CCTV2 ArUco Check", frame2)

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
