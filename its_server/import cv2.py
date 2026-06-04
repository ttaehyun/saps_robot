import cv2
import os
import numpy as np

# =========================
# 영상 경로
# =========================

video_paths = [
    "/home/a/Downloads/1.mp4",
    "/home/a/Downloads/2.mp4",
    "/home/a/Downloads/3.mp4",
    "/home/a/Downloads/4.mp4",
]

# =========================
# 저장 위치: 바탕화면
# =========================

save_dir = "/home/a/Desktop/stone_new_200_images"
total_images = 200

os.makedirs(save_dir, exist_ok=True)

# =========================
# 영상 정보 확인
# =========================

video_infos = []

for video_path in video_paths:
    print(f"확인 중: {video_path}")

    if not os.path.exists(video_path):
        print(f"[오류] 파일 없음: {video_path}")
        continue

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"[오류] 영상 열기 실패: {video_path}")
        continue

    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"프레임 수: {frame_count}")

    if frame_count <= 0:
        print(f"[오류] 프레임 수 확인 실패: {video_path}")
        cap.release()
        continue

    video_infos.append((video_path, frame_count))
    cap.release()

if len(video_infos) == 0:
    print("사용 가능한 영상이 없습니다.")
    exit()

# =========================
# 영상별 추출 장수 계산
# 4개 영상이면 50장씩
# =========================

num_videos = len(video_infos)
base_count = total_images // num_videos
remainder = total_images % num_videos

extract_counts = []

for i in range(num_videos):
    count = base_count
    if i < remainder:
        count += 1
    extract_counts.append(count)

# =========================
# 프레임 추출
# =========================

image_number = 1

for i, (video_path, frame_count) in enumerate(video_infos):
    count = extract_counts[i]

    print("==============================")
    print(f"영상: {video_path}")
    print(f"전체 프레임 수: {frame_count}")
    print(f"추출 예정 장수: {count}")

    cap = cv2.VideoCapture(video_path)

    frame_indices = np.linspace(0, frame_count - 1, count, dtype=int)

    saved_count = 0

    for frame_idx in frame_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(frame_idx))

        ret, frame = cap.read()

        if not ret:
            print(f"[경고] 프레임 읽기 실패: {video_path}, frame={frame_idx}")
            continue

        save_path = os.path.join(save_dir, f"{image_number}.jpg")

        success = cv2.imwrite(save_path, frame)

        if success:
            saved_count += 1
            image_number += 1
        else:
            print(f"[오류] 이미지 저장 실패: {save_path}")

    cap.release()

    print(f"저장 완료: {saved_count}장")

print("==============================")
print(f"전체 저장 완료: {image_number - 1}장")
print(f"저장 위치: {save_dir}")
print("==============================")