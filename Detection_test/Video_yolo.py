from ultralytics import YOLO
import cv2
import os
import time
import numpy as np

# Step 1: YOLOv8 모델 로드
model_name = 'yolov8x'
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, f'model/{model_name}.pt')
model = YOLO(model_path)

# Step 2: 영상 불러오기
video_path = os.path.join(current_dir, 'video/input/spoonvideo10.mp4')
cap = cv2.VideoCapture(video_path)

# 영상 저장을 위한 설정
output_video_path = os.path.join(current_dir, f'video/output/output_{model_name}_spoon10.mp4')
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video_path, fourcc, fps, (frame_width, frame_height))

# Step 3: COCO 데이터셋에서 'spoon' 클래스 ID로 변경 (class ID: 44 -> spoon)
spoon_class_id = 44  # COCO 데이터셋에서 "spoon"에 해당하는 class ID

# Step 4: 영상 분석 루프
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # 더 이상 프레임이 없을 경우 루프 종료

    start_time = time.time()

    # YOLOv8 모델을 사용하여 객체 감지 수행
    results = model(frame)
    boxes = results[0].boxes  # 감지된 바운딩 박스들

    # 숟가락 객체 필터링
    spoon_indices = np.where(boxes.cls.cpu().numpy() == spoon_class_id)[0]  # 숟가락에 해당하는 인덱스 찾기

    # 포크 객체만 그리기
    if len(spoon_indices) > 0:
        # 원본 프레임을 복사하여 숟가락 객체만 표시
        annotated_frame = frame.copy()

        # 숟가락 객체에 해당하는 바운딩 박스 그리기
        for i in spoon_indices:
            xyxy = boxes.xyxy[i].cpu().numpy()  # 바운딩 박스 좌표
            conf = boxes.conf[i].cpu().numpy()  # 신뢰도
            label = f'Spoon {conf:.2f}'  # 라벨 작성 (신뢰도 포함)

            # 바운딩 박스 그리기
            cv2.rectangle(annotated_frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
            # 라벨 텍스트 추가
            cv2.putText(annotated_frame, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # 수정된 프레임을 출력 비디오에 저장
        out.write(annotated_frame)
    else:
        out.write(frame)  # 숟가락을 찾지 못하면 원본 프레임 저장

    end_time = time.time() - start_time

    # 탐지된 숟가락 객체 정보 출력 (옵션)
    # for i in spoon_indices:
    #     print(f"Spoon detected at: {boxes.xyxy[i].cpu().numpy()} with confidence {boxes.conf[i].cpu().numpy()}")

    print(f"Processing time for this frame: {end_time:.2f} seconds")

# Step 5: 모든 리소스 해제
cap.release()
out.release()