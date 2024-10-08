from ultralytics import YOLO
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt

# Step 1: YOLOv8 모델 로드
model = YOLO('yolov8s.pt')  # YOLOv8 small 모델

# Step 2: 이미지 불러오기
image_path = '180_fork1.jpg'  # 분석할 이미지 경로 설정
img = cv2.imread(image_path)

start_time = time.time()

# Step 3: YOLOv8 모델을 사용하여 객체 감지 수행
results = model(img)

model_time = time.time() - start_time

# Step 4: 포크 객체 필터링 (class ID: 42 -> fork)
fork_class_id = 42  # COCO 데이터셋에서 "fork"에 해당하는 class ID
boxes = results[0].boxes  # 감지된 바운딩 박스들
fork_indices = np.where(boxes.cls.cpu().numpy() == fork_class_id)[0]  # 포크에 해당하는 인덱스 찾기

end_time = time.time() - start_time

# Step 5: 포크 객체만 그리기
if len(fork_indices) > 0:
    # 원본 이미지를 복사하여 포크 객체만 표시
    annotated_img = img.copy()

    # 포크 객체에 해당하는 바운딩 박스 그리기
    for i in fork_indices:
        xyxy = boxes.xyxy[i].cpu().numpy()  # 바운딩 박스 좌표
        conf = boxes.conf[i].cpu().numpy()  # 신뢰도
        label = f'Fork {conf:.2f}'  # 라벨 작성 (신뢰도 포함)

        # 바운딩 박스 그리기
        cv2.rectangle(annotated_img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
        # 라벨 텍스트 추가
        cv2.putText(annotated_img, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
else:
    print("포크 객체를 찾을 수 없습니다.")

# Step 6: 포크 객체만 그려진 결과 이미지 저장
output_path = 'output_fork_only.jpg'  # 결과 이미지 저장 경로
cv2.imwrite(output_path, annotated_img)  # 이미지 저장

# Step 7: 탐지된 포크 객체 정보 출력 (옵션)
for i in fork_indices:
    print(f"Fork detected at: {boxes.xyxy[i].cpu().numpy()} with confidence {boxes.conf[i].cpu().numpy()}")

print(model_time, end_time)