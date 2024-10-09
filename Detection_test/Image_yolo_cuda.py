from ultralytics import YOLO
import cv2
import os
import json
import time
import numpy as np
import matplotlib.pyplot as plt



# Step 1: YOLOv8 모델 로드
# 0 = nano, 1 = small, 2 = medium, 3 = large, 4 = extra-large
model_list = ['yolov8n', 'yolov8s', 'yolov8m', 'yolov8l', 'yolov8x']
model_name = model_list[2]
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, f'model/{model_name}.pt')

# run detection using cuda
model = YOLO(model_path).to('cuda')

# Step 2: 이미지 불러오기
image_name = 'fork2'
image_path = os.path.join(current_dir, f'image/input/{image_name}.jpg')  # 분석할 이미지 경로 설정
img = cv2.imread(image_path)

# Detection time start
detect_start_time = time.time()

# Step 3: YOLOv8 모델을 사용하여 객체 감지 수행
results = model(img)

# Detection time end & calculate detect time
detect_time = time.time() - detect_start_time

# 모델 내부 추론 시간 저장
inference_time = results[0].speed['inference']

# Step 4: 포크 객체 필터링 (class ID: 42 -> fork)
fork_class_id = 42  # COCO 데이터셋에서 "fork"에 해당하는 class ID
boxes = results[0].boxes  # 감지된 바운딩 박스들
fork_indices = np.where(boxes.cls.cpu().numpy() == fork_class_id)[0]  # 포크에 해당하는 인덱스 찾기

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
output_dir = os.path.join(current_dir, f'image/output/{image_name}')  # 결과 이미지 저장 폴더
output_path = os.path.join(output_dir, f'{image_name}_{model_name}.jpg')  # 결과 이미지 저장 경로

# 폴더가 존재하지 않으면 생성
os.makedirs(output_dir, exist_ok=True)

cv2.imwrite(output_path, annotated_img)  # 이미지 저장

# Step 7: 탐지된 포크 객체 정보 출력 (옵션)
for i in fork_indices:
    print(f"Fork detected at: {boxes.xyxy[i].cpu().numpy()} with confidence {boxes.conf[i].cpu().numpy()}")

timeList = {'inference_time' : f'{round(inference_time / 1000, 4)}s',
            'code_time' : f'{round(detect_time, 4)}s'}
print(f"infer : {timeList['inference_time']}, code : {timeList['code_time']}")

# 추론시간 및 코드상 검출 시간 저장
json_path = os.path.join(output_dir, f'{image_name}_cuda.json')

# JSON 있는 지 확인
if os.path.exists(json_path):
    with open(json_path, 'r') as json_file:
        data = json.load(json_file)
# 처음 받는 input_data면 빈 딕셔너리로 시작
else:
    data = {key: None for key in model_list}

# 입력 모델에 대한 데이터가 있는 지 확인
if data[model_name] != None:
    data[model_name].append(timeList)
# 이미 있으면 결과 이어서 추가
else:
    data[model_name] = [timeList]

# JSON 파일 저장
with open(json_path, 'w') as json_file:
    json.dump(data, json_file, indent=4)
