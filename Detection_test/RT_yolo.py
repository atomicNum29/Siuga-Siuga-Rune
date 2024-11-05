from ultralytics import YOLO
import cv2
import math
import numpy as np
import os

# 비디오 스트림 열기
url = "http://192.168.0.2:8080/video"
cap = cv2.VideoCapture(url)

# 프레임 너비 및 높이 설정
# frame_width = int(cap.get(3))
# frame_height = int(cap.get(4))

# 비디오 출력 설정
# out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (frame_width, frame_height))

# YOLO 모델을 GPU로 로드
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, f'model/yolov8m.pt')
model = YOLO(model_path).to('cuda')

# 클래스 이름 목록
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", 
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", 
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", 
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", 
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", 
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", 
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", 
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", 
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", 
              "teddy bear", "hair drier", "toothbrush"]



while True:
    success, img = cap.read()
    if not success:
        break

    # YOLO로 객체 감지 (CUDA 사용)
    results = model(img, stream=True, conf=0.7, max_det=50, imgsz=480)

    # 모델 내부 추론 시간 저장
    for result in results:
        boxes = result.boxes
    fork_class_id = 42  # COCO 데이터셋에서 "fork"에 해당하는 class ID
    fork_indices = np.where(boxes.cls.cpu().numpy() == fork_class_id)[0]  # 포크에 해당하는 인덱스 찾기
    
    annotated_img = img.copy()
    # 감지된 객체의 바운딩 박스 그리기
    # Step 5: 포크 객체만 그리기
    if len(fork_indices) > 0:
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

    # 비디오 파일에 프레임 저장
    # out.write(img)

    # 결과 이미지 표시
    cv2.imshow('Image', annotated_img)

    # '1' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 비디오 스트림 및 파일 닫기
cap.release()
# out.release()
cv2.destroyAllWindows()
