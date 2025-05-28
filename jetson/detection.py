import sys

# python detection.py 1 또는 python detection.py 0 처럼 호출할 때
if len(sys.argv) > 1:
    try:
        # 1 : Initialization Code ON
        # 0 : Initialization Code OFF
        INITIAL_MODE = int(sys.argv[1])
        if(INITIAL_MODE == 1):
            print(f"Initialization Code ON")
        elif(INITIAL_MODE == 0):
            print(f"Initialization Code OFF")
        else:
            print(f"{INITIAL_MODE} is Invalid Option parameter for INITIAL_MODE")
            sys.exit(1)   # 에러 발생 시 즉시 종료
    except ValueError:
        print(f"Invalid INITIAL_MODE '{sys.argv[1]}', using default {INITIAL_MODE}")
        sys.exit(1)   # 에러 발생 시 즉시 종료

import cv2
import torch
import os
import time
from ultralytics import YOLO

# ——— 공통 설정 —————————————
CAM_ID         = 0
VIDEO_DEV      = '/dev/video2'
MEAS_CROP      = (400, 400)   # 기준 측정용 크롭
DET_CROP       = (480, 480)   # YOLO 입력용 크롭
NUM_MEASURE    = 30           # 평균 계산할 프레임 수
EXPO_MIN       = 0
EXPO_MAX       = 500
STEP           = 1            # exposure 증감량
TOLERANCE      = 1           # ±2 이내면 OK
ADJUST_INTERVAL= 5           # 몇 프레임마다 조정할지
ADJUST_DURATION= 30.0         # 초 단위, 처음 30초 동안만 조정
ADJUST_DELAY   = 0.1          # 노출 설정 후 대기(초)
STABLE_TIME    = 3.0
TARGET_GRAY    = 15.5

def initialize_exposure():
    # ——— 카메라 & 수동 노출/화이트밸런스 설정 —————————
    cap = cv2.VideoCapture(CAM_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise RuntimeError("웹캠을 열 수 없습니다.")

    # 자동 노출(off) · 자동 WB(off) 고정
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1')  # 수동 노출 모드
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=0')
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=4000')  # 실내 기준
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=brightness=50')  # 실내 기준


    # ——— YOLO 모델 로드 & 윈도우 준비 —————————
    # DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    # print(f'[INFO] Using device: {DEVICE}')
    # model = YOLO('final.pt')

    # ——— 10sec 루프: 자동 노출 조정 —————————
    if INITIAL_MODE:
        print(f"기준 그레이스케일 평균값: {TARGET_GRAY:.2f}")

        WINDOW_NAME = 'Camera'
        #cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        #cv2.resizeWindow(WINDOW_NAME, *DET_CROP)

        expo_val   = int(TARGET_GRAY)  # 일단 기준 그레이스케일 근사값으로 시작
        start_time = time.time()
        stable_start_time = None  # 안정 상태 진입 시점 기록

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            elapsed = time.time() - start_time
            
            h, w = frame.shape[:2]
            if elapsed < ADJUST_DURATION:
                meas_roi = cv2.getRectSubPix(frame, MEAS_CROP, (w/2.0, h/2.0))
                #cv2.imshow(WINDOW_NAME, meas_roi)
                gray = cv2.cvtColor(meas_roi, cv2.COLOR_BGR2GRAY)
                mean_b = gray.mean()
                diff = TARGET_GRAY - mean_b
                print(f"[AutoExpo] mean={mean_b:.1f}, target={TARGET_GRAY:.1f}, diff={diff:.1f}")

                if abs(diff) > TOLERANCE:
                    stable_start_time = None  # 안정 상태 아니므로 리셋
                    if diff > 0 and expo_val < EXPO_MAX:
                        expo_val = min(EXPO_MAX, expo_val + STEP)
                    elif diff < 0 and expo_val > EXPO_MIN:
                        expo_val = max(EXPO_MIN, expo_val - STEP)
                    os.system(f'v4l2-ctl -d {VIDEO_DEV} --set-ctrl=exposure_absolute={expo_val}')
                    time.sleep(ADJUST_DELAY)  # 조정 후 약간의 시간 대기
                else:
                    if stable_start_time is None:
                        stable_start_time = time.time()
                    elif time.time() - stable_start_time >= STABLE_TIME:
                        print("[AutoExpo] 밝기 안정적으로 유지됨. 노출 조정 종료.")
                        break  # 3초 이상 안정적이면 종료
            else:
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()