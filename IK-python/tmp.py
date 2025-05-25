import math
import cv2
import cv2.aruco as aruco
import os
import serial
import time
import csv

class DeltaRobot:
	"""
	R: 베이스 모터 반지름 (mm)
	r: 말단 플랫폼 반지름 (mm)
	L: 상부 링크 길이 (mm)
	l: 하부 링크 길이 (mm)
	"""
	def __init__(self, R, r, L, l):
		self.R = R
		self.r = r
		self.rf = L
		self.re = l
		# 상수
		self.sqrt3 = math.sqrt(3.0)
		self.tan30 = 1.0 / self.sqrt3
		self.sin120 =  self.sqrt3 / 2.0
		self.cos120 = -0.5
		# 플랫폼 변환
		self.f = 2.0 * R / self.tan30
		self.e = 2.0 * r / self.tan30
		
		self.angles = []

	def _angle_yz(self, x0, y0, z0):
		# 베이스·말단 플랫폼 오프셋
		y1  = -0.5 * self.f * self.tan30   # = -R
		y0p =  y0 - 0.5 * self.e * self.tan30  # = y0 - r

		# 이차 방정식 계수
		a = (x0*x0 + y0p*y0p + z0*z0
			 + self.rf*self.rf - self.re*self.re
			 - y1*y1) / (2.0 * z0)
		b = (y1 - y0p) / z0

		# 판별식
		d = -(a + b*y1)**2 + self.rf*self.rf * (b*b + 1.0)
		if d < 0:
			return None

		# 하위 분기: -sqrt(d)
		sd = math.sqrt(d)
		yj = (y1 - a*b - sd) / (b*b + 1.0)
		zj = a + b*yj

		# 각도 계산 (라디안→도)
		theta = math.degrees(math.atan2(-zj, (y1 - yj)))
		return theta

	def inverse(self, x0, y0, z0):
		thetas = []
		for (c, s) in [(1, 0),
					   (self.cos120,  self.sin120),
					   (self.cos120, -self.sin120)]:
			x = x0 * c + y0 * s
			y = -x0 * s + y0 * c
			th = self._angle_yz(x, y, z0)
			if th is None:
				return None
			thetas.append(th)
		self.angles = thetas
		return thetas
	
	def angle_to_steps(self, target_angle_deg):
		# 기본 파라미터
		step_angle = 1.8               # 스텝당 회전 각도 (도)
		microstep_division = 16        # 마이크로스테핑 비율
		gear_ratio = 10                # 감속기 비율 (출력:입력)

		# 마이크로스텝 한 스텝당 각도 (도)
		microstep_angle = step_angle / microstep_division

		# 출력축 기준 목표 각도 → 모터 기준으로 환산
		motor_angle = target_angle_deg * gear_ratio

		# 필요한 마이크로스텝 수 계산
		steps = motor_angle / microstep_angle

		return round(steps)
	
	def send_command_to(self, pico):
		# 명령어(동작 각도)를 시리얼 포트로 전송
		steps = [-self.angle_to_steps(angle) for angle in self.angles]
		command = f"m {steps[0]} {steps[1]} {steps[2]}\n".encode()
		pico.write(command)


# ArUco 딕셔너리 설정
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) #DICT_6X6_250에 해당하는 마크를 탐지

# === Step 1: Lock camera settings (Linux-specific using v4l2-ctl) ===
# os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1')  # 수동 노출 모드
# os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=80')  # 값은 50~200 범위에서 실험
# os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=0')
# os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=4000')  # 실내 기준

# 웹캠 연결 (0은 기본 웹캠)
cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
# 밝기 설정 (0.0 ~ 1.0 또는 카메라 드라이버에 따라 다름)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)

# 노출 설정 (값은 카메라/OS마다 다름. 음수면 자동 모드일 수 있음)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # 예시: 낮은 노출 (숫자는 장치에 따라 다름)

# cap.set(cv2.CAP_PROP_EXPOSURE, -6)

if not cap.isOpened():
	print("Error: Unable to access the webcam.")
	exit()

# 기구학 상수
R = 300.0  # 상부 베이스 반지름
r = 60.0   # 하부 베이스 반지름
L = 400.0  # 상부 링크 길이
l = 890.0  # 하부 링크 길이
pico = serial.Serial('/dev/cu.usbmodem11401', 115200, timeout=1)
myDeltaRobot = DeltaRobot(R=R, r=r, L=L, l=l)

detect_cnt = 0
x = 0
y = 0
z = 0
datas = []
for i in range(150, 250, 10):
	for ii in range(-100, 101, 10):
		for iii in range(-900, -700, 10):
			myDeltaRobot.inverse(ii, i, iii)
			myDeltaRobot.send_command_to(pico)
			time.sleep(2)
			for iv in range(100):
				ret, frame = cap.read()
				if not ret:
					print("Error: Unable to read from webcam.")
					break
				
				cv2.imshow("Webcam", frame)
				cv2.waitKey(1)

				# 그레이스케일로 변환
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) # ArUco 마커 탐지
				if ids is None:
					continue
				if len(ids) != 3:
					continue

				# 탐지된 마커를 표시
				aruco.drawDetectedMarkers(frame, corners)
				frame_x, frame_y = 0, 0
				temp_0x, temp_0y = 0, 0
				temp_1x, temp_1y = 0, 0
				temp_2x, temp_2y = 0, 0

				for corner, id in zip(corners, ids):
					# 꼭짓점 좌표 가져오기
					pts = corner[0]
					
					# 중심 좌표 계산
					center_x = int((pts[0][0] + pts[2][0]) / 2)
					center_y = int((pts[0][1] + pts[2][1]) / 2)
					frame_x += center_x
					frame_y += center_y
					
					# 특정 ID 처리
					if id[0] == 0:
						temp_0x, temp_0y = center_x, center_y
					if id[0] == 1:
						temp_1x, temp_1y = center_x, center_y
					if id[0] == 2:
						temp_2x, temp_2y = center_x, center_y
					
					# 정보 표시
					text = f"ID: {id[0]} Center: ({center_x}, {center_y})"
					#cv2.putText(frame, text, (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
					
				# 마커 간 거리 계산 (유클리드 거리)
				distance = ((temp_0x - temp_1x) ** 2 + (temp_0y - temp_1y) ** 2)**0.5
				distance += ((temp_0x - temp_2x) ** 2 + (temp_0y - temp_2y) ** 2)**0.5
				distance += ((temp_2x - temp_1x) ** 2 + (temp_2y - temp_1y) ** 2)**0.5
				frame_text = f"Distance: {int(distance)} Center: ({int(frame_x / 3)}, {int(frame_y / 3)})"
				cv2.putText(frame, frame_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
				# print(f"{int(distance)} {int(frame_x/3)} {int(frame_y/3)}")

				cv2.imshow("ArUco Marker Detection", frame)
				cv2.waitKey(1)

				detect_cnt += 1
				x += int(frame_x / 3)
				y += int(frame_y / 3)
				z += int(distance)
			
			if detect_cnt > 0:
				x = int(x / detect_cnt)
				y = int(y / detect_cnt)
				z = int(z / detect_cnt)
				print(f"Average Position: ({x}, {y}, {z})\tDelta Robot Position: ({ii}, {i}, -800)")
				datas.append([ii, i, -800, x, y, z])
			detect_cnt = 0
			x = 0
			y = 0
			z = 0

# CSV 파일로 저장
with open('aruco_data.csv', 'w', newline='') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(['Delta Robot Position X', 'Delta Robot Position Y', 'Delta Robot Position Z', 'Camera Position X', 'Camera Position Y', 'Camera Position Z'])
	writer.writerows(datas)


# 리소스 해제
cap.release()
cv2.destroyAllWindows()