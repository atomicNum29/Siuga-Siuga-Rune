import torch
import serial
import math
import time
import numpy as np
import joblib

class CameraRobotPredictor:
    def __init__(self, model_path):
        """
        저장된 모델을 로드하여 좌표 변환을 수행하는 클래스
        
        Parameters:
        model_path: 저장된 모델 파일 경로 (.pkl)
        """
        self.load_model(model_path)
    
    def load_model(self, model_path):
        """저장된 모델을 로드"""
        try:
            model_data = joblib.load(model_path)
            self.model = model_data['model']
            self.poly_features = model_data['poly_features']
            self.scaler_input = model_data['scaler_input']
            self.scaler_output = model_data['scaler_output']
            self.degree = model_data['degree']
            self.alpha = model_data['alpha']
            self.use_ridge = model_data['use_ridge']
            self.is_fitted = model_data['is_fitted']
            
            print(f"모델 로드 성공!")
            print(f"다항식 차수: {self.degree}")
            print(f"정규화 파라미터: {self.alpha}")
            print(f"Ridge 회귀: {self.use_ridge}")
            
        except FileNotFoundError:
            print(f"모델 파일을 찾을 수 없습니다: {model_path}")
            raise
        except Exception as e:
            print(f"모델 로드 중 오류 발생: {e}")
            raise
    
    def predict(self, camera_x, camera_y, camera_z):
        """
        카메라 좌표를 로봇 좌표로 변환
        
        Parameters:
        camera_x, camera_y, camera_z: 카메라 좌표
        
        Returns:
        robot_x, robot_y, robot_z: 로봇 좌표
        """
        if not self.is_fitted:
            raise ValueError("모델이 학습되지 않았습니다.")
        
        # 입력 데이터 준비
        camera_coords = np.array([[camera_x, camera_y, camera_z]])
        
        # 전처리: 정규화
        camera_scaled = self.scaler_input.transform(camera_coords)
        
        # 다항식 특성 생성
        camera_poly = self.poly_features.transform(camera_scaled)
        
        # 예측
        robot_scaled = self.model.predict(camera_poly)
        
        # 역정규화하여 원본 스케일로 복원
        robot_coords = self.scaler_output.inverse_transform(robot_scaled)
        
        return robot_coords[0]  # [robot_x, robot_y, robot_z] 반환
    
    def predict_batch(self, camera_coords_list):
        """
        여러 카메라 좌표를 한번에 변환
        
        Parameters:
        camera_coords_list: [[x1,y1,z1], [x2,y2,z2], ...] 형태의 리스트
        
        Returns:
        robot_coords_list: [[x1,y1,z1], [x2,y2,z2], ...] 형태의 배열
        """
        camera_coords = np.array(camera_coords_list)
        
        # 전처리
        camera_scaled = self.scaler_input.transform(camera_coords)
        camera_poly = self.poly_features.transform(camera_scaled)
        
        # 예측
        robot_scaled = self.model.predict(camera_poly)
        robot_coords = self.scaler_output.inverse_transform(robot_scaled)
        
        return robot_coords

# 모델 클래스 정의 (저장할 때 쓴 것과 동일하게)
class CoordTransformer(torch.nn.Module):
    def __init__(self):
        super(CoordTransformer, self).__init__()
        self.model = torch.nn.Sequential(
            torch.nn.Linear(3, 64),
			# torch.nn.BatchNorm1d(64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 64),
			# torch.nn.BatchNorm1d(64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 3)
        )

    def forward(self, x):
        return self.model(x)

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
		# if steps[0] > 1500 or steps[1] > 1500 or steps[2] > 1500:
		# 	print("각도가 너무 큽니다. 작업 공간을 벗어났습니다.")
		# 	return
		# elif steps[0] < -5500 or steps[1] < -5500 or steps[2] < -5500:
		# 	print("각도가 너무 작습니다. 작업 공간을 벗어났습니다.")
		# 	return
		command = f"m {steps[0]} {steps[1]} {steps[2]}\n".encode()
		pico.write(command)



# 모델 불러오기
model_load_path = "best_model-new.pth"
model = CoordTransformer()
model.load_state_dict(torch.load(model_load_path))
model.eval()

def predict_robot_coords(camera_x, camera_y, camera_z):
    # 입력 데이터를 tensor로 변환하고 float 타입으로 맞춤
    input_tensor = torch.tensor([camera_x, camera_y, camera_z], dtype=torch.float32)
    # 배치 차원을 추가 (1,3)
    input_tensor = input_tensor.unsqueeze(0)
    with torch.no_grad():
        output = model(input_tensor)
    # 결과를 1차원 numpy 배열로 변환
    robot_coords = output.squeeze(0).numpy()
    return robot_coords

import serial.tools.list_ports

# 자동 포트 감지 및 연결
def auto_connect(baudrate=115200, timeout=1):
	ports = list(serial.tools.list_ports.comports())
	for port in ports:
		# 이름이나 설명에 'usbmodem' 또는 'usbserial'이 포함된 포트 우선 연결
		if 'usbmodem' in port.device or 'usbserial' in port.device or 'USB' in port.description:
			try:
				return serial.Serial(port.device, baudrate, timeout=timeout)
			except Exception:
				continue
	# 못 찾으면 첫 번째 포트 시도
	if ports:
		try:
			return serial.Serial(ports[0].device, baudrate, timeout=timeout)
		except Exception:
			pass
	raise RuntimeError("연결 가능한 시리얼 포트를 찾을 수 없습니다.")

mcu = auto_connect()
myDeltaRobot = DeltaRobot(R=300.0, r=60.0, L=400.0, l=890.0)

# model_path = "camera_robot_transform.pkl"
# predictor = CameraRobotPredictor(model_path)

while True:

	command = input("명령어를 입력하세요 (예: m 0 0 0): ")
	if command[0] == 'm':
		# 명령어를 파싱하여 x, y, z 좌표 추출
		_, x, y, z = command.split(' ')
		x = float(x)
		y = float(y)
		z = float(z)
	elif command[0] == 'g':
		print(f"g{command[1:]}\n".encode())
		mcu.write(f"g{command[1:]}\n".encode())
		continue
	elif command[0] == 'r':
		print(f"r{command[1:]}\n".encode())
		mcu.write(f"r{command[1:]}\n".encode())
		continue
	elif command[0] == 'c':
		# 명령어를 파싱하여 x, y, z 좌표 추출
		_, camera_x, camera_y, camera_z = command.split(' ')
		camera_x = float(camera_x)
		camera_y = float(camera_y)
		camera_z = float(camera_z)

		x, y, z = predict_robot_coords(camera_x, camera_y, camera_z)
		print(f"예측된 로봇 좌표: x={x:.3f}, y={y:.3f}, z={z:.3f}")

	
	angles = myDeltaRobot.inverse(x, y, z)
	if angles:
		print(f"θ1: {angles[0]:.2f}°, θ2: {angles[1]:.2f}°, θ3: {angles[2]:.2f}°")
		# 각도를 스텝으로 변환
		myDeltaRobot.send_command_to(mcu)
		time.sleep(0.1)  # 명령어 전송 후 잠시 대기
		print(mcu.readline())
	else:
		print("작업 공간을 벗어났습니다.")

