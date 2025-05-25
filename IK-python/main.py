import serial
import time
import math


# 기구학 상수
R = 290.0  # 상부 베이스 반지름
r = 60.0   # 하부 베이스 반지름
L = 400.0  # 상부 링크 길이
l = 890.0  # 하부 링크 길이

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
		steps = [-self.angle_to_steps(angle) for angle in angles]
		command = f"m {steps[0]} {steps[1]} {steps[2]}\n".encode()
		pico.write(command)



pico = serial.Serial('/dev/tty.usbmodem11401', 115200, timeout=1)
myDeltaRobot = DeltaRobot(R=290.0, r=60.0, L=400.0, l=890.0)

while True:
	recieve = pico.readline()
	if recieve:
		print(recieve.decode().strip())
	# 엔드이펙터 위치 (x, y, z) 입력
	# 예시: 엔드이펙터 위치 (x, y, z) 입력
	# x, y, z = 0.0, 0.0, -600.0
	# x, y, z = input("엔드이펙터 위치 (x, y, z)를 입력하세요 (예: 0.0 0.0 -600.0): ").split(' ')
	command = input("명령어를 입력하세요 (예: m 0 0 0): ")
	if command[0] == 'm':
		# 명령어를 파싱하여 x, y, z 좌표 추출
		_, x, y, z = command.split(' ')
		x = float(x)
		y = float(y)
		z = float(z)
	elif command[0] == 'g':
		print(f"g{command[1:]}\n".encode())
		pico.write(f"g{command[1:]}\n".encode())
		continue
	elif command[0] == 'r':
		print(f"r{command[1:]}\n".encode())
		pico.write(f"r{command[1:]}\n".encode())
		continue
	# x, y, z = map(float, input("엔드이펙터 위치 (x, y, z)를 입력하세요 (예: 0.0 0.0 -600.0): ").split())
	
	angles = myDeltaRobot.inverse(x, y, z)
	if angles:
		print(f"θ1: {angles[0]:.2f}°, θ2: {angles[1]:.2f}°, θ3: {angles[2]:.2f}°")
		# 각도를 스텝으로 변환
		myDeltaRobot.send_command_to(pico)
	else:
		print("작업 공간을 벗어났습니다.")

