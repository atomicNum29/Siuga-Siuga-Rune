import serial
import time
import math


# 기구학 상수
R = 290.0  # 상부 베이스 반지름
r = 60.0   # 하부 베이스 반지름
L = 400.0  # 상부 링크 길이
l = 890.0  # 하부 링크 길이

# 모터 위치 (회전 기준점)
sqrt3 = math.sqrt(3)
pi = math.pi
sin120 = sqrt3 / 2
cos120 = -0.5
tan30 = 1 / sqrt3

# 역기구학 함수
def delta_inverse_kinematics(x0, y0, z0):
	def calc_theta(x0, y0, z0, angle_offset):
		# 로터리 기준 프레임으로 회전 변환
		x = x0 * math.cos(angle_offset) + y0 * math.sin(angle_offset)
		y = -x0 * math.sin(angle_offset) + y0 * math.cos(angle_offset)
		y -= (R - r)

		# 2D 평면에서 역기구학 계산
		a = (x**2 + y**2 + z0**2 + L**2 - l**2) / (2 * z0)
		b = (y) / z0

		discriminant = L**2 - (a + b * y)**2 + x**2
		if discriminant < 0:
			raise ValueError("해당 위치는 작업 공간을 벗어납니다.")
		
		yj = (y - a * b - math.sqrt(discriminant)) / (b**2 + 1)
		zj = a + b * yj
		theta = math.atan2(-zj, y - yj)

		return math.degrees(theta)

	try:
		theta1 = calc_theta(x0, y0, z0, 0)
		theta2 = calc_theta(x0, y0, z0, 2 * pi / 3)
		theta3 = calc_theta(x0, y0, z0, 4 * pi / 3)
		return theta1, theta2, theta3
	except ValueError as e:
		print(e)
		return None

def angle_to_steps(target_angle_deg):
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


pico = serial.Serial('/dev/tty.usbmodem1401', 115200, timeout=1)
while True:
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
	# x, y, z = map(float, input("엔드이펙터 위치 (x, y, z)를 입력하세요 (예: 0.0 0.0 -600.0): ").split())
	
	angles = delta_inverse_kinematics(x, y, z)
	if angles:
		print(f"θ1: {angles[0]:.2f}°, θ2: {angles[1]:.2f}°, θ3: {angles[2]:.2f}°")
		# 각도를 스텝으로 변환
		steps = [-angle_to_steps(angle) for angle in angles]
		print(f"스텝 수: {steps}")
		print(f"m {steps[0]} {steps[1]} {steps[2]}\n".encode())
		pico.write(f"m {steps[0]} {steps[1]} {steps[2]}\n".encode())
	else:
		print("작업 공간을 벗어났습니다.")

# # 예시: 엔드이펙터 위치 (x, y, z) 입력
# x, y, z = 0.0, 0.0, -500.0
# # x, y, z = input("엔드이펙터 위치 (x, y, z)를 입력하세요 (예: 0.0 0.0 -600.0): ").split(' ')
# angles = delta_inverse_kinematics(x, y, z)

# if angles:
# 	print(f"θ1: {angles[0]:.2f}°, θ2: {angles[1]:.2f}°, θ3: {angles[2]:.2f}°")
# 	# 각도를 스텝으로 변환
# 	steps = [angle_to_steps(angle) for angle in angles]
# 	print(f"스텝 수: {steps}")
# else:
# 	print("작업 공간을 벗어났습니다.")