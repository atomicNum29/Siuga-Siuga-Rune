import numpy as np
import pandas as pd

# 데이터 로드
data = pd.read_csv("aruco_data2.csv")

# 좌표 추출
robot_points = data[['Delta Robot Position X', 'Delta Robot Position Y', 'Delta Robot Position Z']].to_numpy()
camera_points = data[['Camera Position X', 'Camera Position Y', 'Camera Position Z']].to_numpy()

# 중심점 계산
robot_center = np.mean(robot_points, axis=0)
camera_center = np.mean(camera_points, axis=0)

# 중심 정렬
robot_centered = robot_points - robot_center
camera_centered = camera_points - camera_center

# 공분산 행렬
H = robot_centered.T @ camera_centered

# SVD
U, S, Vt = np.linalg.svd(H)
R = Vt.T @ U.T

# 반사 제거
if np.linalg.det(R) < 0:
    Vt[2, :] *= -1
    R = Vt.T @ U.T

# 이동 벡터
t = camera_center - R @ robot_center

# 출력
print("R =\n", R)
print("t =\n", t)

# 변환 함수 정의
def transform_robot_to_camera(robot_point, R, t):
    return R @ robot_point + t

# R, t는 앞서 계산된 회전행렬과 이동벡터라고 가정
def transform_camera_to_robot(camera_point, R, t):
    R_inv = np.linalg.inv(R)  # 회전행렬의 역행렬
    return R_inv @ (camera_point - t)

errors = []
for r, c in zip(robot_points, camera_points):
    # c_pred = R @ r + t
    # errors.append(np.linalg.norm(c - c_pred))
    r_pred = transform_camera_to_robot(c, R, t)
    errors.append(np.linalg.norm(r - r_pred))
print("평균 변환 오차:", np.mean(errors), "mm")

r = [0, 150, -800]
print(f"transform_robot_to_camera({r}) = {transform_robot_to_camera(r, R, t)}")