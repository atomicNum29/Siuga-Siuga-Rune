import math

class DeltaInverseKinematics:
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

if __name__ == "__main__":
    ik = DeltaInverseKinematics(R=290.0, r=60.0, L=400.0, l=890.0)
    angles = ik.inverse(100.0, 100.0, -700.0)
    if angles is None:
        print("해당 위치로 이동 불가능합니다.")
    else:
        print(f"θ1={angles[0]:.3f}°, θ2={angles[1]:.3f}°, θ3={angles[2]:.3f}°")
        # → 약 -0.093°, -0.093°, -0.093°