import serial
import requests

# 연결된 Pico의 포트 확인
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

print("Lets go Chuu")

while True:
	line = ser.readline().decode().strip()
	print("받은 데이터:", line)
	if len(line) == 5 and all(c in "01" for c in line):
		try:
			requests.post("http://localhost:5002/api/alert", json={"binary": line})
			print("보낸 데이터:", line)
		except Exception as e:
			print("전송 실패:", e)



