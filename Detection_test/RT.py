import cv2

# IP Webcam 비디오 스트림 URL (핸드폰에서 IP Webcam 앱에서 확인 가능)
url = "http://192.168.0.2:8080/video"  # IP Webcam의 스트림 URL

# 비디오 스트림 열기
cap = cv2.VideoCapture(url)

while True:
    # 프레임 읽기
    success, img = cap.read()
    
    # 프레임이 성공적으로 읽혔는지 확인
    if not success:
        print("비디오 스트림을 불러올 수 없습니다.")
        break

    # 프레임을 화면에 표시
    cv2.imshow('IP Webcam Stream', img)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 비디오 스트림 해제 및 창 닫기
cap.release()
cv2.destroyAllWindows()
