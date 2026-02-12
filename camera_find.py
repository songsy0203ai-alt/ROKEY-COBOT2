import cv2
import sys

def start_camera(index):
    # 1. 카메라 객체 생성
    cap = cv2.VideoCapture(index)

    # 2. 카메라 연결 확인
    if not cap.isOpened():
        print(f">>> [ERROR] 카메라 인덱스 {index}를 찾을 수 없거나 열 수 없습니다.")
        return

    print(f">>> [SUCCESS] 카메라 인덱스 {index} 실행 중... (종료하려면 'q'를 누르세요)")

    # 3. 실시간 화면 출력 루프
    while True:
        ret, frame = cap.read()
        if not ret:
            print(">>> [ERROR] 프레임을 읽어올 수 없습니다.")
            break

        # 화면 사이즈 고정 (서영님 UI 기준 640x480)
        frame = cv2.resize(frame, (640, 480))

        # 윈도우 표시
        cv2.imshow(f"Camera Index: {index}", frame)

        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4. 자원 해제
    cap.release()
    cv2.destroyAllWindows()
    print(">>> 카메라가 안전하게 종료되었습니다.")

if __name__ == "__main__":
    # 인자값이 없는 경우 예외 처리
    if len(sys.argv) < 2:
        print(">>> [USAGE] python3 camera_find.py [INDEX]")
        print(">>> [EXAMPLE] python3 camera_find.py 0")
        sys.exit(1)

    try:
        camera_index = int(sys.argv[1])
        start_camera(camera_index)
    except ValueError:
        print(">>> [ERROR] 인덱스는 정수(숫자)여야 합니다.")