"""
- [코드 기능]: Realsense 카메라를 통해 타이머(2, 6, 7, 8번)와 릴레이(3, 4, 5, 6, 7, 8번) 단자를 
              개별적으로 정밀 탐지하고, 각 단자의 고유 라벨과 정중앙 좌표를 송신함.
- [입력(Input)]: Realsense 카메라 실시간 영상 스트림 (OpenCV VideoCapture)
- [출력(Output)]: 1. 개별 단자 라벨(예: timer 2, relay 3)이 표시된 통합 탐지 윈도우
                2. /eye/terminal_centers 토픽 (각 단자의 고유 이름과 [y, x] 정규화 좌표 데이터)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
from ultralytics import YOLO

class IntegratedEyeNode(Node):
    def __init__(self):
        """
        [함수 설명]: 노드 초기화 및 타이머/릴레이 전용 YOLO 모델 로드.
        [입력]: 없음
        [출력]: IntegratedEyeNode 인스턴스 초기화
        """
        super().__init__('eye_node')
        
        # 학습된 모델 경로 설정
        timer_model_path = '/home/rokey/cobot_ws/src/cobot2_ws/yolo_timer_v22/yolo_timer/train_20260211_184000/weights/best.pt'
        relay_model_path = '/home/rokey/cobot_ws/src/cobot2_ws/relay_yolo/relay_yolo_e100/train_20260211_203857/weights/best.pt'
        
        # 모델 로드 (학습 시 정의된 라벨 정보를 포함함)
        self.timer_model = YOLO(timer_model_path)
        self.relay_model = YOLO(relay_model_path)
        
        # 좌표 데이터 전송용 퍼블리셔
        self.coord_pub = self.create_publisher(String, '/eye/terminal_centers', 10)
        self.bridge = CvBridge()
        
        # RealSense 카메라 연결
        self.cap = cv2.VideoCapture(6) 
        
        # 10Hz 주기로 추론 프로세스 가동
        self.timer = self.create_timer(0.1, self.inference_callback) 

    def inference_callback(self):
        """
        [함수 설명]: 프레임을 획득하여 두 모델에 전달하고, 추출된 개별 단자 정보를 통합함.
        [입력]: 타이머 이벤트
        [출력]: 고유 라벨이 포함된 JSON 데이터 발행
        """
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w, _ = frame.shape
        combined_detections = []

        # --- Model 1: 타이머 모델 추론 (YOLO 모델 자체를 함수에 전달하여 라벨 획득) ---
        timer_results = self.timer_model(frame, verbose=False)
        self.process_results(self.timer_model, timer_results, frame, (0, 255, 0), combined_detections, w, h)

        # --- Model 2: 릴레이 모델 추론 ---
        relay_results = self.relay_model(frame, verbose=False)
        self.process_results(self.relay_model, relay_results, frame, (255, 0, 0), combined_detections, w, h)

        # 통합된 10개(최대)의 단자 정보 발행
        msg = String()
        msg.data = json.dumps(combined_detections)
        self.coord_pub.publish(msg)

        # 통합 결과 시각화
        cv2.imshow("Integrated Terminal Detection (RealSense)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def process_results(self, model, results, frame, color, detection_list, img_w, img_h):
        """
        [함수 설명]: 모델의 실제 라벨 명칭(names)을 사용하여 바운딩 박스와 좌표를 처리함.
        [입력]: 사용 중인 YOLO 모델 객체, 추론 결과, 프레임, 색상, 데이터 저장 리스트, 영상 너비/높이
        [출력]: 고유 라벨(예: 'timer 2')이 포함된 객체 정보 리스트 업데이트
        """
        for result in results:
            for box in result.boxes:
                # YOLO 모델 학습 시 지정한 실제 클래스 이름 가져오기 (핵심 수정 포인트)
                class_id = int(box.cls[0])
                actual_label = model.names[class_id] # 예: "timer 2", "relay 3" 등
                
                # 바운딩 박스 및 중앙 좌표 계산
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Gemini 공간 추론용 $0 \sim 1000$ 정규화 좌표 변환
                norm_y = int((center_y / img_h) * 1000)
                norm_x = int((center_x / img_w) * 1000)

                # 고유 라벨 정보를 포함하여 데이터 저장
                detection_list.append({
                    "label": actual_label,
                    "point": [norm_y, norm_x]
                })

                # 시각화: 실제 라벨 이름 표시
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                cv2.putText(frame, actual_label, (int(x1), int(y1)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def main(args=None):
    """
    [함수 설명]: ROS2 노드 실행 및 자원 정리.
    """
    rclpy.init(args=args)
    node = IntegratedEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()