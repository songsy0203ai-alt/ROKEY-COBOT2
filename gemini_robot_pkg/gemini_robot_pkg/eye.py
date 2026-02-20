# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/eye.py

"""
- [코드 기능]: Realsense 카메라를 통해 회로도 완성 작업에 필요한 5가지 부품인 Timer, Relay, Lamp, Switch, Power를 동시에 객체탐지함.
              5개의 YOLO 모델을 병렬로 운용하여 각 부품의 고유 라벨과 정규화 좌표를 송신함.
- [입력(Input)]: Realsense 카메라 실시간 영상 스트림 (OpenCV VideoCapture)
- [출력(Output)]: 1. 5종 부품(Timer, Relay, Lamp, Switch, Power)이 색상별로 표시된 통합 탐지 윈도우
                2. /eye/terminal_centers 토픽 (고유 라벨명과 [y, x] 정규화 좌표 JSON 데이터)
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
        [함수 설명]: 노드 초기화 및 5종 부품(Timer, Relay, Lamp, Switch, Power) 전용 YOLO 모델 로드.
        """
        super().__init__('eye_node')
        
        # 1. 학습된 모델 경로 설정
        timer_model_path = '/home/ssy/cobot_ws/src/cobot2_ws/yolo_timer_v22/yolo_timer/train_20260211_184000/weights/best.pt'
        relay_model_path = '/home/ssy/cobot_ws/src/cobot2_ws/relay_yolo/relay_yolo_e100/train_20260211_203857/weights/best.pt'
        lamp_model_path = '/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/trained_models/best_lamp.pt'
        switch_model_path = '/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/trained_models/best_switch.pt'
        power_model_path = '/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/trained_models/best_power.pt'
        
        # 2. 모델 로드 (학습된 클래스 이름 정보 포함)
        self.timer_model = YOLO(timer_model_path)
        self.relay_model = YOLO(relay_model_path)
        self.lamp_model = YOLO(lamp_model_path)
        self.switch_model = YOLO(switch_model_path)
        self.power_model = YOLO(power_model_path)
        
        # 3. ROS2 통신 및 카메라 설정
        self.coord_pub = self.create_publisher(String, '/eye/terminal_centers', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(6) # Realsense 카메라 인덱스 유지
        
        # 10Hz 주기로 추론 프로세스 가동
        self.timer = self.create_timer(0.1, self.inference_callback) 
        self.get_logger().info("Integrated Eye Node 가동: 5종 모델 동시 탐지 중...")

    def inference_callback(self):
        """
        [함수 설명]: 프레임을 획득하여 5개의 모델에 순차적으로 전달하고 결과를 통합함.
        """
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w, _ = frame.shape
        combined_detections = []

        # --- Model 1: 타이머 탐지 (Green) ---
        timer_results = self.timer_model(frame, verbose=False, conf=0.15)
        self.process_results(self.timer_model, timer_results, frame, (0, 255, 0), combined_detections, w, h)

        # --- Model 2: 릴레이 탐지 (Blue) ---
        relay_results = self.relay_model(frame, verbose=False, conf=0.15)
        self.process_results(self.relay_model, relay_results, frame, (255, 0, 0), combined_detections, w, h)

        # --- Model 3: 램프 탐지 (Yellow) ---
        lamp_results = self.lamp_model(frame, verbose=False, conf=0.15)
        self.process_results(self.lamp_model, lamp_results, frame, (0, 255, 255), combined_detections, w, h)

        # --- Model 4: 스위치 탐지 (Cyan) ---
        switch_results = self.switch_model(frame, verbose=False, conf=0.15)
        self.process_results(self.switch_model, switch_results, frame, (255, 255, 0), combined_detections, w, h)

        # --- Model 5: 전원 탐지 (Cyan) ---
        power_results = self.power_model(frame, verbose=False, conf=0.15)
        self.process_results(self.power_model, power_results, frame, (255, 255, 0), combined_detections, w, h)

        # 통합된 객체 정보 발행
        msg = String()
        msg.data = json.dumps(combined_detections)
        self.coord_pub.publish(msg)

        # 통합 시각화 결과 출력
        cv2.imshow("Multi-Model Detection (Timer/Relay/Lamp/Switch/Power)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def process_results(self, model, results, frame, color, detection_list, img_w, img_h):
        """
        [함수 설명]: 각 모델의 결과를 처리하여 정규화 좌표를 생성하고 시각화함.
        """
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                actual_label = model.names[class_id]
                
                # 중앙 좌표 계산
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Gemini 공간 추론용 0 ~ 1000 정규화
                norm_y = int((center_y / img_h) * 1000)
                norm_x = int((center_x / img_w) * 1000)

                detection_list.append({
                    "label": actual_label,
                    "point": [norm_y, norm_x]
                })

                # 시각화 로직
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(frame, actual_label, (int(x1), int(y1)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def main(args=None):
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