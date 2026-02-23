# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/eye_test.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
import numpy as np
import os
import sys
import time
from ultralytics import YOLO
from scipy.spatial.transform import Rotation

# [수정]: ROS 2 패키지 구조에 맞는 임포트 방식
try:
    # 패키지명을 포함하여 임포트 시도
    from gemini_robot_pkg.realsense import ImgNode
except ImportError:
    try:
        # 로컬 임포트 시도 (스크립트 직접 실행 시)
        from realsense import ImgNode
    except ImportError:
        print("\n[에러] realsense.py를 찾을 수 없습니다.")
        print("현재 경로:", os.getcwd())
        print("sys.path:", sys.path)
        sys.exit(1) # 임포트 실패 시 프로그램 종료 (NameError 방지)

class IntegratedEyeNode(Node):
    def __init__(self):
        super().__init__('eye_node')
        
        # 1. 모델 및 캘리브레이션 설정
        self.load_models()
        self.calib_path = "/home/ssy/Tutorial/Calibration_Tutorial/T_gripper2camera_Ours.npy"
        self.gripper2cam = np.load(self.calib_path)
        
        # 2. RealSense 데이터 노드 초기화 (test.py 방식)
        self.img_node = ImgNode()
        rclpy.spin_once(self.img_node)
        time.sleep(1.0)
        self.intrinsics = self.img_node.get_camera_intrinsic()
        
        # 3. 상태 변수
        self.scan_start_time = self.get_clock().now()
        self.scan_duration = 10.0
        self.best_detections = {}
        self.scan_completed = False
        
        # 4. ROS2 통신
        self.coord_pub = self.create_publisher(String, '/eye/terminal_centers', 10)
        self.timer = self.create_timer(0.1, self.inference_callback)
        
        self.get_logger().info("Eye Node 가동: 10초 스캔 및 정밀 좌표 변환 모드")

    def load_models(self):
        # 모델 경로 설정 (서영님의 기존 경로 유지)
        base_path = '/home/ssy/cobot_ws/src/cobot2_ws'
        self.models = {
            'timer': YOLO(f'{base_path}/yolo_timer_v22/yolo_timer/train_20260211_184000/weights/best.pt'),
            'relay': YOLO(f'{base_path}/relay_yolo/relay_yolo_e100/train_20260211_203857/weights/best.pt'),
            'lamp': YOLO(f'{base_path}/gemini_robot_pkg/trained_models/best_lamp.pt'),
            'switch': YOLO(f'{base_path}/gemini_robot_pkg/trained_models/best_switch.pt'),
            'power': YOLO(f'{base_path}/gemini_robot_pkg/trained_models/best_power.pt')
        }

    def get_camera_pos(self, u, v, z):
        """픽셀(u, v)과 깊이(z)를 카메라 좌표계(x, y, z)로 변환"""
        if z is None or z == 0: return None
        cam_x = (u - self.intrinsics["ppx"]) * z / self.intrinsics["fx"]
        cam_y = (v - self.intrinsics["ppy"]) * z / self.intrinsics["fy"]
        return np.array([cam_x, cam_y, z, 1.0])

    def get_robot_pose_matrix(self):
        """현재 로봇의 위치(P_SCAN 등)를 행렬로 변환 (현재는 고정 위치 가정)"""
        # 실제 환경에서는 DSR_ROBOT2의 get_current_posx()를 써야 하지만, 
        # 스캔 시 로봇이 P_SCAN에 고정되어 있다면 그 값을 상수로 넣는 것이 안전합니다.
        # 예: P_SCAN = [423.9, -147.1, 402.4, 168.1, -179.8, 167.8]
        x, y, z, rx, ry, rz = [423.9, -147.1, 402.4, 168.1, -179.8, 167.8]
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def inference_callback(self):
        # 1. RealSense 프레임 획득
        rclpy.spin_once(self.img_node, timeout_sec=0.01)
        frame = self.img_node.get_color_frame()
        depth_frame = self.img_node.get_depth_frame()
        if frame is None or depth_frame is None: return

        h, w, _ = frame.shape
        elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
        combined_detections = []

        # 2. 모델별 추론 및 결과 처리
        colors = {'timer': (0,255,0), 'relay': (255,0,0), 'lamp': (0,255,255), 'switch': (255,255,0), 'power': (255,0,255)}
        
        for name, model in self.models.items():
            results = model(frame, verbose=False, conf=0.15)
            self.process_results(name, model, results, frame, depth_frame, colors[name], combined_detections, w, h, elapsed)

        # 3. 10초 후 결과 출력
        if elapsed > self.scan_duration and not self.scan_completed:
            self.scan_completed = True
            self.print_final_results()

        # 4. 발행 및 시각화
        self.coord_pub.publish(String(data=json.dumps(combined_detections)))
        cv2.imshow("Detection (Press Q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): rclpy.shutdown()

    def process_results(self, type_name, model, results, frame, depth_frame, color, detection_list, img_w, img_h, elapsed):
        base2gripper = self.get_robot_pose_matrix()
        base2cam = base2gripper @ self.gripper2cam

        for result in results:
            for box in result.boxes:
                label = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # 깊이 값 및 좌표 변환 (test.py 로직)
                z = depth_frame[v, u] if 0 <= v < img_h and 0 <= u < img_w else 0
                cam_pos = self.get_camera_pos(u, v, z)
                
                robot_pos = [0.0, 0.0, 0.0]
                if cam_pos is not None:
                    robot_pos = np.dot(base2cam, cam_pos)[:3]

                # 최고 신뢰도 갱신
                if elapsed <= self.scan_duration:
                    if label not in self.best_detections or conf > self.best_detections[label]['conf']:
                        self.best_detections[label] = {'conf': conf, 'robot': robot_pos, 'pixel': (u, v)}

                detection_list.append({"label": label, "point": [int((v/img_h)*1000), int((u/img_w)*1000)]})
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

    def print_final_results(self):
        self.get_logger().info("="*50)
        self.get_logger().info("10초 스캔 완료. 최고 신뢰도 로봇 좌표 결과:")
        for label, d in self.best_detections.items():
            r = d['robot']
            self.get_logger().info(f"[{label}] Conf: {d['conf']:.2f} | Robot X: {r[0]:.1f}, Y: {r[1]:.1f}, Z: {r[2]:.1f}")
        self.get_logger().info("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedEyeNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()