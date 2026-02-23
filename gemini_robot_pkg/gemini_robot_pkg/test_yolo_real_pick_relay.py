import cv2
import rclpy
import numpy as np
import time
from ultralytics import YOLO
from rclpy.node import Node
from gemini_robot_pkg.realsense import ImgNode
from scipy.spatial.transform import Rotation
from gemini_robot_pkg.onrobot import RG
import DR_init

# 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

class YoloRobotNode(Node):
    def __init__(self):
        super().__init__("yolo_robot_node")
        
        # 1. 모델 로드
        self.model = YOLO('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/trained_models/best_relay.pt') 
        
        # 2. Realsense 노드 연결
        self.img_node = ImgNode()
        print("카메라 파라미터를 기다리는 중...")
        while rclpy.ok():
            rclpy.spin_once(self.img_node, timeout_sec=0.1)
            self.intrinsics = self.img_node.get_camera_intrinsic()
            if self.intrinsics is not None:
                break
        print("카메라 파라미터 수신 완료.")

        # 3. 캘리브레이션 파일 로드 (경로 확인 필수)
        calib_path = "/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/calib_npy/T_gripper2camera_Ours.npy"
        self.gripper2cam = np.load(calib_path)
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    def accumulate_detections(self, duration=10):
        print(f"\n>>> {duration}초 동안 검출 및 강력한 좌표 변환을 시작합니다...")
        
        class_names = self.model.names
        best_detections = {}
        start_time = time.time()

        while time.time() - start_time < duration:
            rclpy.spin_once(self.img_node, timeout_sec=0.01)
            color_frame = self.img_node.get_color_frame()
            depth_frame = self.img_node.get_depth_frame()
            
            if color_frame is None or depth_frame is None:
                continue

            results = self.model.predict(source=color_frame, imgsz=640, conf=0.1, verbose=False)
            result = results[0]

            # OBB 결과가 있는지 확인
            if result.obb is not None and len(result.obb) > 0:
                for i in range(len(result.obb)):
                    cls_id = int(result.obb.cls[i].item())
                    conf = float(result.obb.conf[i].item())
                    label_name = class_names[cls_id]
                    
                    coords = result.obb.xywhr[i].cpu().numpy()
                    cx, cy, w, h, r_val = coords[0], coords[1], coords[2], coords[3], coords[4]

                    # --- [강력한 Depth 샘플링] ---
                    # 중앙점 기준 BBox 크기의 20% 영역을 샘플링하여 0이 아닌 값들의 중앙값 취득
                    x1, y1 = int(cx - w*0.1), int(cy - h*0.1)
                    x2, y2 = int(cx + w*0.1), int(cy + h*0.1)
                    
                    # 이미지 범위 안으로 제한
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(depth_frame.shape[1]-1, x2), min(depth_frame.shape[0]-1, y2)
                    
                    depth_roi = depth_frame[y1:y2, x1:x2]
                    valid_depths = depth_roi[depth_roi > 0]

                    if len(valid_depths) > 0:
                        depth_z = np.median(valid_depths)
                        
                        # 카메라 좌표계 -> 로봇 베이스 좌표계 변환
                        camera_pos = self.get_camera_pos(cx, cy, depth_z, self.intrinsics)
                        robot_pos = self.transform_to_base(camera_pos)

                        if label_name not in best_detections or conf > best_detections[label_name][0]:
                            best_detections[label_name] = [conf, robot_pos[0], robot_pos[1], robot_pos[2], r_val]
                            print(f"[SUCCESS] {label_name} 변환 완료 (Z: {depth_z:.1f}mm)")
                    else:
                        # Depth를 전혀 못 읽는 경우 (반사가 너무 심함)
                        if time.time() % 3 < 0.05:
                            print(f"[WARNING] {label_name} 검출됨, 하지만 주변 Depth 값이 모두 0입니다.")

            # 시각화
            cv2.imshow("YOLO-OBB Detection", result.plot())
            if cv2.waitKey(1) & 0xFF == 27: break

        print("\n" + "="*70)
        print("       [최종 로봇 좌표 결과 (mm)]")
        print("="*70)
        if not best_detections:
            print("(!) 단자가 하나도 변환되지 않았습니다. Depth 측정 환경을 확인하세요.")
        else:
            for name in sorted(best_detections.keys()):
                d = best_detections[name]
                print(f"Label: {name:10} | Conf: {d[0]:.2f} | X: {d[1]:7.2f}, Y: {d[2]:7.2f}, Z: {d[3]:7.2f}")
        print("="*70 + "\n")
        
        return best_detections

    def get_camera_pos(self, cx, cy, z, intrinsics):
        fx, fy = intrinsics['fx'], intrinsics['fy']
        ppx, ppy = intrinsics['ppx'], intrinsics['ppy']
        x = (cx - ppx) * z / fx
        y = (cy - ppy) * z / fy
        return (x, y, z)

    def transform_to_base(self, camera_coords):
        from DSR_ROBOT2 import get_current_posx 
        coord = np.append(np.array(camera_coords), 1)
        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        # Doosan Robot ZYZ 컨벤션 적용
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloRobotNode()
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = yolo_node

    try:
        from DSR_ROBOT2 import wait
        yolo_node.accumulate_detections(duration=10)
    finally:
        cv2.destroyAllWindows()
        yolo_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()