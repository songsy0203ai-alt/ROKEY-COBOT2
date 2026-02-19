<<<<<<< HEAD
# gemini_robot_pkg/gemini_robot_pkg/nerve.py
=======
# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/nerve.py
>>>>>>> 66a82552cbdf12daf76af50fda1379f9c49ee5eb

"""
- [코드 기능]: 
    1. GeminiNerve: 호모그래피 행렬 연산 및 파일 I/O 담당 (Utility Class)
    2. NerveNode: ROS2 토픽 수신 및 좌표 변환 실행 노드 (ROS2 Node)
- [입력(Input)]: /brain/normalized_coords (Gemini의 정규화 좌표 JSON)
- [출력(Output)]: /nerve/robot_coords (최종 변환된 로봇의 [X, Y] mm 좌표)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json
import os

class GeminiNerve:
    """호모그래피 행렬의 저장, 로드 및 좌표 변환 수학 모델을 담당하는 클래스"""
    def __init__(self, calibration_path):
        self.calibration_path = calibration_path
        self.homography_matrix = None
        self.load_calibration()

    def load_calibration(self):
        """저장된 JSON 파일에서 3x3 호모그래피 행렬을 로드함"""
        if os.path.exists(self.calibration_path):
            try:
                with open(self.calibration_path, 'r') as f:
                    data = json.load(f)
                    self.homography_matrix = np.array(data['matrix'])
                return True
            except Exception:
                return False
        return False

    def save_calibration(self, matrix):
        """측정된 호모그래피 행렬을 JSON 파일로 저장함"""
        self.homography_matrix = matrix
        data = {'matrix': matrix.tolist()}
        os.makedirs(os.path.dirname(self.calibration_path), exist_ok=True)
        with open(self.calibration_path, 'w') as f:
            json.dump(data, f)

    def denormalize_pixel(self, y_norm, x_norm, img_w, img_h):
        """0~1000 정규화 좌표를 실제 이미지 픽셀 좌표로 복원"""
        real_x = (x_norm / 1000.0) * img_w
        real_y = (y_norm / 1000.0) * img_h
        return real_x, real_y

    def convert_to_robot_coords(self, y_norm, x_norm, img_w=1280, img_h=720):
        """
        Gemini 좌표(픽셀 평면)를 로봇 좌표(mm 단위 평면)로 변환
        수학적 모델: $$ \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix} = H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} $$
        """
        if self.homography_matrix is None:
            return None
            
        px, py = self.denormalize_pixel(y_norm, x_norm, img_w, img_h)
        pixel_point = np.array([[px], [py], [1.0]])
        robot_point = np.dot(self.homography_matrix, pixel_point)
        
        # 동차 좌표계(Homogeneous coordinates) 정규화
        w = robot_point[2][0]
        robot_x = robot_point[0][0] / w
        robot_y = robot_point[1][0] / w
        return [robot_x, robot_y]

class NerveNode(Node):
    """GeminiNerve를 사용하여 실시간 좌표 변환을 수행하는 ROS2 노드"""
    def __init__(self):
        super().__init__('nerve_node')
        
        # 캘리브레이션 데이터 경로 설정
        calib_path = os.path.expanduser(
            '~/cobot_ws/src/cobot2_ws/gemini_robot_pkg/data/calibration/matrix.json'
        )
        self.nerve_util = GeminiNerve(calib_path)

        if self.nerve_util.homography_matrix is None:
            self.get_logger().warn("캘리브레이션 파일이 없습니다. calib_node를 먼저 실행하세요.")

        # ROS2 구독 및 발행 설정
        self.subscription = self.create_subscription(
            String, '/brain/normalized_coords', self.coord_callback, 10
        )
        self.robot_pub = self.create_publisher(String, '/nerve/robot_coords', 10)
        
        self.get_logger().info("Nerve 노드가 준비되었습니다.")

    def coord_callback(self, msg):
        if self.nerve_util.homography_matrix is None:
            return

        try:
            tasks = json.loads(msg.data)
            if not tasks: return
            
            y_norm, x_norm = tasks[0]['point']
            label = tasks[0]['label']

            # [핵심 수정]: 1280x720 -> 640x480 (또는 실제 카메라 출력 해상도)
            # 팁: eye.py에서 frame.shape를 출력해서 나오는 값을 넣으세요!
            robot_xy = self.nerve_util.convert_to_robot_coords(y_norm, x_norm, img_w=640, img_h=480)
            
            if robot_xy:
                result = {"label": label, "robot_x": robot_xy[0], "robot_y": robot_xy[1]}
                self.robot_pub.publish(String(data=json.dumps(result)))
                self.get_logger().info(f"변환 완료: {label} -> X:{robot_xy[0]:.1f}, Y:{robot_xy[1]:.1f}")
        except Exception as e:
            self.get_logger().error(f"좌표 변환 중 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NerveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
