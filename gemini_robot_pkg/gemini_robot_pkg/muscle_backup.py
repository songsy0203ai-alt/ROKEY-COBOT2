# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/muscle.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import DR_init
import time
import json
import threading
from .onrobot import RG  # 그리퍼 라이브러리

# =========================
# 1. 로봇 및 그리퍼 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight3"
ROBOT_TCP = "GripperDA_v1_3"

VELOCITY, ACC = 120, 100 # 80,80이 기본값

# 그리퍼 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# =========================
# 2. ROS2 통신용 커스텀 노드
# =========================
class ScrewTaskNode(Node):
    def __init__(self):
        super().__init__('screw_task_node', namespace=ROBOT_ID)
        self.target_x = None
        self.target_y = None
        self.target_label = None
        
        # nerve.py에서 보내는 로봇 좌표 구독
        self.subscription = self.create_subscription(
            String, '/nerve/robot_coords', self.coord_callback, 10
        )

    def coord_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.target_label = data['label']
            self.target_x = data['robot_x']
            self.target_y = data['robot_y']
            self.get_logger().info(f"좌표 수신 완료: [{self.target_label}] X:{self.target_x:.1f}, Y:{self.target_y:.1f}")
        except Exception as e:
            self.get_logger().error(f"좌표 수신 에러: {e}")

# =========================
# 3. 로봇 초기화 함수
# =========================
def initialize_robot():
    """로봇 ID, 모델, 툴, TCP 및 모드 초기화"""
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import (
        set_tool, set_tcp, set_robot_mode, 
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, wait
    )

    print(f">>> [{ROBOT_ID}] 하드웨어 초기화 시작...")
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    wait(0.5)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    wait(1.0)
    print(f">>> 로봇이 AUTONOMOUS 모드로 준비되었습니다.")

# =========================
# 4. 메인 작업 시퀀스
# =========================
def perform_screw_task(node):
    from DSR_ROBOT2 import posx, movel, wait

    # 작업 좌표 정의 (기존 픽업 관련)
    P_SCAN = posx(423.92, -147.13, 402.39, 168.05, -179.78, 167.78)
    P_TOOL_PICK_UP = posx(424.62, 224.29, 352.75, 22.57, -179.38, 21.95)
    P_TOOL_PICK = posx(424.62, 224.29, 192.75, 22.57, -179.38, 21.95)

    print("시퀀스 시작: 홈 위치 이동")
    movel(P_SCAN, vel=VELOCITY, acc=ACC)
    
    # 1. 그리퍼 개방
    gripper.set_target_width(70)
    gripper.open_gripper()
    wait(1.0)
    print("1단계 완료")

    # 2. 드라이버 픽업
    print("드라이버 픽업 중...")
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)
    movel(P_TOOL_PICK, vel=VELOCITY, acc=ACC)
    
    gripper.set_target_width(26)
    gripper.close_gripper()
    wait(1.5)
    print("2단계 완료")

    # 3. 드라이버 인출 및 작업 준비
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)
    print("작업 수행 위치로 이동 가능...")
    wait(1.0)

    # >>>>>>>place 1<<<<<<<<<
    print("Nerve 노드로부터 작업 좌표를 기다리는 중입니다...")
    
    # ROS2 토픽이 들어올 때까지 대기
    while node.target_x is None and rclpy.ok():
        time.sleep(0.5)
        
    # 좌표 획득 완료!
    target_x = node.target_x
    target_y = node.target_y
    label = node.target_label
    
    print(f"목표물 [{label}] 확인! 좌표(X: {target_x:.1f}, Y: {target_y:.1f})로 접근을 시작합니다.")

    # [주의] Z축 높이와 자세(rx, ry, rz)는 서영님의 실제 단자대 높이에 맞게 반드시 수정해야 합니다!
    # 여기서는 P_TOOL_PICK_UP의 자세를 유지하면서 Z축만 조정하는 예시입니다.
    SAFE_Z = 350.0   # 충돌 방지용 안전 접근 높이
    WORK_Z = 190.0   # 실제 나사가 있는 작업 높이 (서영님이 캘리브레이션한 평면 높이)
    RX, RY, RZ = 22.57, -179.38, 21.95 # 하방 수직을 향하는 드라이버 자세
    
    P_TARGET_ABOVE = posx(target_x, target_y, SAFE_Z, RX, RY, RZ)
    P_TARGET_INSERT = posx(target_x, target_y, WORK_Z, RX, RY, RZ)

    # 안전 높이로 먼저 수평 이동
    movel(P_TARGET_ABOVE, vel=VELOCITY, acc=ACC)
    wait(0.5)
    
    # 수직으로 체결 위치까지 하강
    movel(P_TARGET_INSERT, vel=VELOCITY, acc=ACC)
    print(f"[{label}] 나사 체결/해제 작업 시뮬레이션 중...")
    wait(2.0)
    
    # 작업 완료 후 다시 안전 높이로 상승
    movel(P_TARGET_ABOVE, vel=VELOCITY, acc=ACC)
    print(f"[{label}] 작업 완료!")
    # >>>>>>>place 1 끝<<<<<<<<<

    print("드라이버 반납 중...")
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)
    movel(P_TOOL_PICK, vel=VELOCITY, acc=ACC)
    
    gripper.set_target_width(70)
    gripper.open_gripper()
    wait(1.0)
    print("3단계 완료")

    # 4. 종료
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)
    movel(P_SCAN, vel=VELOCITY, acc=ACC)
    print("모든 작업이 완료되었습니다.")

# =========================
# 5. 메인 실행부
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = ScrewTaskNode()
    DR_init.__dsr__node = node

    # 백그라운드에서 ROS2 콜백(구독)을 지속적으로 처리하기 위한 스레드 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        initialize_robot()
        
        # node 객체를 넘겨주어 내부에서 좌표를 읽을 수 있게 함
        perform_screw_task(node)

    except KeyboardInterrupt:
        print("사용자에 의해 중단됨")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()