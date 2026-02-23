# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/muscle_1_ssy.py

import rclpy
from rclpy.node import Node
import DR_init
import time
from .onrobot import RG  # 그리퍼 라이브러리

# =========================
# 1. 로봇 및 그리퍼 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY, ACC = 120, 100 # 80,80이 기본값

# 그리퍼 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# =========================
# 2. 로봇 초기화 함수
# =========================
def initialize_robot():
    """로봇 ID, 모델, 툴, TCP 및 모드 초기화"""
    # 전역 설정을 먼저 노드에 바인딩
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    # 내부 임포트 (이 시점에 서비스 클라이언트가 정상 생성됨)
    from DSR_ROBOT2 import (
        set_tool, set_tcp, set_robot_mode, 
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, wait
    )

    print(f">>> [{ROBOT_ID}] 하드웨어 초기화 시작...")
    
    # 안전한 설정을 위해 매뉴얼 모드 전환 후 설정
    set_robot_mode(ROBOT_MODE_MANUAL)
    wait(0.5)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    # 동작을 위해 자동 모드로 전환
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    wait(1.0)
    print(f">>> 로봇이 AUTONOMOUS 모드로 준비되었습니다.")

# =========================
# 3. 메인 작업 시퀀스
# =========================
def perform_screw_task():
    # 함수 내부에서 필요한 기능만 임포트 (핵심)
    from DSR_ROBOT2 import posx, movel, wait, DR_MV_MOD_REL

    # 작업 좌표 정의
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
    wait(1.5) # 파지 안정화 시간
    print("2단계 완료")

    # 3. 드라이버 인출 및 반납 (예시 시퀀스)
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)
    print("작업 수행 위치로 이동 가능...")
    wait(2.0)

    # 현재 위치에서 90도씩 4회전
    P_ROT = posx(0,0,0,0,0,30)
    for _ in range(4):
        movel(P_ROT, time=0.5, mod=DR_MV_MOD_REL)

    # 원 위치로 90도씩 4회전
    P_ROT = posx(0,0,0,0,0,-30)
    for _ in range(4):
        movel(P_ROT, time=0.5, mod=DR_MV_MOD_REL)

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
# 4. 메인 실행부
# =========================
def main(args=None):
    rclpy.init(args=args)
    # 네임스페이스를 명시적으로 지정하여 노드 생성
    node = Node('screw_task_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        # 로봇 초기화 (ID 설정 및 모드 변경 포함)
        initialize_robot()
        
        # 실제 작업 수행
        perform_screw_task()

    except KeyboardInterrupt:
        print("사용자에 의해 중단됨")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()