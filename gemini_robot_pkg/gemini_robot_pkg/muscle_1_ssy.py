"""
[코드 기능]
두산 협동 로봇(M0609)과 OnRobot RG2 그리퍼를 제어하여 특정 위치에서 스크루 드라이버 툴을 픽업하고, 
작업 수행 후 다시 제자리에 반납하는 공정 자동화 시퀀스를 수행합니다.

[입력(Input)]
- 로봇 설정: ROBOT_ID, ROBOT_MODEL, TOOL, TCP 정보
- 그리퍼 설정: 그리퍼 모델명, Tool Changer의 IP 및 Port 주소
- 작업 좌표: P_SCAN(홈), P_TOOL_PICK_UP(대기), P_TOOL_PICK(파지) 위치 데이터 (posx)

[출력(Output)]
- 로봇 동작: 실제 로봇 팔의 물리적 이동 및 그리퍼 개폐 제어
- 콘솔 로그: 초기화 상태, 작업 진행 상황 및 오류 메시지 출력
- ROS2 메시지: 작업 종료 및 로그 정보 퍼블리시
"""

import rclpy
from rclpy.node import Node  # ROS2 노드 관리를 위한 클래스
import DR_init              # 두산 로봇 초기화 라이브러리
import time
import json
import datetime as dt
import numpy as np
from onrobot import RG      # OnRobot 그리퍼 제어 라이브러리

# =========================
# 전역 변수 및 상태 초기화
# =========================
stop_requested = False      # 안전 정지 요청 상태 변수
rs_pipeline = None          # RealSense 카메라 파이프라인 객체 (필요 시 사용)
rs_intrinsics = None        # 카메라 내부 파라미터 정보

# =========================
# 로봇 및 그리퍼 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight3"
ROBOT_TCP = "GripperDA_v1_3"

ON, OFF = 1, 0              # 디지털 입출력 제어용 상수

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# 지정된 IP와 포트로 그리퍼 객체 생성 (통신 연결)
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# 로봇 이동 시 기본 속도와 가속도 (단위: mm/s, mm/s^2)
VELOCITY = 80
ACC = 80

# =========================
# 작업 로직 함수
# =========================
def perform_screw_task(node):
    """
    [기능] 스크루 드라이버 픽업 및 반납 시퀀스 실행
    [입력] node: rclpy.node.Node 객체 (ROS2 통신용)
    [출력] None (로봇의 물리적 동작 수행)
    """
    # 동작 제어에 필요한 DSR 라이브러리 함수 로컬 임포트
    from DSR_ROBOT2 import (
        posx, movel, wait, DR_MV_MOD_REL
    )
    
    global rs_pipeline, rs_intrinsics, stop_requested

    # 좌표 데이터 정의 (X, Y, Z, A, B, C)
    P_SCAN = posx(423.92, -147.13, 402.39, 168.05, -179.78, 167.78)           # 홈/스캔 위치
    P_TOOL_PICK_UP = posx(424.62, 224.29, 392.75, 22.57, -179.38, 21.95)     # 드라이버 픽업 상단 대기 위치
    P_TOOL_PICK = posx(424.62, 224.29, 192.75, 22.57, -179.38, 21.95)        # 실제 드라이버 파지 위치

    # 1. 초기 위치 이동 및 그리퍼 개방 준비
    movel(P_SCAN, vel=VELOCITY, acc=ACC)                                     # 홈 위치로 직선 이동
    gripper.set_target_width(70)                                             # 그리퍼 목표 너비 70mm 설정
    gripper.open_gripper()                                                   # 그리퍼 개방 명령
    wait(1)                                                                  # 동작 완료 대기
    
    # 2. 드라이버 픽업 위치 접근
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)                             # 픽업 상단으로 이동
    movel(P_TOOL_PICK, vel=VELOCITY, acc=ACC)                                # 드라이버 파지 지점으로 하강
    
    # 3. 드라이버 파지
    gripper.set_target_width(26)                                             # 드라이버 핸들 두께에 맞춰 16mm 설정
    gripper.close_gripper()                                                  # 그리퍼 닫기
    wait(1)                                                                  # 확실한 파지를 위한 대기

    # 4. 드라이버 인출
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)                             # 드라이버를 수직으로 뽑아 올림

    # [작업 영역: 필요 시 나사 체결 좌표로 이동 로직 삽입]

    # 5. 드라이버 반납 (역순 동작)
    movel(P_TOOL_PICK_UP, vel=VELOCITY, acc=ACC)                             # 반납 위치 상단 대기
    movel(P_TOOL_PICK, vel=VELOCITY, acc=ACC)                                # 반납 거치대로 하강
    
    # 6. 드라이버 해제
    gripper.set_target_width(70)                                             # 그리퍼 개방 너비 설정
    gripper.open_gripper()                                                   # 드라이버 놓기
    wait(1)

    # 7. 홈 위치 복귀
    movel(P_SCAN, vel=VELOCITY, acc=ACC)                                     # 최종 홈 위치로 이동하여 작업 종료

def main(args=None):
    """
    [기능] ROS2 노드 초기화 및 전체 프로세스 실행 관리
    [입력] args: 커맨드라인 인자 (기본값 None)
    [출력] None
    """
    # ROS2 초기화
    rclpy.init(args=args)
    node = Node('screw_task_node')

    try:
        # 로봇 하드웨어 인터페이스 연결 초기화
        DR_init.dr_init(ROBOT_ID, ROBOT_MODEL)
        
        # 로봇 좌표계 및 툴 설정 (DSR_ROBOT2 내부 전역 설정 업데이트)
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(ROBOT_TOOL)                                                 # 정의된 툴 무게/중심 설정 적용
        set_tcp(ROBOT_TCP)                                                   # 정의된 TCP 좌표계 적용
        
        print(f"로봇 초기화 완료: {ROBOT_MODEL}")
        
        # 메인 시퀀스 함수 호출
        perform_screw_task(node)
        
        print("모든 작업 시퀀스가 정상 완료되었습니다.")

    except KeyboardInterrupt:
        print("사용자에 의한 강제 종료(Ctrl+C)가 감지되었습니다.")
    except Exception as e:
        print(f"시스템 오류 발생: {str(e)}")
    finally:
        # 자원 해제 및 종료
        node.destroy_node()                                                  # ROS2 노드 파괴
        rclpy.shutdown()                                                     # ROS2 시스템 종료

if __name__ == '__main__':
    main()