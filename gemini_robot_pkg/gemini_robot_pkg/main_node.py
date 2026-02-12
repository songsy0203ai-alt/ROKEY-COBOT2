#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[코드 기능]
카메라를 통해 작업 환경을 촬영하고, Gemini AI가 회로도와 현장 사진을 분석하여 
추출한 대상의 위치로 Doosan 로봇을 이동시키는 전체 제어 시퀀스(Orchestration) 노드입니다.

[입력(Input)]
1. 웹캠 영상: 작업 공간의 실시간 사진.
2. 회로도 이미지: 분석의 기준이 되는 레퍼런스 이미지 파일.
3. Gemini API 응답: 분석된 객체의 픽셀 좌표 및 레이블 정보.

[출력(Output)]
1. MoveLine Service Call: 계산된 로봇 좌표(X, Y, Z)로의 실제 로봇 이동 명령.
2. 로그 출력: 작업 분석 결과 및 이동 좌표 정보 터미널 출력.
"""

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine # Doosan 로봇 이동 서비스 (라인 이동)
from gemini_robot_pkg.gemini_robot_pkg.brain_backup import GeminiBrain
from gemini_robot_pkg.nerve import GeminiNerve
import cv2
import os

class MainOrchestrator(Node):
    def __init__(self):
        """
        MainOrchestrator 클래스 초기화 및 서비스 클라이언트 설정
        """
        super().__init__('main_node')
        
        # 1. 초기 설정 및 모듈 로드
        # AI 판단을 위한 GeminiBrain 객체 생성
        api_key = "YOUR_GEMINI_API_KEY" 
        self.brain = GeminiBrain(api_key)
        
        # 픽셀 좌표를 로봇 좌표로 변환하기 위한 캘리브레이션 행렬 로드
        calib_path = os.path.expanduser('~/cobot_ws/src/cobot2_ws/gemini_robot_pkg/data/calibration/matrix.json')
        self.nerve = GeminiNerve(calib_path)
        
        # 2. Doosan 로봇 서비스 클라이언트 설정
        # dsr_msgs2/srv/MoveLine 서비스를 사용하여 로봇의 직선 이동 제어
        self.move_line_cli = self.create_client(MoveLine, '/dsr01/motion/move_line')
        
        # 서비스 서버가 활성화될 때까지 대기
        while not self.move_line_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 서비스를 기다리는 중...')

    def run_collaboration_sequence(self):
        """
        전체 협업 시퀀스 실행 (인지 -> 분석 -> 변환 -> 동작)
        """
        # --- 1. 사진 촬영 (Eye) ---
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if not ret: 
            self.get_logger().error('카메라 영상을 획득할 수 없습니다.')
            return
        
        # 분석을 위해 임시 경로에 이미지 저장
        img_path = "/tmp/current_workspace.jpg"
        cv2.imwrite(img_path, frame)
        cap.release()

        # --- 2. Gemini에게 판단 요청 (Brain) ---
        self.get_logger().info('Gemini가 회로도를 분석하고 다음 작업을 추론 중입니다...')
        circuit_path = os.path.expanduser('~/cobot_ws/src/cobot2_ws/gemini_robot_pkg/data/circuits/circuit_1.png')
        
        # 브레인 모듈을 통해 이미지 분석 후 JSON 형태의 텍스트 응답 획득
        response = self.brain.get_next_task_point(circuit_path, img_path)
        
        # --- 3. 좌표 파싱 및 변환 (Nerve) ---
        # 텍스트 응답에서 좌표 리스트 [ {point: [y, x], label: 'name'}, ... ] 추출
        tasks = self.nerve.parse_gemini_response(response)
        
        if not tasks:
            self.get_logger().error('작업 좌표를 추출하지 못했습니다.')
            return

        # 분석된 각 작업 포인트에 대해 반복 실행
        for task in tasks:
            y, x = task['point']
            label = task['label']
            self.get_logger().info(f'대상 감지: {label} (Pixel: y={y}, x={x})')

            # 캘리브레이션 행렬을 사용하여 픽셀(u,v)을 로봇 좌표(mm)로 변환
            robot_xy = self.nerve.convert_to_robot_coords(y, x)
            
            if robot_xy:
                self.get_logger().info(f'로봇 이동 목표: X={robot_xy[0]:.2f}, Y={robot_xy[1]:.2f}')
                # --- 4. 로봇 이동 명령 전송 (Muscle) ---
                self.send_robot_move_cmd(robot_xy[0], robot_xy[1])

    def send_robot_move_cmd(self, x, y):
        """
        Doosan 로봇에게 이동 명령 전송 (Muscle)
        :param x: 목표 X 좌표 (mm)
        :param y: 목표 Y 좌표 (mm)
        :return: 서비스 호출 결과
        """
        req = MoveLine.Request()
        
        # 로봇 베이스 기준 목표 포즈 설정
        # pos: [x, y, z, rx, ry, rz]
        # Z축은 안전을 위해 지면에서 150mm 높이 유지, 툴 방향은 바닥을 수직으로 향함(180.0)
        req.pos = [float(x), float(y), 150.0, 0.0, 180.0, 0.0] 
        req.vel = 100.0     # 선속도 (mm/s)
        req.acc = 100.0     # 가속도 (mm/s^2)
        req.time = 0.0      # 도달 시간 (0일 경우 vel/acc 기준)
        
        # 비동기 서비스 호출 및 완료 대기
        future = self.move_line_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    """
    ROS2 노드 초기화 및 실행 진입점
    """
    rclpy.init(args=args)
    orchestrator = MainOrchestrator()
    orchestrator.run_collaboration_sequence()
    rclpy.shutdown()

if __name__ == '__main__':
    main()