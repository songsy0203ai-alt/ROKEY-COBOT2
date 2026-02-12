#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import DR_init
import time
import json
import datetime as dt
import numpy as np
import pyrealsense2 as rs
import cv2
from PIL import Image
import google.generativeai as genai
from std_msgs.msg import Bool, String

# # =========================================================
# # ✅ GEMINI API 설정 (여기에 본인의 API KEY를 입력하세요)
# # =========================================================
# GEMINI_API_KEY = "석진님_api_key"

# # API 키가 설정되지 않았을 경우를 대비한 안전장치
# if GEMINI_API_KEY == "석진님_api_key":
#     print("경고: GEMINI_API_KEY가 설정되지 않았습니다. 코드를 수정해주세요.")

# genai.configure(api_key=GEMINI_API_KEY)
# model = genai.GenerativeModel('gemini-robotics-er-1.5-preview')

# =========================
# 로봇 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight3"
ROBOT_TCP = "GripperDA_v1_3"

ON, OFF = 1, 0

ROS_LOG_TOPIC = "/m0609/job_event"
STOP_TOPIC = "/stop_request"
RESTART_TOPIC = f"/{ROBOT_ID}/restart"
START_TOPIC = "start"
FINISH_TOPIC = "first"

# =========================
# 전역 변수
# =========================
start_triggered = False
running = False
stop_requested = False
rs_pipeline = None
rs_intrinsics = None

ABORT_STATES = {3, 5, 6, 9, 10, 15}

# =========================
# 유틸리티 함수
# =========================
def now_iso():
    return dt.datetime.now().isoformat(timespec="milliseconds")

def publish_log(node, log_pub, level, category, event, message, payload=None):
    data = {
        "ts_robot": now_iso(), "level": level, "category": category, 
        "event": event, "message": message, "payload": payload or {}
    }
    msg = String()
    msg.data = json.dumps(data, ensure_ascii=False)
    log_pub.publish(msg)

# # =========================
# # ✅ Gemini 통신 함수
# # =========================
# def ask_gemini_for_coordinates(pipeline):
#     """
#     RealSense에서 이미지를 캡처하여 Gemini에게 보내고,
#     나사 구멍의 픽셀 좌표 [x, y]를 받아옵니다.
#     """
#     # 1. RealSense 이미지 캡처
#     frames = pipeline.wait_for_frames()
#     color_frame = frames.get_color_frame()
#     if not color_frame:
#         return None

#     # numpy array -> PIL Image 변환
#     color_image = np.asanyarray(color_frame.get_data())
#     pil_image = Image.fromarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))

#     # 2. Gemini에게 보낼 프롬프트 구성
#     prompt = """
#     Look at this image of an electrical terminal block.
#     I need to drive a screw into the empty terminal hole.
#     Find the center pixel coordinates [x, y] of the target screw hole.
    
#     Output format must be strictly JSON:
#     {"point": [x, y]}
    
#     Do not add any markdown formatting or explanation. Just the JSON.
#     """

#     # 3. Gemini 호출
#     try:
#         response = model.generate_content([prompt, pil_image])
#         text_res = response.text.strip()
        
#         # Markdown backticks 제거
#         if text_res.startswith("```"):
#             text_res = text_res.replace("```json", "").replace("```", "")
        
#         result = json.loads(text_res)
#         return result["point"]  # [x, y] 반환
#     except Exception as e:
#         print(f"Gemini API Error: {e}")
#         return None

# # =========================
# # ✅ 좌표 변환 (Hand-Eye Calibration) - [수정됨]
# # =========================
# def convert_camera_to_base(cam_x, cam_y, cam_z, robot_current_pos):
#     """
#     카메라 좌표계(Camera Frame)의 위치를 로봇 베이스 좌표계(Base Frame)로 변환합니다.
#     주의: 실제 현장에 맞는 Hand-Eye Calibration Matrix가 필요하지만,
#     여기서는 카메라가 그리퍼 끝에 달려있고 축이 정렬되어 있다고 가정한 약식 계산입니다.
#     """
#     # 현재 로봇 끝단(Tool) 위치 (mm 단위)
#     tool_x, tool_y, tool_z = robot_current_pos[0], robot_current_pos[1], robot_current_pos[2]
    
#     # 카메라가 툴 중심에서 얼마나 떨어져 있는지 (Offset, mm 단위)
#     # [사용자 수정 필요] 실제 카메라 설치 위치에 따라 수정하세요.
#     CAMERA_OFFSET_X = 0.0
#     CAMERA_OFFSET_Y = 50.0  # 예: 카메라가 툴보다 Y축으로 50mm 옆에 있음
#     CAMERA_OFFSET_Z = -100.0 # 예: 카메라가 툴 끝보다 100mm 위에 있음

#     # 카메라 좌표(m)를 로봇 좌표(mm) 스케일로 변환
#     # RealSense 좌표계: X(우), Y(하), Z(전방) -> 로봇 좌표계 변환 필요
#     # 로봇이 바닥을 보고 있을 때(Rx=180, Ry=180)를 기준으로 가정:
#     # 카메라 X -> 로봇 -X
#     # 카메라 Y -> 로봇 -Y
#     # 카메라 Z -> 로봇 -Z (깊이)
    
#     rel_x = -cam_x * 1000
#     rel_y = -cam_y * 1000
#     rel_z = -cam_z * 1000

#     # 최종 좌표 계산 (현재 툴 위치 + 오프셋 + 카메라 상대 위치)
#     base_x = tool_x + CAMERA_OFFSET_X + rel_x
#     base_y = tool_y + CAMERA_OFFSET_Y + rel_y
#     base_z = tool_z + CAMERA_OFFSET_Z + rel_z

#     return base_x, base_y, base_z

# =========================
# 작업 로직
# =========================
def perform_screw_task(node, finish_pub, log_pub, get_robot_state):
    from DSR_ROBOT2 import (
        posx, posj, movej, movel, move_home, wait,
        task_compliance_ctrl, set_stiffnessx, set_desired_force,
        release_force, release_compliance_ctrl, set_digital_output,
        get_tool_force, get_current_posx, DR_FC_MOD_REL, DR_TOOL, DR_HOME_TARGET_USER, DR_MV_MOD_REL
    )
    
    global rs_pipeline, rs_intrinsics, stop_requested

    # 안전 정지 체크 함수
    def abort_if_safety(where: str):
        if stop_requested:
            publish_log(node, log_pub, "WARN", "STOP", "WEB.STOP_ABORT", f"STOP @ {where}")
            raise RuntimeError(f"WEB_STOP_ABORT: {where}")
        st = get_robot_state()
        if st in ABORT_STATES:
            publish_log(node, log_pub, "ERROR", "SAFETY", "SAFETY.ABORT", f"State {st} @ {where}")
            raise RuntimeError(f"SAFETY_ABORT: {st}")

    def grip(on=True):
        val1, val2 = (ON, OFF) if on else (OFF, ON)
        set_digital_output(1, val1); set_digital_output(2, val2)
        wait(0.5)

    # 초기 위치 설정
    # P_SCAN: 카메라가 바닥을 잘 볼 수 있는 위치 (높이 조절 필요)
    P_SCAN = posj(0, 0, 90, 0, 90, 0) 
    
    # 드라이버 픽업 위치
    P_TOOL_PICK = posx(431.01, 221.37, 282.923, 180, 180, 180)
    P_TOOL_PICK_DOWN = posx(431.01, 221.37, 84.923, 180, 180, 180)

    publish_log(node, log_pub, "INFO", "TASK", "TASK.START", "Gemini 연동 작업 시작")

    # 1. 드라이버 잡기
    abort_if_safety("move home")
    move_home(DR_HOME_TARGET_USER)
    
    abort_if_safety("pick driver")
    grip(False)
    movel(P_TOOL_PICK, time=2); movel(P_TOOL_PICK_DOWN, time=2)
    grip(True)
    movel(P_TOOL_PICK, time=2)

    # 2. 스캔 위치 이동
    abort_if_safety("scan pose")
    publish_log(node, log_pub, "INFO", "MOTION", "STEP.SCAN_POSE", "Gemini 시각 스캔 위치 이동")
    movej(P_SCAN, time=2)
    wait(1.5) # 이미지 안정화 대기 (조금 더 여유 있게)

    # 3. Gemini에게 좌표 요청
    publish_log(node, log_pub, "INFO", "AI", "GEMINI.REQUEST", "Gemini에게 좌표 요청 중...")
    
    target_pixel = ask_gemini_for_coordinates(rs_pipeline)
    
    if target_pixel is None:
        publish_log(node, log_pub, "ERROR", "AI", "GEMINI.FAIL", "좌표 수신 실패")
        raise RuntimeError("Gemini가 좌표를 찾지 못했습니다.")
    
    u, v = int(target_pixel[0]), int(target_pixel[1])
    publish_log(node, log_pub, "INFO", "AI", "GEMINI.RESPONSE", f"Gemini 응답: [{u}, {v}]")

    # 4. Depth 측정 및 변환 (Hybrid Logic)
    frames = rs_pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    dist_m = depth_frame.get_distance(u, v)
    
    # Depth 0일 경우 간단한 보정 (주변 3x3 픽셀 평균) - [추가됨]
    if dist_m <= 0:
        dist_list = []
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                d = depth_frame.get_distance(u+dx, v+dy)
                if d > 0: dist_list.append(d)
        if dist_list:
            dist_m = sum(dist_list) / len(dist_list)
        else:
            raise RuntimeError("Depth 측정 실패 (유효하지 않은 거리)")

    # 3D 좌표 변환 (Camera Frame)
    camera_xyz = rs.rs2_deproject_pixel_to_point(rs_intrinsics, [u, v], dist_m)
    cam_x, cam_y, cam_z = camera_xyz
    
    # 로봇 베이스 좌표 변환
    curr_pos = get_current_posx()[0]
    target_x, target_y, target_z_floor = convert_camera_to_base(cam_x, cam_y, cam_z, curr_pos)
    
    publish_log(node, log_pub, "INFO", "AI", "CALC.COORD", f"목표 좌표: X={target_x:.1f}, Y={target_y:.1f}")

    # 5. 접근 (Approach)
    SAFE_Z_OFFSET = 50.0 # 충돌 방지를 위해 50mm 위로 접근 - [값 상향 조정]
    # Rx, Ry, Rz는 180, 180, 0 (그리퍼가 바닥을 보는 자세)로 고정
    P_APPROACH = posx(target_x, target_y, target_z_floor + SAFE_Z_OFFSET, 180, 180, 0)
    
    abort_if_safety("approach")
    movel(P_APPROACH, time=2)
    
    # 6. 터치 센싱 (힘 제어) - [복구 및 완성됨]
    publish_log(node, log_pub, "INFO", "FORCE", "FORCE.SEARCH", "높이 탐색 시작")
    
    task_compliance_ctrl()
    set_stiffnessx([3000, 3000, 1000, 1000, 1000, 1000])
    # Z축 아래로 5N 힘 가하기
    set_desired_force(fd=[0,0,-5,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)

    t_start = time.time()
    contact = False

    while rclpy.ok():
        abort_if_safety("searching_contact")
        
        # 힘 센서 확인
        f = get_tool_force(DR_TOOL)
        if abs(f[2]) > 5.0: # 5N 이상 감지 시
            publish_log(node, log_pub, "INFO", "FORCE", "FORCE.CONTACT", "바닥 감지 성공")
            contact = True
            release_force(time=0.1) # 누르기 중단
            break
        
        # 타임아웃
        if time.time() - t_start > 10.0:
            publish_log(node, log_pub, "ERROR", "FORCE", "FORCE.TIMEOUT", "바닥을 찾지 못함")
            release_force(time=0.1)
            release_compliance_ctrl()
            raise RuntimeError("Force Search Timeout")
        
        wait(0.01)

    if not contact:
        release_compliance_ctrl()
        raise RuntimeError("Contact Failed")

    # 7. 나사 체결 (Screwing) - [복구 및 완성됨]
    publish_log(node, log_pub, "INFO", "SCREW", "SCREW.PROCESS", "나사 체결 수행")
    
    # 현재 위치에서 90도씩 4회전
    P_ROT = posx(0,0,0,0,0,90)
    for _ in range(4):
        abort_if_safety("screwing")
        movel(P_ROT, time=0.5, mod=DR_MV_MOD_REL)

    # 8. 복귀 및 종료
    release_compliance_ctrl() # 힘 제어 해제
    
    # 위로 살짝 들기 (Retract)
    movel(posx(0,0,-50,0,0,0), time=1, mod=DR_MV_MOD_REL)
    
    # 드라이버 반납 (필요 시 주석 해제)
    # abort_if_safety("return driver")
    # movel(P_TOOL_PICK, time=3)
    # ...
    
    move_home(DR_HOME_TARGET_USER)
    
    finish_pub.publish(Bool(data=True))
    publish_log(node, log_pub, "INFO", "TASK", "TASK.FINISH", "작업 완료")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("bolt_node_gemini", namespace=ROBOT_ID)
    
    log_pub = node.create_publisher(String, ROS_LOG_TOPIC, 10)
    finish_pub = node.create_publisher(Bool, FINISH_TOPIC, 10)

    # RealSense 초기화
    global rs_pipeline, rs_intrinsics
    try:
        rs_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = rs_pipeline.start(config)
        rs_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    except Exception as e:
        node.get_logger().error(f"RS Init Fail: {e}")
        return

    # DR Init 및 Main Loop
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    global start_triggered, running, stop_requested
    
    def start_cb(msg: Bool):
        global start_triggered
        if msg.data: start_triggered = True
        
    def stop_cb(msg: Bool):
        global stop_requested
        if msg.data: stop_requested = True
        
    node.create_subscription(Bool, START_TOPIC, start_cb, 10)
    node.create_subscription(Bool, STOP_TOPIC, stop_cb, 10)

    try:
        from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp, get_robot_state, ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        
        publish_log(node, log_pub, "INFO", "SYSTEM", "NODE_START", "Gemini Node 준비 완료")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            
            # STOP 요청 시 리셋
            if stop_requested:
                # 필요한 경우 리셋 로직 추가
                pass

            if start_triggered and not running:
                running = True
                start_triggered = False
                stop_requested = False
                try:
                    perform_screw_task(node, finish_pub, log_pub, get_robot_state)
                except Exception as e:
                    publish_log(node, log_pub, "ERROR", "SYSTEM", "EXCEPTION", f"{e}")
                finally:
                    running = False
            time.sleep(0.01)
    finally:
        if rs_pipeline: rs_pipeline.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()