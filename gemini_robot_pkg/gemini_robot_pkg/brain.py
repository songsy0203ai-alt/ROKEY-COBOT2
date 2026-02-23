# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/brain.py

"""
[코드 기능]
- Doosan M0609 협동 로봇의 의사결정 체계(Brain)를 담당하는 ROS2 노드입니다.
- 환경 스캔(Eye), 음성 명령(Ear) 데이터를 취합하여 Gemini AI 모델에 전달하고, 
  회로도 분석을 통해 최적의 결선 작업 단자를 결정합니다.
- 사용자 승인(Mouth/Ear) 절차를 거쳐 최종 좌표를 하위 제어 노드(Nerve)로 전송합니다.

[입력(Input)]
1. /eye/terminal_centers (std_msgs/String): 카메라를 통해 탐지된 단자들의 라벨 및 정규화된 좌표 (JSON 형식).
2. /ear/speech_text (std_msgs/String): 사용자의 음성 명령 텍스트.
3. 이미지 파일: PLC 회로도, 릴레이 회로도, 타이머 회로도 (Local Path).

[출력(Output)]
1. /brain/normalized_coords (std_msgs/String): 승인된 작업 대상 단자의 라벨 및 [y, x] 좌표 (JSON 리스트).
2. /mouth/speech_text (std_msgs/String): 로봇이 사용자에게 보내는 질문 및 상태 안내 텍스트.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from google import genai
from google.genai import types
import json
import os
import re
import PIL.Image
from rclpy.time import Time

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # 1. Gemini API 설정
        # API 키와 모델 이름을 설정하여 멀티모달 추론 환경을 준비합니다.
        self.api_key = "AIzaSyA0AOB7tjo1NSuJx-s_AIKmFv36icA_sM8"
        self.client = genai.Client(api_key=self.api_key)
        self.model_name = "gemini-robotics-er-1.5-preview"
        
        # 2. 상태 관리 및 데이터 저장소
        # 로봇의 현재 공정 상태를 관리하며, 시각 데이터를 저장하는 DB와 명령 변수를 초기화합니다.
        self.state = 'SCANNING' # 초기 상태: 주변 환경 스캔
        self.object_db = {}     # '라벨': [y, x] 형태의 탐지 데이터 저장
        self.current_proposed_task = None 
        self.last_user_command = "현재 특별한 명령 없음. 회로도에 근거하여 최적의 결선 순서를 결정해."
            # last_user_command는 사용자가 아직 별다른 말(예: '중단해', '6번부터 해')을 하지 않았으니, 
            # "너는 기본적으로 도면에 적힌 원칙대로 순서를 짜라"고 Gemini에게 가이드라인을 주는 텍스트일 뿐입니다.
        
        # 스캔 유지 시간 설정 (15초간 Eye 데이터를 수집)
        self.scan_duration = 15.0
        self.start_time = self.get_clock().now()
        
        # 3. ROS2 통신 설정
        # 외부 노드로부터 데이터를 받고(Sub), 결정된 명령을 전달(Pub)하기 위한 인터페이스 정의
        self.eye_sub = self.create_subscription(String, '/eye/terminal_centers', self.eye_callback, 10)
        self.ear_sub = self.create_subscription(String, '/ear/speech_text', self.ear_callback, 10)
        self.coord_pub = self.create_publisher(String, '/brain/normalized_coords', 10)
        self.mouth_pub = self.create_publisher(String, '/mouth/speech_text', 10)
        
        # 4. 리소스 경로 설정
        # 회로도 분석을 위해 Gemini에게 전달할 참조 이미지 경로를 설정합니다.
        self.circuit_diagram_path = os.path.expanduser('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/plc_circuit.png')
        self.relay_diagram_path = os.path.expanduser('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/relay_circuit.jpg')
        self.timer_diagram_path = os.path.expanduser('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/timer_circuit.jpg')
        
        # 5. 제어 루프 타이머
        # 1초 주기로 현재 상태를 점검하고 상태 전이를 관리합니다.
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info(f"[Brain 노드] 초기화 완료. 현재 상태: {self.state}")

    def eye_callback(self, msg):
        """
        [Input] msg: eye.py에서 Yolo로 탐지된 객체의 라벨과 좌표가 포함된 JSON 문자열
        [Output] None (클래스 내부 self.object_db(단자별 좌표 데이터베이스. Gemini가 작업 환경 공간을 이해하는데 쓰임.) 업데이트)
        기능: 카메라 노드(Eye)에서 오는 실시간 좌표 데이터를 로봇의 기억 장치(object_db)에 갱신합니다.
        """
        try:
            detections = json.loads(msg.data)
            for item in detections:
                self.object_db[item['label']] = item['point']
        except Exception as e:
            self.get_logger().error(f"Eye Callback Error: {e}")

    def ear_callback(self, msg):
        """
        [Input] msg: 사용자의 음성 발화 텍스트
        [Output] None (상태 전환 및 execute_task 호출 제어)
        기능: 사용자의 답변을 분석하여 로봇의 제안을 승인할지, 아니면 다시 추론할지 결정합니다.
        """
        user_talk = msg.data.strip()
        self.get_logger().info(f"👂 사용자 음성 수신: {user_talk} (현재 상태: {self.state})")

        # 로봇이 질문을 던지고 답변을 기다리는 상태인 경우
        if self.state == 'WAITING_APPROVAL':
            # 긍정 표현이 포함된 경우 작업을 실행함
            if any(word in user_talk for word in ["응", "어", "그래", "작업해", "오케이", "수행해"]):
                self.get_logger().info("✅ 승인 확인. 로봇에게 좌표를 전송합니다.")
                self.execute_task()
             
            # 부정 또는 수정 표현이 포함된 경우 다시 분석함
            elif any(word in user_talk for word in ["아니", "하지마", "말고", "다른거"]):
                self.get_logger().warn("❌ 거절 또는 수정 요청 수신. 재분석을 시작합니다.")
                self.last_user_command = user_talk
                self.reasoning_step()
        
        else:
            # 기타 상황에서는 사용자의 말을 기록하여 다음 추론의 맥락으로 활용
            self.last_user_command = user_talk

    def control_loop(self):
        """
        [Input/Output] None
        기능: 노드의 주기적 상태 점검. SCANNING 시간이 종료되면 REASONING 단계로 자동 전환합니다.
        """
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        
        if self.state == 'SCANNING':
            if elapsed_time < self.scan_duration:
                self.get_logger().info(f"환경 스캔 중... ({elapsed_time:.1f}s / {self.scan_duration}s)")
            else:
                self.get_logger().info("--- 스캔 완료: 추론 단계로 진입합니다. ---")
                self.reasoning_step()

    def reasoning_step(self):
        """
        [Input] self.object_db, 회로도 이미지, 사용자 명령 텍스트
        [Output] None (Mouth 노드로 질문 텍스트 송신)
        기능: Gemini AI에 회로도와 현재 환경 정보를 전달하여 다음 작업 타겟을 결정하고 사용자에게 묻습니다.
        """
        if not self.object_db:
            self.get_logger().warn("저장된 환경 정보가 없습니다. 스캔 데이터를 기다립니다.")
            return

        self.state = 'REASONING'
        try:
            self.get_logger().info("Gemini에게 작업 시퀀스 분석 요청 중...")
            # 회로도 이미지 로드
            circuit_img = PIL.Image.open(self.circuit_diagram_path)
            relay_img = PIL.Image.open(self.relay_diagram_path)
            timer_img = PIL.Image.open(self.timer_diagram_path)

            # AI에게 전달할 프롬프트 구성 (환경 데이터와 사용자 의도 포함)
            prompt = f"""
            너는 Doosan M0609 로봇의 협업 지능이야. 
            제공된 PLC 회로도와 [환경 데이터], 그리고 [사용자의 음성 명령]을 종합하여 단 하나의 최우선 작업 단자를 결정해.

            [사용자의 최근 음성 명령]: "{self.last_user_command}" # Gemini가 공간 추론 및 환경 이해를 수행하도록 명령
            [환경 데이터]: {json.dumps(self.object_db, indent=2)}

            [수행 지침]:
            1. 사용자의 음성 명령이 있다면 최우선으로 반영해.
            2. 결과는 반드시 아래의 JSON 리스트 형식으로만 답해. 텍스트 설명은 생략해.
               [{{"step": 1, "label": "라벨명", "point": [y, x]}}]
            """

            # Gemini 멀티모달 추론 실행
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[circuit_img, prompt, relay_img, timer_img],
                config=types.GenerateContentConfig(temperature=0.0)
            )
            
            # 응답에서 JSON 데이터만 추출
            json_match = re.search(r'\[\s*\{.*\}\s*\]', response.text, re.DOTALL)
            if json_match:
                result_data = json.loads(json_match.group())
                if result_data:
                    self.current_proposed_task = result_data[0]
                    label = self.current_proposed_task.get('label', '알 수 없는 단자')
                    
                    # 사용자에게 확인 질문 전송 (Mouth 노드 연동)
                    speech_msg = String()
                    speech_msg.data = f"{label} 작업을 진행할까요?"
                    self.mouth_pub.publish(speech_msg)
                    
                    self.state = 'WAITING_APPROVAL'
                    self.get_logger().info(f"질문 송신 완료: {label}. 승인을 기다립니다.")
            else:
                self.get_logger().error("Gemini 응답에서 유효한 JSON을 찾지 못했습니다.")
                self.state = 'IDLE'

        except Exception as e:
            self.get_logger().error(f"Reasoning Error: {e}")
            self.state = 'IDLE'

    def execute_task(self):
        """
        [Input] self.current_proposed_task (Gemini가 결정한 데이터)
        [Output] None (Nerve 노드로 Gemini 식으로 정규화 된 좌표 데이터 송신)
        기능: 사용자가 승인한 단자의 좌표 정보(Gemini 식으로 정규화 된 좌표)를 최종적으로 물리 제어 노드(Nerve)에 전달합니다.
        """
        if self.current_proposed_task:
            self.state = 'EXECUTING'
            
            # Nerve 노드가 인식할 수 있는 JSON 리스트 형식으로 좌표 데이터 직렬화
            result_msg = String()
            result_msg.data = json.dumps([self.current_proposed_task])
            self.coord_pub.publish(result_msg)
            
            self.get_logger().info(f"🚀 실행 명령 전송됨: {self.current_proposed_task['label']}")
            
            # 상태 초기화 및 다음 대기 상태로 전환
            self.last_user_command = "현재 특별한 명령 없음."
            self.state = 'IDLE' 
        else:
            self.get_logger().error("실행할 작업 정보가 없습니다.")

def main(args=None):
    # ROS2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()