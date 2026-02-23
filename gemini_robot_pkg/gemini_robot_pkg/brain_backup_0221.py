# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/brain.py

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
        
        # 1. Gemini API 설정 (Robotics ER 프리뷰 모델 사용)
        self.api_key = "AIzaSyA0AOB7tjo1NSuJx-s_AIKmFv36icA_sM8"
        self.client = genai.Client(api_key=self.api_key)
        self.model_name = "gemini-robotics-er-1.5-preview"
        
        # 2. 데이터 저장소 및 상태 관리
        self.object_db = {}
        self.is_scanning = True
        self.scan_duration = 15.0
        self.reasoning_interval = 20.0
        self.start_time = self.get_clock().now()
        
        # [추가] 사용자의 최신 음성 명령 저장 변수
        self.last_user_command = "현재 특별한 명령 없음. 도면에 따라 결선을 진행해." 
        
        # 3. ROS2 통신 설정
        self.subscription = self.create_subscription(
            String, '/eye/terminal_centers', self.terminal_callback, 10
        )
        self.coord_pub = self.create_publisher(String, '/brain/normalized_coords', 10)
        
        # [추가] 입(Mouth)으로 보낼 음성 토픽과 귀(Ear)에서 받을 명령 토픽 설정
        self.mouth_pub = self.create_publisher(String, '/mouth/speech_text', 10)
        self.ear_sub = self.create_subscription(
            String, '/ear/speech_text', self.ear_callback, 10
        )
        
        # 4. 리소스 경로
        self.circuit_diagram_path = os.path.expanduser('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/plc_circuit.png')
        self.relay_diagram_path = os.path.expanduser('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/relay_circuit.jpg')
        self.timer_diagram_path = os.path.expanduser('/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/timer_circuit.jpg')
        
        # 5. 제어 루프
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info("[Brain 노드] 귀(Ear)와 입(Mouth)이 연결되었습니다. 스캔 후 협업을 시작합니다.")

    # [추가] 귀(ear.py)에서 들려온 말을 저장하는 콜백 함수
    def ear_callback(self, msg):
        self.last_user_command = msg.data
        self.get_logger().info(f"👂 사용자 명령 수신: {self.last_user_command}")

    def terminal_callback(self, msg):
        try:
            detections = json.loads(msg.data)
            if self.is_scanning:
                for item in detections:
                    label = item['label']
                    point = item['point']
                    self.object_db[label] = point
        except Exception as e:
            self.get_logger().error(f"Callback Error: {e}")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        
        if self.is_scanning:
            if elapsed_time < self.scan_duration:
                self.get_logger().info(f"환경 스캔 중... ({elapsed_time:.1f}s / {self.scan_duration}s)")
            else:
                self.is_scanning = False
                self.get_logger().info("--- 스캔 완료: 환경 정보가 고정되었습니다. ---")
                self.timer.cancel()
                self.timer = self.create_timer(self.reasoning_interval, self.reasoning_step)
                self.reasoning_step()
        
    def reasoning_step(self):
        if not self.object_db:
            self.get_logger().warn("저장된 환경 정보가 없습니다. 스캔 실패 가능성.")
            return

        try:
            self.get_logger().info("Gemini에게 상황 분석 및 음성 답변 요청 중...")
            circuit_img = PIL.Image.open(self.circuit_diagram_path)
            relay_img = PIL.Image.open(self.relay_diagram_path)
            timer_img = PIL.Image.open(self.timer_diagram_path)

            # [수정] 프롬프트에 '사용자 음성 명령' 섹션을 추가하여 상황 인지 능력 강화
            prompt = f"""
            너는 Doosan M0609 로봇의 협업 지능이야. 
            제공된 PLC 회로도들과 [환경 데이터], 그리고 [사용자의 음성 명령]을 종합하여 작업 순서를 결정해줘.

            [사용자의 최근 음성 명령]:
            "{self.last_user_command}"

            [회로도 컴포넌트 - 실제 단자 라벨 매핑 정보]:
            - 푸시버튼 (PB1): PB1(1), PB1(2) / PB2: PB2(1), PB2(2) / PB3: PB3(1), PB3(2)
            - 릴레이 (R): relay 4, relay 3, relay 8, relay 5, relay 6, relay 7
            - 타이머 (T): timer 6, timer 7, timer 2, timer 8
            - 램프 (L1): L1(1), L1(2) / L2: L2(1), L2(2) / L3: L3(1), L3(2)
            - 전원: Power(1), Power(2)

            [환경 데이터 (현재 로봇이 알고 있는 좌표)]:
            {json.dumps(self.object_db, indent=2)}

            [수행 지침]:
            1. 사용자의 음성 명령이 있다면 최우선으로 반영해. (예: "작업 중단해"라고 하면 빈 리스트 반환)
            2. 결선 순서는 회로도 기준 왼쪽->오른쪽, 위->아래 원칙을 지켜.
            3. 인간에게 드라이버가 필요하면 'screwdriver'와 'palm' 좌표를 포함해.
            4. **중요**: 결과는 반드시 아래의 JSON 리스트 형식으로만 답해. 텍스트 설명은 하지마.
               [{{"step": 1, "label": "라벨명", "point": [y, x]}}, ...]
            """

            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[circuit_img, prompt, relay_img, timer_img],
                config=types.GenerateContentConfig(
                    temperature=0.0,
                    thinking_config=types.ThinkingConfig(thinking_budget=200)
                )
            )
            
            # [추가] Gemini의 응답이 오면 사용자에게 작업 시작을 알리는 기능 (Mouth 연동)
            speech_msg = String()
            speech_msg.data = "명령을 확인했습니다. 분석된 순서대로 협업을 시작합니다."
            self.mouth_pub.publish(speech_msg)

            # JSON 추출 및 퍼블리싱
            json_match = re.search(r'\[\s*\{.*\}\s*\]', response.text, re.DOTALL)
            if json_match:
                try:
                    result_data = json.loads(json_match.group())
                    sequence_labels = [item.get('label', 'N/A') for item in result_data]
                    self.get_logger().info(f"★★★ 확정 시퀀스 ★★★: {' -> '.join(sequence_labels)}")
                    
                    result_msg = String()
                    result_msg.data = json_match.group()
                    self.coord_pub.publish(result_msg)

                    # 사용한 명령은 초기화
                    self.last_user_command = "현재 특별한 명령 없음."
                except Exception as json_err:
                    self.get_logger().error(f"JSON 파싱 에러: {json_err}")
            else:
                self.get_logger().warn("JSON 형식을 찾을 수 없습니다.")

        except Exception as e:
            if "429" in str(e):
                self.get_logger().error("API 쿼터 초과.")
            else:
                self.get_logger().error(f"Reasoning Error: {e}")

def main(args=None):
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