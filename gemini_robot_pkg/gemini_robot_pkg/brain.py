# /home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/gemini_robot_pkg/brain.py

"""
- [코드 기능]: 
    1. 초기 10초간 '스캔 모드'를 통해 작업 공간 내 단자, 도구, 손바닥 좌표를 수집하여 DB화함.
    2. 스캔 완료 후, API 쿼터 보호를 위해 20초 주기로 회로도 분석 및 협업 시퀀스 수행.
    3. 정규화된 좌표([y, x])를 신경 노드(nerve.py)로 발행.
- [입력(Input)]: /eye/terminal_centers (실시간 YOLO 탐지 데이터)
- [출력(Output)]: /brain/normalized_coords (Gemini가 결정한 목표 정규화 좌표)
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
        
        # 1. Gemini API 설정 (Robotics ER 프리뷰 모델 사용)
        self.api_key = "AIzaSyA0AOB7tjo1NSuJx-s_AIKmFv36icA_sM8" # cobot2 라는 api key 입력하면 됨 ㅇㅇ
        self.client = genai.Client(api_key=self.api_key)
        self.model_name = "gemini-robotics-er-1.5-preview"
        
        # 2. 데이터 저장소 및 상태 관리
        self.object_db = {}          # 10초간 수집된 객체별 고정 좌표 저장소
        self.is_scanning = True      # 현재 스캔 모드 여부
        self.scan_duration = 15.0    # 스캔 지속 시간 (초)
        self.reasoning_interval = 20.0 # [중요] 쿼터 초과 방지를 위한 20초 주기 (분당 3회 호출)
        self.start_time = self.get_clock().now()
        
        # 3. ROS2 통신 설정
        self.subscription = self.create_subscription(
            String, '/eye/terminal_centers', self.terminal_callback, 10
        )
        self.coord_pub = self.create_publisher(String, '/brain/normalized_coords', 10)
        
        # 4. 리소스 경로
        self.circuit_diagram_path = os.path.expanduser(
            '/home/ssy/cobot_ws/src/cobot2_ws/gemini_robot_pkg/resource/plc_circuit.png'
        )
        
        # 5. 제어 루프 (스캔 확인 및 추론 실행)
        # 초기에는 1초마다 스캔 종료 여부 체크
        self.timer = self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info("[Brain 노드] 가동: 15초간 주변 환경 스캔 후 쿼터 보호 모드로 전환합니다.")

    def terminal_callback(self, msg):
        """실시간 YOLO 탐지 정보를 수신하여 스캔 모드일 경우 DB 업데이트"""
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
        """스캔 완료 여부를 체크하고 모드를 전환함"""
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        
        if self.is_scanning:
            if elapsed_time < self.scan_duration:
                self.get_logger().info(f"환경 스캔 중... ({elapsed_time:.1f}s / {self.scan_duration}s)")
            else:
                self.is_scanning = False
                self.get_logger().info("--- 스캔 완료: 환경 정보가 고정되었습니다. ---")
                self.get_logger().info(f"수집된 객체 리스트: {list(self.object_db.keys())}")
                
                # 스캔 종료 후 타이머를 20초 주기로 재설정 (쿼터 보호 핵심)
                self.timer.cancel()
                self.timer = self.create_timer(self.reasoning_interval, self.reasoning_step)
                
                # 대기 없이 즉시 첫 번째 추론 실행
                self.reasoning_step()
        
    def reasoning_step(self):
        """저장된 Object DB를 기반으로 회로도 분석 및 다음 좌표 결정"""
        if not self.object_db:
            self.get_logger().warn("저장된 환경 정보가 없습니다. 스캔 실패 가능성.")
            return

        try:
            self.get_logger().info("Gemini에게 다음 협업 단계를 묻는 중... (API 호출)")
            circuit_img = PIL.Image.open(self.circuit_diagram_path)
            
            # 프롬프트 초안 _ 영어
            # prompt = f"""
            # You are the collaborative intelligence of a Doosan M0609 robot. 
            # Perform PLC circuit wiring tasks based on the pre-scanned [Environment Data] below.

            # [Environment Data (Fixed Coordinates)]:
            # {json.dumps(self.object_db, indent=2)}

            # [Instructions]:
            # 1. Analyze the circuit diagram to find the terminal (e.g., timer 2, relay 6) for the current wiring sequence.
            # 2. Retrieve the [y, x] coordinates for the **exactly matching label** from the [Environment Data].
            # 3. If you need to deliver a screwdriver to a human, utilize the 'screwdriver' and 'palm' coordinates.
            # 4. The output MUST be provided ONLY in the following JSON format: [{{"point": [y, x], "label": "label_name"}}]
            # """

            # ==================================================================================

            # 프롬프트 초안 _ 한국어
            # prompt = f"""
            # 너는 Doosan M0609 로봇의 협업 지능이야. 
            # 미리 스캔된 아래의 [환경 데이터]를 기반으로 PLC 회로도 작업을 수행해.

            # [환경 데이터 (고정 좌표)]:
            # {json.dumps(self.object_db, indent=2)}

            # [수행 지침]:
            # 1. 회로도에서 현재 결선 순서에 맞는 단자(예: timer 2, relay 6)를 찾아.
            # 2. [환경 데이터]에서 해당 단자와 **정확히 일치하는 label**의 좌표를 가져와.
            # 3. 인간에게 드라이버를 전달해야 할 경우 'screwdriver'와 'palm' 좌표를 활용해.
            # 4. 결과는 반드시 [{{"point": [y, x], "label": "라벨명"}}] 형식의 JSON으로만 답해.
            # """

            # ==================================================================================

            # 프롬프트 v2 _ 한국어
            # prompt = f"""
            # 너는 Doosan M0609 로봇의 협업 지능이야. 
            # 제공된 PLC 회로도를 분석하여 [환경 데이터]에 포함된 모든 단자들의 전체 결선 작업 순서를 결정해줘.

            # [환경 데이터 (고정 좌표)]:
            # {json.dumps(self.object_db, indent=2)}

            # [수행 지침]:
            # 1. 회로도와 [환경 데이터]를 대조하여, 작업해야 할 모든 단자들의 논리적인 작업 순서를 정해.
            # 2. 인간에게 드라이버를 전달해야 할 시점이 있다면 'screwdriver'와 'palm' 위치도 순서에 포함시켜.
            # 3. 결과는 반드시 작업 순서대로 정렬된 [{{"step": 1, "label": "라벨명", "point": [y, x]}}, ...] 형식의 JSON으로만 답해.
            # """

            # ==================================================================================
            
            # 프롬프트 v3 _ 한국어

            prompt = f"""
            너는 Doosan M0609 로봇의 협업 지능이야. 
            제공된 PLC 회로도를 분석하여 [환경 데이터]에 포함된 단자들의 전체 결선 작업 순서를 결정해줘.

            [회로도 컴포넌트 - 실제 단자 라벨 매핑 정보]:
            - 푸시버튼 (PB1): PB1(1), PB1(2)
            - 푸시버튼 (PB2): PB2(1), PB2(2)
            - 푸시버튼 (PB3): PB3(1), PB3(2)
            - 릴레이 (R): relay 4, relay 3, relay 8, relay 5, relay 6, relay 7
            - 타이머 (T): timer 6, timer 7, timer 2, timer 8
            - 램프 (L1): L1(1), L1(2)
            - 램프 (L2): L2(1), L2(2)
            - 램프 (L3): L3(1), L3(2)
            - Plus 전원 (Power1): Power(1)
            - Minus 전원 (Power2) : Power(2)

            [환경 데이터 (현재 로봇이 알고 있는 좌표)]:
            {json.dumps(self.object_db, indent=2)}

            [결선 순서 결정 원칙]:
            1. 회로도 도면을 기준으로 **왼쪽에서 오른쪽으로, 위에서 아래로** 흐르는 결선 순서를 준수해.
            2. 위 매핑 정보를 참조하여 회로도 상의 기호가 [환경 데이터]의 어떤 라벨에 해당하는지 정확히 파악해.
            3. 인간 작업자가 드라이버를 필요로 하는 시점에는 'screwdriver'와 'palm' 좌표를 작업 순서 사이에 포함시켜.
            4. 결과는 반드시 작업 순서대로 정렬된 아래 JSON 리스트 형식으로만 답해:
               [{{"step": 1, "label": "라벨명", "point": [y, x]}}, ...]
            """

            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[circuit_img, prompt],
                config=types.GenerateContentConfig(
                    temperature=0.0,
                    thinking_config=types.ThinkingConfig(thinking_budget=200)
                )
            )
            
            # JSON 추출 및 퍼블리싱 (nerve.py 전송)
            json_match = re.search(r'\[\s*\{.*\}\s*\]', response.text, re.DOTALL)
            if json_match:
                try:
                    result_data = json.loads(json_match.group())
                    
                    # [추가] 터미널에 전체 작업 시퀀스 요약 출력
                    sequence_labels = [item.get('label', 'N/A') for item in result_data]
                    self.get_logger().info(f"★★★ 확정된 전체 작업 시퀀스 ★★★: {' -> '.join(sequence_labels)}")
                    
                    # nerve.py로 전체 리스트 전송
                    result_msg = String()
                    result_msg.data = json_match.group()
                    self.coord_pub.publish(result_msg)
                except Exception as json_err:
                    self.get_logger().error(f"JSON 파싱 에러: {json_err}")
            else:
                self.get_logger().warn("Gemini의 응답에서 JSON 형식을 찾을 수 없습니다.")

        except Exception as e:
            # 429 에러 발생 시 로그 출력 후 다음 주기 대기
            if "429" in str(e):
                self.get_logger().error("API 쿼터가 초과되었습니다. 다음 호출까지 대기합니다.")
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
