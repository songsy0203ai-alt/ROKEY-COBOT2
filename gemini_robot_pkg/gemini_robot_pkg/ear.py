import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile
import os
import time

# 최신 Google Gen AI SDK (2026 기준)
from google import genai
from google.genai import types

class EarNode(Node):
    def __init__(self):
        super().__init__('ear_node')
        
        # 1. 설정: API 키 및 클라이언트 초기화
        # 주의: 실제 배포 시에는 환경 변수 사용을 권장합니다.
        self.api_key = "AIzaSyAyyHw26J8N-MHQlcYvUkyGggrBOPoSJGU"
        self.client = genai.Client(api_key=self.api_key)
        self.model_id = "gemini-2.5-flash" # 2026년형 고속 멀티모달 모델
        
        self.duration = 5      # 녹음 시간 (초)
        self.samplerate = 16000 # 음성 인식에 적합한 샘플링 레이트
        
        # 2. 통신 설정: 인식된 텍스트를 발행할 Topic
        self.speech_pub = self.create_publisher(String, '/ear/speech_text', 10)
        
        self.get_logger().info("👂 귀(Ear) 노드가 가동되었습니다. (google-genai SDK 적용)")
        
        # 3. 주기적 실행을 위한 타이머
        # 1.0초 간격으로 체크하지만, 실제로는 녹음(5초) 후에 다음 루프가 실행됩니다.
        self.create_timer(1.0, self.stt_callback)

    def stt_callback(self):
        """음성 인식 프로세스를 반복 실행하는 콜백"""
        text = self.speech2text()
        if text:
            msg = String()
            msg.data = text.strip()
            self.speech_pub.publish(msg)
            self.get_logger().info(f"✅ 인식 결과: {text}")

    def speech2text(self):
        """음성 녹음 -> Gemini API 전송 -> 텍스트 변환"""
        self.get_logger().info("🎤 주변 소리를 듣는 중...")
        
        try:
            # 음성 녹음 시작
            recording = sd.rec(int(self.duration * self.samplerate), 
                               samplerate=self.samplerate, channels=1)
            sd.wait() # 녹음이 끝날 때까지 대기
            
            # 임시 파일 생성 및 오디오 저장
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_file:
                wav.write(tmp_file.name, self.samplerate, recording)
                tmp_path = tmp_file.name

            # 4. 최신 SDK 방식의 멀티모달 요청
            with open(tmp_path, "rb") as f:
                audio_bytes = f.read()
                
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=[
                    "당신은 음성 인식을 담당하는 AI입니다. 다음 오디오를 듣고 텍스트로만 정확히 받아쓰세요. 배경 소음은 무시합니다.",
                    types.Part.from_bytes(data=audio_bytes, mime_type="audio/wav")
                ]
            )
            
            # 결과 파일 삭제 및 텍스트 반환
            os.remove(tmp_path)
            return response.text if response.text else None
            
        except Exception as e:
            self.get_logger().error(f"❌ 오류 발생: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = EarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 노드를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()