# /home/ssy/cobot_ws/src/cobot2_ws/pick_and_place_voice/voice_processing/stt.py

"""
[코드 기능]: OpenAI Whisper API를 사용하여 5초간 음성을 녹음하고 텍스트로 변환(STT)하는 클래스
[입력(Input)]: 마이크를 통한 아날로그 음성 신호
[출력(Output)]: 변환된 문자열(String)
"""

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile

class STT:
    def __init__(self, openai_api_key):
        """
        [인풋]: openai_api_key (String) - OpenAI API 인증 키
        [아웃풋]: 없음 (클래스 인스턴스 초기화)
        """
        self.client = OpenAI(api_key=openai_api_key) # OpenAI 클라이언트 객체 생성
        self.duration = 5  # 녹음 지속 시간 (단위: 초)
        self.samplerate = 16000  # 샘플링 레이트 (Whisper 모델 최적화 값인 16kHz 설정)


    def speech2text(self):
        """
        [인풋]: 없음 (내부 마이크 장치 사용)
        [아웃풋]: transcript.text (String) - 인식된 텍스트 결과
        """
        # 녹음 시작 알림 및 설정된 시간만큼 음성 데이터 수집
        print("음성 녹음을 시작합니다. \n 5초 동안 말해주세요...")
        audio = sd.rec(int(self.duration * self.samplerate), samplerate=self.samplerate, channels=1, dtype='int16')
        sd.wait() # 녹음이 끝날 때까지 프로세스 대기
        print("녹음 완료. Whisper에 전송 중...")

        # API 전송을 위해 임시 WAV 파일 생성 (파일 시스템에 물리적 저장)
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            # 수집된 넘파이 배열 형태의 오디오를 WAV 파일 포맷으로 작성
            wav.write(temp_wav.name, self.samplerate, audio)

            # 생성된 임시 파일을 읽기 모드로 열어 Whisper API에 전달
            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f)

        # 변환된 최종 텍스트 출력 및 반환
        print("STT 결과: ", transcript.text)
        return transcript.text