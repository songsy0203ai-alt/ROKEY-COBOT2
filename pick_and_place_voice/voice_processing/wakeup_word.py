# /home/ssy/cobot_ws/src/cobot2_ws/pick_and_place_voice/voice_processing/wakeup_word.py

"""
[코드 기능]: openwakeword 라이브러리를 사용하여 실시간 오디오 스트림에서 특정 호출어(Wake-word)를 감지
[입력(Input)]: PyAudio 또는 유사 라이브러리로부터 전달받은 오디오 스트림(Stream)
[출력(Output)]: 호출어 감지 여부 (Boolean: True/False)
"""

import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

# ROS2 패키지 공유 디렉토리에서 학습된 모델 파일(.tflite) 경로 탐색
package_path = get_package_share_directory("pick_and_place_voice")
MODEL_NAME = "hello_rokey_8332_32.tflite"
MODEL_PATH = os.path.join(package_path, f"resource/{MODEL_NAME}")


class WakeupWord:
    def __init__(self, buffer_size):
        """
        [인풋]: buffer_size (Int) - 한 번에 읽어들일 오디오 버퍼의 크기
        [아웃풋]: 없음 (변수 초기화)
        """
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0] # 파일명에서 확장자를 제외한 모델 이름 추출
        self.stream = None # 외부에서 주입될 오디오 스트림 객체
        self.buffer_size = buffer_size

    def is_wakeup(self):
        """
        [인풋]: 없음 (self.stream 객체로부터 데이터를 직접 읽음)
        [아웃풋]: 감지 여부 (Boolean)
        """
        # 스트림으로부터 버퍼 사이즈만큼 데이터를 읽어 16비트 정수형 배열로 변환
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        # 입력된 오디오 샘플링 레이트(예: 48kHz)를 모델 기준인 16kHz로 리샘플링
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        
        # 모델을 사용하여 호출어 포함 확률 계산
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name] # 특정 모델의 신뢰도 값 추출
        print("confidence: ", confidence)
        
        # 계산된 신뢰도가 임계값(0.3)을 넘을 경우 호출어 감지로 판단
        if confidence > 0.3:
            print("Wakeword detected!")
            return True
        return False

    def set_stream(self, stream):
        """
        [인풋]: stream (PyAudio Stream Object) - 실시간 마이크 입력 스트림
        [아웃풋]: 없음 (모델 로드 및 스트림 설정)
        """
        # 지정된 경로의 모델 파일을 로드하여 예측 모델 객체 생성
        self.model = Model(wakeword_models=[MODEL_PATH])
        self.stream = stream