"""
[코드 기능]: 자비스(Jarvis) 호출어 감지를 위한 .tflite 모델 학습 통합 프로세스 실행
[입력(Input)]: 없음 (스크립트 내부 설정값 및 HuggingFace 온라인 데이터셋 사용)
[출력(Output)]: ./my_custom_model/jarvis.tflite (최종 학습된 모델 파일)
"""

import os
import sys
import yaml
import subprocess
import shutil
from pathlib import Path

# ==========================================
# 1. 설정값 정의 (RTX 4060 최적화)
# ==========================================
TARGET_PHRASES = ["jarvis", "javis", "zarvis", "zavis"]
MODEL_NAME = "jarvis"
STEPS = 25000       # 4060 성능을 고려하여 학습 횟수 상향
N_SAMPLES = 5000     # 합성 음성 데이터 개수
DEVICE = "cuda"      # GPU 사용 설정

# 경로 설정
BASE_DIR = os.getcwd()
MODEL_DIR = os.path.join(BASE_DIR, "my_custom_model")
RESOURCE_DIR = os.path.join(BASE_DIR, "resources")
os.makedirs(RESOURCE_DIR, exist_ok=True)

def run_command(cmd, description):
    print(f"\n[실행 중]: {description}")
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[오류 발생]: {description} 단계에서 문제가 발생했습니다.\n{e}")
        sys.exit(1)

# ==========================================
# 2. 필수 데이터 및 레포지토리 준비
# ==========================================
if not os.path.exists("openwakeword"):
    run_command("git clone https://github.com/dscripka/openwakeword", "openwakeword 레포지토리 클론")

# 기초 모델(Embedding/Spectrogram) 다운로드
MODEL_URLS = [
    "https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/embedding_model.tflite",
    "https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/melspectrogram.tflite"
]
for url in MODEL_URLS:
    target_path = os.path.join("openwakeword/openwakeword/resources/models", os.path.basename(url))
    if not os.path.exists(target_path):
        os.makedirs(os.path.dirname(target_path), exist_ok=True)
        run_command(f"wget {url} -O {target_path}", f"{os.path.basename(url)} 다운로드")

# 학습용 특성 데이터(ACAV100M) 다운로드 (시간이 다소 소요될 수 있음)
if not os.path.exists("openwakeword_features_ACAV100M_2000_hrs_16bit.npy"):
    run_command("wget https://huggingface.co/datasets/davidscripka/openwakeword_features/resolve/main/openwakeword_features_ACAV100M_2000_hrs_16bit.npy", "훈련용 특성 데이터 다운로드")
if not os.path.exists("validation_set_features.npy"):
    run_command("wget https://huggingface.co/datasets/davidscripka/openwakeword_features/resolve/main/validation_set_features.npy", "검증용 데이터 다운로드")

# ==========================================
# 3. YAML 설정 파일 생성
# ==========================================
config = {
    "target_phrase": TARGET_PHRASES,
    "model_name": MODEL_NAME,
    "n_samples": N_SAMPLES,
    "n_samples_val": 1000,
    "steps": STEPS,
    "target_accuracy": 0.7,
    "target_recall": 0.5,
    "device": DEVICE,
    "background_paths": [], # 비어있을 경우 기본 데이터셋 사용 시도
    "false_positive_validation_data_path": "validation_set_features.npy",
    "feature_data_files": {"ACAV100M": "openwakeword_features_ACAV100M_2000_hrs_16bit.npy"}
}

config_path = "jarvis_config.yaml"
with open(config_path, 'w') as f:
    yaml.dump(config, f)
print(f"\n[알림]: {config_path} 생성 완료.")

# ==========================================
# 4. 학습 단계별 실행 (openwakeword.train 스크립트 호출)
# ==========================================
TRAIN_SCRIPT = "openwakeword/openwakeword/train.py"

# Step 1: 합성 데이터 생성
run_command(f"python3 {TRAIN_SCRIPT} --training_config {config_path} --generate_clips", "음성 합성 데이터 생성")

# Step 2: 데이터 증강 (노이즈 추가 등)
run_command(f"python3 {TRAIN_SCRIPT} --training_config {config_path} --augment_clips", "데이터 증강")

# Step 3: 모델 학습 시작
run_command(f"python3 {TRAIN_SCRIPT} --training_config {config_path} --train_model", "신경망 모델 학습")

# ==========================================
# 5. TFLite 변환
# ==========================================
def convert_to_tflite():
    print("\n[진행 중]: ONNX 모델을 TFLite로 변환합니다.")
    import onnx
    from onnx_tf.backend import prepare
    import tensorflow as tf
    import tempfile

    onnx_path = os.path.join(MODEL_DIR, f"{MODEL_NAME}.onnx")
    tflite_path = os.path.join(MODEL_DIR, f"{MODEL_NAME}.tflite")

    if not os.path.exists(onnx_path):
        print(f"[오류]: {onnx_path} 파일을 찾을 수 없습니다.")
        return

    onnx_model = onnx.load(onnx_path)
    tf_rep = prepare(onnx_model, device="CPU")
    
    with tempfile.TemporaryDirectory() as tmp_dir:
        tf_rep.export_graph(os.path.join(tmp_dir, "tf_model"))
        converter = tf.lite.TFLiteConverter.from_saved_model(os.path.join(tmp_dir, "tf_model"))
        tflite_model = converter.convert()
        with open(tflite_path, 'wb') as f:
            f.write(tflite_model)
    
    print(f"\n[완료]: 최종 모델이 생성되었습니다 -> {tflite_path}")

convert_to_tflite()