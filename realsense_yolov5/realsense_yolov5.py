import pyrealsense2 as rs
import numpy as np
import cv2
import torch

# -------------------------------------
# 1. RealSense 파이프라인 초기화
# -------------------------------------
pipeline = rs.pipeline()
config = rs.config()

# 색상 스트림 활성화 (해상도: 640x480, FPS: 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 파이프라인 시작
pipeline.start(config)

# -------------------------------------
# 2. YOLOv5 모델 로드 (torch.hub 이용)
# -------------------------------------
# 'yolov5s' 모델 사용 (pretrained)
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# 기본 신뢰도(threshold)를 설정 (필요시 변경)
model.conf = 0.5  # 신뢰도 임계값

# COCO dataset에서 'person' 클래스는 인덱스 0 입니다.
PERSON_CLASS_ID = 0

print("실시간 영상 스트림을 시작합니다. 종료하려면 'q'를 누르세요.")

try:
    while True:
        # -------------------------------------
        # 3. RealSense 카메라로부터 프레임 읽기
        # -------------------------------------
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # 프레임을 numpy 배열로 변환 (BGR 형식)
        img = np.asanyarray(color_frame.get_data())

        # -------------------------------------
        # 4. YOLOv5 모델을 이용한 추론 수행
        # -------------------------------------
        results = model(img)  # 모델에 이미지를 전달

        # 결과에서 감지된 객체들을 가져옵니다.
        # 결과 텐서는 [x1, y1, x2, y2, confidence, class] 형식입니다.
        detections = results.xyxy[0].cpu().numpy()

        # -------------------------------------
        # 5. 사람(person) 클래스만 필터링하여 표시
        # -------------------------------------
        for *box, conf, cls in detections:
            if int(cls) == PERSON_CLASS_ID:
                x1, y1, x2, y2 = map(int, box)
                # 바운딩 박스 그리기 (녹색)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 텍스트 표시: 클래스 이름과 신뢰도
                cv2.putText(img, f"person {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # -------------------------------------
        # 6. 결과 영상 출력
        # -------------------------------------
        cv2.imshow("RealSense YOLOv5 - Person Detection", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print("에러 발생:", e)

finally:
    # 파이프라인 중지 및 창 닫기
    pipeline.stop()
    cv2.destroyAllWindows()
