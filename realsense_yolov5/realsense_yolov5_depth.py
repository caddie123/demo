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
# Depth 스트림 활성화 (해상도: 640x480, FPS: 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 파이프라인 시작
pipeline.start(config)

# -------------------------------------
# 2. YOLOv5 모델 로드 (torch.hub 이용)
# -------------------------------------
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
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
        
        # 색상 프레임 및 depth 프레임 가져오기
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # 색상 프레임을 numpy 배열로 변환 (BGR 형식)
        color_image = np.asanyarray(color_frame.get_data())
        
        # Depth 프레임을 numpy 배열로 변환
        depth_image = np.asanyarray(depth_frame.get_data())
        # Depth 이미지를 8비트로 변환 후, 컬러맵 적용
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # -------------------------------------
        # 4. YOLOv5 모델을 이용한 추론 수행 (색상 이미지에 대해)
        # -------------------------------------
        results = model(color_image)
        detections = results.xyxy[0].cpu().numpy()

        # -------------------------------------
        # 5. 사람(person) 클래스만 필터링하여 표시 (색상 이미지 위에)
        # -------------------------------------
        for *box, conf, cls in detections:
            if int(cls) == PERSON_CLASS_ID:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f"person {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # -------------------------------------
        # 6. 결과 영상 출력 (색상 이미지와 depth 데이터)
        # -------------------------------------
        cv2.imshow("RealSense YOLOv5 - Person Detection", color_image)
        cv2.imshow("Depth Data", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print("에러 발생:", e)

finally:
    # 파이프라인 중지 및 창 닫기
    pipeline.stop()
    cv2.destroyAllWindows()
