import warnings
import logging

# FutureWarning(미래 경고) 및 기타 불필요한 로그 억제
warnings.filterwarnings("ignore", category=FutureWarning)
logging.getLogger().setLevel(logging.ERROR)

import pyrealsense2 as rs
import numpy as np
import cv2
import torch

# -------------------------------------
# 1. RealSense 파이프라인 및 스트림 설정
# -------------------------------------
pipeline = rs.pipeline()
config = rs.config()

# 색상 스트림과 depth 스트림 활성화 (해상도: 640x480, 30 FPS)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 파이프라인 시작
pipeline.start(config)

# 색상 스트림에 맞춰 depth 스트림 정렬(align) 설정
align_to = rs.stream.color
align = rs.align(align_to)

# -------------------------------------
# 2. YOLOv5 모델 로드 (torch.hub 이용)
# -------------------------------------
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.conf = 0.5  # 신뢰도 임계값 설정
PERSON_CLASS_ID = 0  # COCO 데이터셋에서 사람(person) 클래스 인덱스

# 카메라 파라미터 (해상도, 수평 FOV)
img_width = 640
horizontal_fov = 69.4  # 단위: degree (필요에 따라 수정)

print("실시간 스트리밍 시작 (종료하려면 'q' 키를 누르세요).")

try:
    while True:
        # 프레임 획득 및 정렬
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # 색상 프레임을 numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())

        # -------------------------------------
        # 3. YOLOv5를 이용한 객체 감지 (색상 이미지)
        # -------------------------------------
        results = model(color_image)
        detections = results.xyxy[0].cpu().numpy()

        # 각 검출에 대해 처리
        for *box, conf, cls in detections:
            if int(cls) == PERSON_CLASS_ID:
                # 바운딩 박스 좌표 정수형 변환
                x1, y1, x2, y2 = map(int, box)
                # 바운딩 박스 중심 좌표 계산
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                # 해당 중심 좌표의 depth (미터 단위) 읽기
                depth_value = depth_frame.get_distance(center_x, center_y)
                # 수평 각도 계산:
                # 이미지 중앙(픽셀 img_width/2) 대비 중심 좌표의 오프셋에 수평 FOV를 적용
                angle_deg = (center_x - (img_width / 2)) * (horizontal_fov / img_width)

                # 터미널에 정보 출력 (불필요한 경고 메시지는 억제됨)
                print(f"Person detected: Center=({center_x}, {center_y}), Depth={depth_value:.2f}m, Angle={angle_deg:.1f}°")

                # (선택사항) 바운딩 박스 그리기
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # -------------------------------------
        # 4. 결과 영상 출력
        # -------------------------------------
        cv2.imshow("RealSense YOLOv5 - Person Detection", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print("에러 발생:", e)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

