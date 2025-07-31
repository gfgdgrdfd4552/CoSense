import cv2
import os
import time
import torch
import numpy as np
from sort import Sort
from tqdm import tqdm
import warnings
import csv  # 放在文件开头

warnings.filterwarnings("ignore", category=FutureWarning,
                        message=".*torch.cuda.amp.autocast.*")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RIGHT_RATIO = 0.5
DETECTION_LINE_COLOR = (0, 0, 255)
DETECTION_LINE_THICKNESS = 2

def load_model_with_retry(retries=3, delay=5):
    model_path = os.path.join(BASE_DIR, 'yolov5s.pt')
    for attempt in range(retries):
        try:
            model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
            return model
        except Exception as e:
            print(f"模型加载失败，尝试 {attempt + 1}/{retries}: {e}")
            if attempt < retries - 1:
                time.sleep(delay)
            else:
                raise

def detect_right_vehicles(frame, model, right_ratio):
    height, width = frame.shape[:2]
    right_boundary = int(width * (1 - right_ratio))
    results = model(frame)
    detections = []
    for *box, conf, cls in results.xyxy[0]:
        if conf > 0.5:
            x1, y1, x2, y2 = map(int, box)
            if (x1 + x2) / 2 > right_boundary:
                detections.append([x1, y1, x2, y2, conf.item()])
    return detections, right_boundary

def get_confidence_for_track(track_box, detections):
    x1, y1, x2, y2 = track_box
    for det in detections:
        dx1, dy1, dx2, dy2, det_conf = det
        cx, cy = (dx1+dx2)/2, (dy1+dy2)/2
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            return det_conf
    return 0.0  # 如果实在找不到，就返回 0

def process_video(input_path, output_path, model):
    # 1) 初始化 CSV 并写入表头
    output_csv_path = os.path.join(os.path.dirname(input_path), 'detection_output.csv')

    with open(output_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['frame', 'x1', 'y1', 'x2', 'y2', 'track_id', 'confidence'])

    cap = cv2.VideoCapture(input_path)
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_info_txt = os.path.join(os.path.dirname(input_path), 'frame_count.txt')
    with open(frame_info_txt, 'w') as f:
        f.write(f"{total_frames}\n")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    tracker = Sort(max_age=5, min_hits=3, iou_threshold=0.3)
    all_ids = set()


    with tqdm(total=total_frames, desc=f"处理 {os.path.basename(input_path)}") as pbar:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            detections, right_boundary = detect_right_vehicles(frame, model, RIGHT_RATIO)

            # 画检测区域
            # cv2.line(frame, (right_boundary, 0), (right_boundary, height),
            #          DETECTION_LINE_COLOR, DETECTION_LINE_THICKNESS)
            # cv2.putText(frame, "Detection Area", (right_boundary + 10, 30),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, DETECTION_LINE_COLOR, 2)

            dets_np = np.array(detections) if detections else np.empty((0, 5))
            tracks = tracker.update(dets_np)

            frame_index = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            for t in tracks:
                x1, y1, x2, y2, tid = t
                x1, y1, x2, y2, tid = map(int, (x1, y1, x2, y2, tid))
                all_ids.add(tid)

                # 2) 找到这个 track 对应的检测置信度
                conf = get_confidence_for_track((x1, y1, x2, y2), detections)

                # 3) 写入 CSV
                with open(output_csv_path, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([frame_index, x1, y1, x2, y2, tid, conf])

                # 4) 画框和 ID
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'ID:{tid}', (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            out.write(frame)
            pbar.update(1)

    cap.release()
    out.release()
    print(f"\n视频 {os.path.basename(input_path)} 中出现的车辆 ID：{sorted(all_ids)}")







def main():
    LAB3_DIR = os.path.join(BASE_DIR, r'D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2')
    subdirs = [os.path.join(LAB3_DIR, d) for d in os.listdir(LAB3_DIR)
               if os.path.isdir(os.path.join(LAB3_DIR, d))]

    if not subdirs:
        print(f"Lab3 目录下没有任何子文件夹: {LAB3_DIR}")
        return

    print(f"找到 {len(subdirs)} 个子文件夹，开始遍历...")
    model = load_model_with_retry()

    for folder in subdirs:
        video_files = [f for f in os.listdir(folder)
                       if f.lower().endswith('.mp4') and f.lower().startswith("rgb")]
        if not video_files:
            print(f"⏭️ 子文件夹中没有 rgb 开头的视频: {folder}")
            continue

        print(f"\n📁 当前目录: {folder}，RGB 视频数量: {len(video_files)}")
        for video_file in video_files:
            input_path  = os.path.join(folder, video_file)
            output_path = os.path.join(folder, f"right_tracked_{video_file}")
            print(f"🎬 正在处理: {video_file}")
            start_time = time.time()
            try:
                process_video(input_path, output_path, model)
                elapsed = time.time() - start_time
                print(f"✔️ 完成: {output_path}（耗时: {elapsed:.2f}秒）")
            except Exception as e:
                print(f"❌ 处理失败: {video_file} - 错误: {e}")

if __name__ == '__main__':
    main()
