"""生成违停车辆的id和全局坐标，保存在illegal_vehicles_global_avg_coords中，同时保存了reference(vehicle_id+time+coord).csv"""
import os
import cv2
import json
import csv
import numpy as np
from collections import defaultdict
from sklearn.cluster import DBSCAN

def compute_label_avg_coords(coords, labels):
    label_coords = defaultdict(list)
    for coord, label in zip(coords, labels):
        label_coords[label].append(coord)
    avg_dict = {}
    for label, points in label_coords.items():
        points_array = np.array(points)
        avg_coord = np.mean(points_array, axis=0)
        avg_dict[label] = tuple(float(x) for x in avg_coord)
    return avg_dict

def judge_labels_dispersion(data, iqr_threshold):
    label_points = {}
    for entry in data:
        label = int(entry[2][-2])
        #print(label)
        y = entry[-1][0]
        label_points.setdefault(label, []).append(y)
    result = {}
    for label, y_vals in label_points.items():
        if len(y_vals) <= 0:             ##################################
            continue
        y_vals = np.array(y_vals)
        q1 = np.percentile(y_vals, 25)
        q3 = np.percentile(y_vals, 75)
        iqr = q3 - q1
        result[label] = "0" if iqr > iqr_threshold else "1"
    return result





def dbscan_dominant_cluster_center(coords, eps=3.0, min_samples=2):
    """
    对 (X, Y) 坐标点运行 DBSCAN，并返回最大簇的中心坐标。
    若聚类失败，则回退到均值。
    """
    coords = np.array(coords)
    if len(coords) < min_samples:
        return np.mean(coords[:, 0]), np.mean(coords[:, 1])  # fallback

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coords)
    labels = clustering.labels_

    if len(set(labels)) <= 1 or all(labels == -1):
        # 全部是噪声 或 只有一个类
        return np.mean(coords[:, 0]), np.mean(coords[:, 1])

    # 找出最大簇
    unique, counts = np.unique(labels[labels != -1], return_counts=True)
    dominant_label = unique[np.argmax(counts)]
    dominant_cluster = coords[labels == dominant_label]

    # 返回该簇的中心点
    avg_x = np.mean(dominant_cluster[:, 0])
    avg_y = np.mean(dominant_cluster[:, 1])
    return avg_x, avg_y


def save_clustered_labels_avg_coords(data, dispersion_result, video_dir):
    """
    计算并保存：
      - 每个违停车辆的平均全局坐标 (Avg_X, Avg_Y)
      - 每个违停车辆的平均检测置信度 (Avg_Confidence)
    """
    csv_path = os.path.join(video_dir, "illegal_vehicles_global_avg_coords.csv")
    label_coords = {}
    label_confs  = {}

    for i, entry in enumerate(data):
        if len(entry) < 6:
            print(f"[警告] 第{i}个entry长度为{len(entry)}，应为≥6，内容为：{entry}")
            continue

        # entry[2] 结构： [x1, y1, x2, y2, track_id, confidence]
        label = int(entry[2][-2])       # track_id
        conf  = float(entry[2][-1])     # confidence
        coord = entry[5]                # 全局坐标 (X, Y)
        label_coords.setdefault(label, []).append(coord)
        label_confs .setdefault(label, []).append(conf)

    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['illegal parking Vehicle ID', 'Avg_X', 'Avg_Y', 'Avg_Confidence'])
        for label, status in dispersion_result.items():
            if status == "1":
                coords   = np.array(label_coords[label])
                avg_x    = np.mean(coords[:, 0])
                avg_y    = np.mean(coords[:, 1])
#                avg_x, avg_y = dbscan_dominant_cluster_center(coords)
                avg_conf = float(np.mean(label_confs[label]))
                writer.writerow([label, avg_x, avg_y, avg_conf])

    print(f"违停车辆的平均坐标及置信度已保存至: {csv_path}")


def process_video_folder(video_dir):
    print(f"\n正在处理文件夹: {video_dir}")

    illegal_parking_threshold = 5
    c_x = 320
    f_x = 268.51188197672957
    f_y = 268.51188197672957
    c_y = 240

    frame_info_txt = os.path.join(video_dir, "frame_count.txt")
    csv_path = os.path.join(video_dir, "detection_output.csv")

    # 读取 all_frame
    try:
        with open(frame_info_txt, 'r') as f:
            all_frame = int(f.readline().strip())
            print(f"{os.path.basename(frame_info_txt)} 中的 all_frame 值为: {all_frame}")
    except Exception as e:
        print(f"读取 {frame_info_txt} 出错: {e}")
        return

    # 检查 detection_output.csv 是否有内容，不覆盖 all_frame
    try:
        with open(csv_path, 'r', newline='') as f:
            reader = csv.reader(f)
            rows = list(reader)
            if len(rows) <= 1:
                print(f"{csv_path} 文件为空或仅有表头，跳过")
                return
    except Exception as e:
        print(f"读取 {csv_path} 出错: {e}")
        return

    # 获取视频时长
    video_duration = None
    for file_name in os.listdir(video_dir):
        if file_name.lower().endswith(".mp4"):
            video_path = os.path.join(video_dir, file_name)
            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                print(f"无法打开视频文件: {file_name}")
                continue
            fps = cap.get(cv2.CAP_PROP_FPS)
            frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            video_duration = frame_count / fps if fps > 0 else 0
            print(f"选定视频: {file_name}，时长: {video_duration:.2f} 秒")
            cap.release()
            break

    if video_duration is None or video_duration <= 0:
        print("未找到有效视频文件或时长无效，跳过此文件夹")
        return

    json_files = [f for f in os.listdir(video_dir) if f.lower().endswith(".json")]
    all_results = []
    for json_file in json_files:
        json_path = os.path.join(video_dir, json_file)
        with open(json_path, 'r') as f:
            data = json.load(f)

        file_result = {
            'json_file': json_file,
            'frame_indices': []
        }

        if isinstance(data, list):
            for item in data:
                if 'timestamp' in item:
                    ts = item['timestamp']
                    ratio = ts / video_duration
                    frame_index = int(ratio * all_frame)
                    file_result['frame_indices'].append(frame_index)
        elif isinstance(data, dict) and 'timestamp' in data:
            ts = data['timestamp']
            ratio = ts / video_duration
            frame_index = int(ratio * all_frame)
            file_result['frame_indices'].append(frame_index)
        else:
            print(f"{json_file} 无效结构或缺少 timestamp 字段")
        all_results.append(file_result)
    final_results = []
    for item in all_results:
        json_name = item['json_file']
        frame_indices = item['frame_indices']
        final_results.append([json_name, frame_indices])
    print(final_results)
    frame_to_data = defaultdict(list)
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or not row[0].isdigit():
                continue
            frame_index = int(row[0])
#            frame_to_data[frame_index].append(row[1:6])
            frame_to_data[frame_index].append(row[1:7])

    final_results_expanded = []
    for item in final_results:
        json_file = item[0]
        frame_indices = item[1]
        for frame_idx in frame_indices:
            if frame_idx in frame_to_data:
                for csv_values in frame_to_data[frame_idx]:
                    final_results_expanded.append([json_file, frame_idx, csv_values])

    for item in final_results_expanded:
        if len(item) < 3 or not isinstance(item[2], list) or len(item[2]) < 4:
            continue
        x1, y1, x2, y2 = map(int, item[2][:4])
        cx = int((x1 + x2) / 2)
        cy = int(y2)
        center_point = (cx, cy)
        item.append(center_point)

    for item in final_results_expanded:
        if len(item) < 4 or not isinstance(item[-1], tuple):
            continue
        json_file = item[0]
        json_path = os.path.join(video_dir, json_file)
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                if 'camera' in data and isinstance(data['camera'], dict) and 'z' in data['camera']:
                    h = float(data['camera']['z'])
                else:
                    print(f"[跳过] {json_file} 中 camera 字段缺失或结构异常")
                    continue
        except Exception as e:
            print(f"[错误] 打开 {json_file} 失败：{e}")
            continue

        u, v = item[-1]
        if v == c_y:
            print(f"[跳过] v == c_y，除0风险")
            continue

        denominator = (v - c_y)
        X = (h * f_y) * (u - c_x) / (f_x * denominator)
        Z = (h * f_y) * f_x / (f_x * denominator)
        item.append((X, Z))
        #print(item)
    for item in final_results_expanded:
        if len(item) < 2 or not isinstance(item[-1], (list, tuple)):
            continue
        json_file = item[0]
        json_path = os.path.join(video_dir, json_file)
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            print(f"[错误] 无法打开 {json_file}：{e}")
            continue

        cam = data.get('camera', {})
        if not isinstance(cam, dict) or 'x' not in cam or 'y' not in cam:
            print(f"[跳过] {json_file} 中缺少 camera.x 或 camera.y")
            continue

        try:
            cx_cam = float(cam['x'])
            cy_cam = float(cam['y'])
        except ValueError:
            print(f"[跳过] {json_file} 中 camera.x/y 不是数值")
            continue
        # offset_x = -cy_cam
        # offset_y = -cx_cam
        #
        # orig_x, orig_y = item[-1]
        # new_coord = (orig_x + offset_x, orig_y + offset_y)
        # item.append(new_coord)
        #  替换原来的 offset_x, offset_y 写法
        orig_x, orig_z = item[-1]

        x_world = orig_x + cx_cam  # Z (前向) → 加到 x
        y_world = orig_z + cy_cam  # X (右侧) → 加到 y

        new_coord = (x_world, y_world)
        item.append(new_coord)

    # 打印每条记录，方便调试
    for row in final_results_expanded:
        print(row)

    coords = []
    labels = []
    for item in final_results_expanded:
        coord = item[-1]
        label = item[2][-2]
        coords.append(coord)
        labels.append(label)

    coords = np.array(coords)
    labels = np.array(labels)

    avg_coords = compute_label_avg_coords(coords, labels)
    """for label, avg in avg_coords.items():
        print(f"标签 {label}: 平均坐标 {avg}")"""
    """print(final_results_expanded)"""
    print("-----------------------")
    dispersion_result = judge_labels_dispersion(final_results_expanded, iqr_threshold=illegal_parking_threshold)
    #print(dispersion_result)

    save_clustered_labels_avg_coords(final_results_expanded, dispersion_result, video_dir)
    output_csv_path = os.path.join(video_dir, "reference(vehicle_id+time+coord).csv")

    with open(output_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Vehicle ID', 'Timestamp', 'Global Coordinate (X, Y)'])  # 表头

        for item in final_results_expanded:
            if len(item) < 6 or not isinstance(item[5], (list, tuple)):
                continue

            vehicle_id = item[2][-2]
            global_coord = item[5]

            # 获取 timestamp
            json_file = item[0]
            json_path = os.path.join(video_dir, json_file)
            try:
                with open(json_path, 'r') as f:
                    data = json.load(f)
                timestamp = data.get("timestamp", None)
                if timestamp is None:
                    continue
                timestamp = round(float(timestamp), 2)
            except Exception as e:
                print(f"[跳过] 无法读取 {json_file} 中的 timestamp: {e}")
                continue

            writer.writerow([vehicle_id, timestamp, global_coord])

    #print(f"reference(vehicle_id+time+coord).csv 已保存至: {output_csv_path}")

if __name__ == "__main__":
    root_dir = (r"D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2")
    subdirs = [os.path.join(root_dir, d) for d in os.listdir(root_dir)
               if os.path.isdir(os.path.join(root_dir, d))]

    for video_dir in subdirs:
        process_video_folder(video_dir)



    print("\n所有子文件夹处理完毕。")
