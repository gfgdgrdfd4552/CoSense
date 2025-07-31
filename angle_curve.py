import os
import json
import math
import pandas as pd

# ====================== 配置参数 ======================

root_path = r"D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2"  # 顶层文件夹路径
#root_path = r"D:\evaluation\vehicle_tracking\New\123"  # 顶层文件夹路径

DETECT_RADIUS = 20
DETECT_ANGLE_MIN = 0
DETECT_ANGLE_MAX = 50
#选择屏蔽的车辆编号（把违停车辆屏蔽掉）
#EXCLUDE_IDS = set(range(3837, 3857))#left
EXCLUDE_IDS = set(range(3989, 4009))#right
#EXCLUDE_IDS = set(range(3217, 3237))#123

VEHICLE_LENGTH = 4.5
VEHICLE_WIDTH = 2.0

# ====================== 工具函数 ======================

def angle_between(p):
    angle = math.degrees(math.atan2(p[1], p[0]))
    return angle  # 保留原始角度，不进行 %360

def point_in_sector(p):
    dist = math.hypot(p[0], p[1])
    if dist > DETECT_RADIUS:
        return False
    angle = angle_between(p)
    return DETECT_ANGLE_MIN <= angle <= DETECT_ANGLE_MAX

def merge_intervals(intervals):
    if not intervals:
        return []
    intervals.sort()
    merged = [intervals[0]]
    for current in intervals[1:]:
        prev = merged[-1]
        if current[0] <= prev[1]:
            merged[-1] = (prev[0], max(prev[1], current[1]))
        else:
            merged.append(current)
    return merged

def compute_visible_ratio(merged_angles, angle_min=0, angle_max=50):
    sector_span = angle_max - angle_min
    if sector_span <= 0:
        return 0.0
    blocked_total = 0.0
    for a, b in merged_angles:
        a_clamped = max(a, angle_min)
        b_clamped = min(b, angle_max)
        if a_clamped < b_clamped:
            blocked_total += (b_clamped - a_clamped)
    visible_ratio = max(0.0, 1.0 - blocked_total / sector_span)
    return visible_ratio

# ====================== 主处理函数 ======================

def process_single_folder(folder_path):
    print(f"\n📂 正在处理文件夹: {folder_path}")
    results = []
    for filename in sorted(os.listdir(folder_path)):
        if not filename.endswith(".json"):
            continue

        filepath = os.path.join(folder_path, filename)
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
        except Exception as e:
            print(f"❌ 无法读取 {filename}: {e}")
            continue

        cam = data.get("camera", {})
        cam_x = cam.get("x", 0)
        cam_y = cam.get("y", 0)
        yaw_deg = cam.get("yaw", 0)
        yaw_rad = math.radians(yaw_deg)
        timestamp = round(data.get("timestamp", 0), 2)

        cos_theta = math.cos(yaw_rad)
        sin_theta = math.sin(yaw_rad)

        frame_result = {
            "file": filename,
            "timestamp": timestamp,
            "vehicles": []
        }

        for v in data.get("vehicles", []):
            vx = v.get("x", 0)
            vy = v.get("y", 0)
            dx = vx - cam_x
            dy = vy - cam_y

            x_local = cos_theta * dx + sin_theta * dy
            y_local = -sin_theta * dx + cos_theta * dy

            frame_result["vehicles"].append({
                "id": v.get("id"),
                "x_local": round(x_local, 3),
                "y_local": round(y_local, 3)
            })

        results.append(frame_result)

    # 遍历每帧，分析遮挡角度
    file_time_list = []
    for frame in results:
        detected_angles = []
        for v in frame["vehicles"]:
            vid = v["id"]
            if vid in EXCLUDE_IDS:
                continue

            x = v["x_local"]
            y = v["y_local"]

            if not point_in_sector((x, y)):
                continue

            left_top = (x + VEHICLE_LENGTH / 2, y - VEHICLE_WIDTH / 2)
            right_bottom = (x - VEHICLE_LENGTH / 2, y + VEHICLE_WIDTH / 2)
            a1 = angle_between(left_top)
            a2 = angle_between(right_bottom)
            start, end = sorted([a1, a2])
            detected_angles.append((start, end))

        merged_angles = merge_intervals(detected_angles)
        visible_ratio = compute_visible_ratio(merged_angles, DETECT_ANGLE_MIN, DETECT_ANGLE_MAX)

        print(f"  📷 Frame {frame['file']}: Visible Ratio = {round(visible_ratio, 3)}")
        file_time_list.append([frame["file"], frame["timestamp"], merged_angles, visible_ratio])

    file_time_list.sort(key=lambda x: x[1])

    # 保存 CSV
    df = pd.DataFrame(
        [[round(e[1], 2), e[2], e[3]] for e in file_time_list],
        columns=["Time", "Occluded Angle", "Visibility Ratio"]
    )
    save_path = os.path.join(folder_path, "visibility_ratio_log.csv")
    df.to_csv(save_path, index=False)
    print(f"  ✅ 可视比率已保存至: {save_path}")

# ====================== 主执行逻辑 ======================

for subdir in sorted(os.listdir(root_path)):
    full_subdir = os.path.join(root_path, subdir)
    if os.path.isdir(full_subdir):
        process_single_folder(full_subdir)
