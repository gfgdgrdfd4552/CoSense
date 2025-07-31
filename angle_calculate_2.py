'''右三车道行驶时计算的每帧的遮挡角度和可视比率'''

import os
import json
import math
import pandas as pd

# 配置
#path = r"D:\evaluation\vehicle_tracking\New\1\vehicle_1898"
import sys  # 添加这一行

# 从命令行参数读取目标文件夹路径
if len(sys.argv) < 2:
    print("请提供子文件夹路径作为参数")
    sys.exit(1)

path = sys.argv[1]  # 替代原来的写死路径
detect_range = 20#探测距离
"""x_min = -13.6#第二车道的左边界
x_max = -12.9#第二车道的有边界"""
x_min = 36.6#第二车道的左边界
x_max = 37.5#第二车道的有边界
length = 4.5
width = 2.0
target_range = [(40, 90)]
target_total_span = sum(end - start for start, end in target_range)

# 自动构建 file_time_list
file_time_list = []
for filename in sorted(os.listdir(path)):
    if filename.endswith(".json"):
        with open(os.path.join(path, filename), 'r') as f:
            data = json.load(f)
            ts = data.get("timestamp", None)
            if ts is not None:
                file_time_list.append([filename, ts])

# 主处理流程
for entry in file_time_list:
    filename = entry[0]
    filepath = os.path.join(path, filename)

    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
    except:
        continue

    # Camera 坐标变换
    cam = data.get("camera", {})
    cam_x = cam.get("y", 0)
    cam_y = cam.get("x", 0)
    cam_y = cam_y + length / 2
    # 车辆坐标变换
    vehicles = data.get("vehicles", [])
    filtered_vehicles = []
    for v in vehicles:
        tx = v["y"]
        ty = v["x"]

        # 满足 y 条件 + 探测范围
        if ty > cam_y and (ty - cam_y) < detect_range:
            # 满足 x 条件
            if x_min <= tx <= x_max:
                filtered_vehicles.append((v["id"], tx, ty))

    # 将筛选后的车辆加入到原 entry 中
    if filtered_vehicles:
        entry.append(filtered_vehicles)
# 遍历每一项，更新车辆坐标信息
for entry in file_time_list:
    if len(entry) < 3:
        continue  # 没有车辆信息，跳过

    new_vehicle_list = []
    for vehicle in entry[2]:
        vid, x, y = vehicle

        # 新定义的角点：
        left_top = (x - width / 2, y + length / 2)
        right_bottom = (x + width / 2, y - length / 2)


        new_vehicle_list.append((vid, x, y, left_top, right_bottom))

    entry[2] = new_vehicle_list

def compute_angle_a(A, B, C):
    """
    计算以 A 为顶点，由 AB 和 AC 构成的夹角 ∠BAC，单位为度。

    参数:
        A, B, C: 三个点坐标，格式为 (x, y)

    返回:
        角度 ∠BAC，单位为度。
    """
    # 向量 AB 和 AC
    ABx, ABy = B[0] - A[0], B[1] - A[1]
    ACx, ACy = C[0] - A[0], C[1] - A[1]

    # 点积和模长
    dot = ABx * ACx + ABy * ACy
    norm_AB = math.hypot(ABx, ABy)
    norm_AC = math.hypot(ACx, ACy)

    if norm_AB == 0 or norm_AC == 0:
        raise ValueError("向量长度为 0，无法计算角度。")

    # 计算角度，限制余弦值在 [-1, 1] 范围内
    cos_theta = max(min(dot / (norm_AB * norm_AC), 1), -1)
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)

    return angle_deg



# 遍历每一帧，追加角度信息
for entry in file_time_list:
    if len(entry) < 3:
        continue

    # 获取摄像机坐标 W 和 E
    filename = entry[0]
    filepath = os.path.join(path, filename)
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
            cam_x = data["camera"]["y"]
            cam_y = data["camera"]["x"]
            W = (cam_x, cam_y)
            E = (cam_x + 1, cam_y)
    except:
        continue

    # 追加角度到每辆车记录
    updated_list = []
    for vehicle in entry[2]:
        vid, x, y, P, Q = vehicle

        angle_P = compute_angle_a(W, P, E)
        angle_Q = compute_angle_a(W, Q, E)

        updated_list.append((vid, x, y, P, Q, angle_P, angle_Q))

    entry[2] = updated_list
def merge_intervals(intervals):
    """
    合并多个角度区间 [(start, end), ...]，返回不重叠的并集区间列表
    """
    if not intervals:
        return []
    intervals.sort()
    merged = [intervals[0]]
    for current in intervals[1:]:
        prev = merged[-1]
        if current[0] <= prev[1]:  # 有重叠
            merged[-1] = (prev[0], max(prev[1], current[1]))
        else:
            merged.append(current)
    return merged
for entry in file_time_list:
    if len(entry) < 3:
        continue

    angle_ranges = []
    for vehicle in entry[2]:
        if len(vehicle) >= 7:
            a1 = vehicle[-2]
            a2 = vehicle[-1]
            b, a = sorted([a1, a2])  # 保证 b < a
            angle_ranges.append((b, a))

    # 合并所有角度区间
    merged_angles = merge_intervals(angle_ranges)

    # 将合并结果添加为该 entry 的第 4 个元素
    entry.append(merged_angles)

def interval_intersection(a, b):
    """
    计算两个区间列表 a 和 b 的交集，返回一个新的不重叠区间列表。
    """
    i, j = 0, 0
    result = []
    while i < len(a) and j < len(b):
        start = max(a[i][0], b[j][0])
        end = min(a[i][1], b[j][1])
        if start < end:
            result.append((start, end))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return result
for entry in file_time_list:
    if len(entry) < 4:
        continue

    angle_ranges = entry[3]

    # 计算交集
    intersect = interval_intersection(angle_ranges, target_range)

    # 求交集角度总长
    total_angle = sum(end - start for start, end in intersect)

    # 添加新元素
    entry.append(intersect)                 # 第五项：交集区间
    entry.append(1 - total_angle / target_total_span)        # 第六项：覆盖率比例
# 输出最终结果
for entry in file_time_list:
    if len(entry) >= 2:
        entry[1] = round(entry[1], 2)

# 按时间排序（升序）
file_time_list.sort(key=lambda x: x[1])

for entry in file_time_list:
    print(entry)


# data = []
# for entry in file_time_list:
#     if len(entry) >= 6:
#         time = round(entry[1], 2)
#         occluded_range = entry[4]  # 不做任何处理，直接写入
#         coverage_ratio = entry[5]
#         data.append([time, occluded_range, coverage_ratio])


data = []
for entry in file_time_list:
    # 时间戳肯定有
    time = round(entry[1], 2)

    if len(entry) >= 6:
        occluded_range  = entry[4]
        coverage_ratio  = entry[5]
    else:
        # 没有计算到任何车辆，就默认不遮挡、可视率 1.0
        occluded_range  = []
        coverage_ratio  = 1.0

    data.append([time, occluded_range, coverage_ratio])


# 创建 DataFrame
df = pd.DataFrame(data, columns=["Time", "Occluded Angle", "Visibility Ratio"])

# 保存路径
save_path = os.path.join(path, "visibility_ratio_log.csv")

# 保存为 CSV 文件
df.to_csv(save_path, index=False)
print(f"📄 CSV 文件已成功保存到：{save_path}")