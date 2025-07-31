"""
生成两个矩阵：
1. 违停车辆矩阵（二进制，表示车辆是否存在）
2. 检测置信度矩阵（车辆检测的置信度分数）
"""

import os
import csv
import math
import random
from typing import Dict, List, Tuple

# =============== 配置参数 ===============
# 根目录路径
ROOT_DIR = r"D:\vehicle_tracking_new\vehicle_tracking\newnew\heiyeW2"
# 匹配车辆与参考点的最大距离阈值
MAX_DISTANCE = 10
# 备选ID范围（用于无法匹配参考点的车辆）
#EXTRA_IDS = [1000, 1001, 1002, 1003, 1004]
# 生成从 1000 到 1050（含）的 ID 列表
EXTRA_IDS = list(range(1000, 1081))

# 参考点数据（格式：ID,X,Y）
#拐弯时的违停车辆
REFERENCE_POINTS_DATA = """
923,-17.8793,214.1732
924,-17.8699,223.7849
925,-18.2408,232.8399
926,-19.0663,241.8647
927,-20.3443,250.8367
928,-22.0717,259.7331
929,-24.2441,268.5314
930,-26.8559,277.2094
931,-29.9005,285.7451
932,-33.3704,294.1171
933,-37.2567,302.3041
934,-41.5495,310.2854
935,-46.2380,318.0409
936,-51.3104,325.5509
937,-56.7539,332.7964
938,-62.5547,339.7592
939,-68.6981,346.4217
940,-75.1686,352.7669
941,-81.9498,358.7789
942,-89.0246,364.4426
"""
# REFERENCE_POINTS_DATA = """
# 463,-280.0000,40.0000
# 464,-273.0000,40.0000
# 465,-266.0000,40.0000
# 467,-259.0000,40.0000
# 468,-252.0000,40.0000
# 469,-245.0000,40.0000
# 471,-238.0000,40.0000
# 472,-231.0000,40.0000
# 473,-224.0000,40.0000
# 474,-217.0000,40.0000
# 476,-210.0000,40.0000
# 477,-203.0000,40.0000
# 478,-196.0000,40.0000
# 479,-192.0000,40.0000
# 481,-189.0000,40.0000
# 482,-182.0000,40.0000
# 483,-175.0000,40.0000
# 484,-168.0000,40.0000
# 485,-161.0000,40.0000
# 486,-154.0000,40.0000
# 487,-147.0000,40.0000
# """

# =============== 数据读取与处理函数 ===============

def parse_reference_points(data: str) -> Dict[int, Tuple[float, float]]:
    """
    解析参考点数据字符串
    返回字典：{车辆ID: (x坐标, y坐标)}
    """
    reference_points = {}
    for line in data.strip().split('\n'):
        parts = line.strip().split(',')
        if len(parts) < 3:
            continue
        try:
            vid = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            reference_points[vid] = (x, y)
        except ValueError:
            continue
    return reference_points


def read_vehicle_coords(csv_path: str) -> Dict[int, Tuple[Tuple[float, float], float]]:
    """
    从CSV文件读取车辆坐标和置信度
    返回字典：{车辆ID: ((x坐标, y坐标), 置信度)}
    """
    vehicle_coords = {}
    if not os.path.exists(csv_path):
        return vehicle_coords

    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        next(reader, None)  # 跳过表头
        for row in reader:
            if len(row) < 4:
                continue
            try:
                vid = int(row[0])
                x = float(row[1])
                y = float(row[2])
                conf = float(row[3])
                vehicle_coords[vid] = ((x, y), conf)
            except ValueError:
                continue
    return vehicle_coords


def calculate_distances(vehicle_coords: Dict[int, Tuple[Tuple[float, float], float]],
                        reference_points: Dict[int, Tuple[float, float]]) -> Tuple[List[List[float]], List[float]]:
    """
    计算每辆车到所有参考点的距离
    返回:
        distances: 每辆车到所有参考点的距离列表
        confidences: 每辆车的置信度列表
    """
    distances = []
    confidences = []
    for vid, (coord, conf) in vehicle_coords.items():
        vx, vy = coord
        # 计算到每个参考点的距离
        dists = [math.hypot(vx - rx, vy - ry) for rx, ry in reference_points.values()]
        distances.append(dists)
        confidences.append(conf)
    return distances, confidences


def match_vehicles_to_reference(distances_all: List[List[List[float]]],
                                reference_ids: List[int]) -> List[Tuple[int, int]]:
    """
    将检测到的车辆匹配到参考点ID
    返回匹配结果列表：(文件夹索引, 匹配的参考点ID)
    """
    result_list = []
    extra_id_counter = 0

    for folder_idx, dist_list in enumerate(distances_all):
        used_ids = set()  # 已使用的参考点ID

        for dists in dist_list:
            if not dists:
                continue

            # 按距离排序参考点索引
            sorted_indices = sorted(range(len(dists)), key=lambda k: dists[k])
            assigned_id = None

            # 优先匹配距离小于阈值的参考点
            for idx in sorted_indices:
                if dists[idx] < MAX_DISTANCE:
                    candidate_id = reference_ids[idx]
                    if candidate_id not in used_ids:
                        assigned_id = candidate_id
                        used_ids.add(candidate_id)
                        break

            # 如果未找到匹配，使用备选策略
            if assigned_id is None:
                # 尝试匹配距离小于50的最近参考点
                if any(d < 50 for d in dists):
                    assigned_id = reference_ids[sorted_indices[0]]
                else:
                    # 使用备选ID
                    available_ids = [eid for eid in EXTRA_IDS if eid not in used_ids]
                    if available_ids:
                        assigned_id = random.choice(available_ids)
                        used_ids.add(assigned_id)
                    else:
                        # 生成新的唯一ID
                        assigned_id = max(EXTRA_IDS) + extra_id_counter + 1
                        extra_id_counter += 1
                        used_ids.add(assigned_id)

            result_list.append((folder_idx, assigned_id))

    return result_list


def get_visibility_ratio(folder_path: str, match_value: str) -> float:
    """
    从visibility_ratio_log.csv文件中获取匹配的可见性比率
    """
    csv_path = os.path.join(folder_path, "visibility_ratio_log.csv")
    if not os.path.exists(csv_path):
        return None

    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 3 and row[0].strip() == match_value:
                try:
                    return float(row[2])
                except ValueError:
                    continue
    return None


def calculate_folder_visibility_avg(folder_path: str) -> float:
    """
    计算文件夹中所有可见性比率的平均值
    """
    csv_path = os.path.join(folder_path, "visibility_ratio_log.csv")
    if not os.path.exists(csv_path):
        return 0.0

    values = []
    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 3:
                try:
                    value = float(row[2])
                    if not math.isnan(value):
                        values.append(value)
                except ValueError:
                    continue

    return sum(values) / len(values) if values else 0.0


# =============== 矩阵生成函数 ===============

def create_matrix(rows: int, cols: int, default_value=0) -> List[List]:
    """创建指定大小的矩阵"""
    return [[default_value] * cols for _ in range(rows)]


def save_matrix_to_csv(matrix: List[List], header: List[str], row_names: List[str], output_path: str):
    """将矩阵保存为CSV文件"""
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i, row in enumerate(matrix):
            writer.writerow([row_names[i]] + row)


# =============== 主处理流程 ===============

def main():
    # 1. 准备参考点数据
    reference_points = parse_reference_points(REFERENCE_POINTS_DATA)
    reference_ids = sorted(reference_points.keys())
    print(f"已解析 {len(reference_points)} 个参考点")

    # 2. 获取所有子目录
    subdirs = [os.path.join(ROOT_DIR, d)
               for d in os.listdir(ROOT_DIR)
               if os.path.isdir(os.path.join(ROOT_DIR, d))]
    folder_names = [os.path.basename(d) for d in subdirs]
    print(f"找到 {len(subdirs)} 个子目录")

    # 3. 处理每个子目录的数据
    distances_all = []  # 每个文件夹的距离数据
    confidences_all = []  # 每个文件夹的置信度数据
    vehicle_ids_all = []  # 每个文件夹的车辆ID列表
    valid_flags = []  # 标记每个文件夹是否有效

    for subdir in subdirs:
        csv_path = os.path.join(subdir, "illegal_vehicles_global_avg_coords.csv")

        # 检查文件是否存在
        if not os.path.exists(csv_path):
            print(f"警告: {csv_path} 不存在，跳过")
            distances_all.append([])
            confidences_all.append([])
            vehicle_ids_all.append([])
            valid_flags.append(False)
            continue

        # 读取车辆坐标和置信度
        vehicle_coords = read_vehicle_coords(csv_path)
        if not vehicle_coords:
            print(f"警告: {csv_path} 无有效数据，跳过")
            distances_all.append([])
            confidences_all.append([])
            vehicle_ids_all.append([])
            valid_flags.append(False)
            continue

        # 计算距离
        distances, confidences = calculate_distances(vehicle_coords, reference_points)
        distances_all.append(distances)
        confidences_all.append(confidences)
        vehicle_ids_all.append(list(vehicle_coords.keys()))
        valid_flags.append(True)

    # 4. 匹配车辆到参考点
    match_results = match_vehicles_to_reference(distances_all, reference_ids)
    print(f"匹配完成，共 {len(match_results)} 个匹配结果")

    # 5. 构建ID映射关系
    id_mapping = []  # (原始ID, 匹配ID)
    folder_counters = {i: 0 for i in range(len(subdirs))}  # 每个文件夹的计数器

    for folder_idx, matched_id in match_results:
        if folder_idx not in folder_counters:
            continue

        counter = folder_counters[folder_idx]
        if counter < len(vehicle_ids_all[folder_idx]):
            original_id = vehicle_ids_all[folder_idx][counter]
            id_mapping.append((original_id, matched_id))
            folder_counters[folder_idx] += 1

    # 6. 计算可见性比率
    visibility_values = []

    for original_id, matched_id in id_mapping:
        folder_idx = None
        # 查找原始ID所在的文件夹
        for idx, id_list in enumerate(vehicle_ids_all):
            if original_id in id_list:
                folder_idx = idx
                break

        if folder_idx is None:
            print(f"警告: 未找到原始ID {original_id} 所在子目录，跳过")
            visibility_values.append([])
            continue

        # 读取参考文件
        ref_csv = os.path.join(subdirs[folder_idx], "reference(vehicle_id+time+coord).csv")
        if not os.path.exists(ref_csv):
            print(f"警告: {ref_csv} 不存在，跳过")
            visibility_values.append([])
            continue

        # 提取匹配值
        match_values = []
        with open(ref_csv, 'r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        vid = int(row[0])
                        if vid == original_id:
                            match_values.append(row[1])
                    except ValueError:
                        continue

        # 获取可见性比率
        folder_path = subdirs[folder_idx]
        vis_ratios = [get_visibility_ratio(folder_path, val) for val in match_values]
        visibility_values.append([v for v in vis_ratios if v is not None])

    # 7. 计算平均可见性比率
    avg_visibility = []
    for ratios in visibility_values:
        if ratios:
            avg_visibility.append(sum(ratios) / len(ratios))
        else:
            avg_visibility.append(0.0)

    # 8. 计算每个文件夹的平均可见性比率（用于填充矩阵）
    folder_avg_visibility = [calculate_folder_visibility_avg(d) for d in subdirs]

    # 9. 构建所有ID列表（参考点ID + 额外ID）
    extra_ids = sorted(set(mid for _, mid in match_results if mid >= 1000))
    all_ids = reference_ids + extra_ids
    id_to_col = {vid: idx for idx, vid in enumerate(all_ids)}

    # 10. 创建并保存违停车辆矩阵
    num_rows = len(subdirs)
    num_cols = len(all_ids)
    parking_matrix = create_matrix(num_rows, num_cols, 0)

    for folder_idx, matched_id in match_results:
        col_idx = id_to_col.get(matched_id)
        if col_idx is not None:
            parking_matrix[folder_idx][col_idx] = 1

    parking_matrix_path = os.path.join(ROOT_DIR, "illegal_parking_matrix.csv")
    save_matrix_to_csv(
        parking_matrix,
        ["VideoFolderName"] + [str(vid) for vid in all_ids],
        folder_names,
        parking_matrix_path
    )
    print(f"违停车辆矩阵已保存至: {parking_matrix_path}")

    # 11. 创建并保存检测置信度矩阵
    confidence_matrix = create_matrix(num_rows, num_cols, 0.0)

    # 准备置信度列表 (folder_idx, matched_id, confidence)
    confidence_list = []
    for (folder_idx, matched_id), (original_id, _) in zip(match_results, id_mapping):
        folder_vehicles = vehicle_ids_all[folder_idx]
        if original_id in folder_vehicles:
            idx_in_folder = folder_vehicles.index(original_id)
            confidence = confidences_all[folder_idx][idx_in_folder]
            confidence_list.append((folder_idx, matched_id, confidence))

    # 填充置信度矩阵
    for folder_idx, matched_id, conf in confidence_list:
        col_idx = id_to_col.get(matched_id)
        if col_idx is not None:
            confidence_matrix[folder_idx][col_idx] = conf

    confidence_matrix_path = os.path.join(ROOT_DIR, "detection_confidence_matrix.csv")
    save_matrix_to_csv(
        confidence_matrix,
        ["VideoFolderName"] + [str(vid) for vid in all_ids],
        folder_names,
        confidence_matrix_path
    )
    print(f"检测置信度矩阵已保存至: {confidence_matrix_path}")

    # 12. 创建并保存加权矩阵
    weighted_matrix = create_matrix(num_rows, num_cols, 0.0)

    # 填充加权矩阵
    for i, (folder_idx, matched_id) in enumerate(match_results):
        col_idx = id_to_col.get(matched_id)
        if col_idx is not None:
            weighted_matrix[folder_idx][col_idx] = avg_visibility[i]

    # 用文件夹平均可见性比率填充空白
    for i in range(num_rows):
        if not valid_flags[i]:
            continue
        for j in range(num_cols):
            if weighted_matrix[i][j] == 0.0:
                weighted_matrix[i][j] = folder_avg_visibility[i]

    # ===== 新增补丁：处理全零行 =====
    for i in range(num_rows):
        if not valid_flags[i]:
            continue

        # 检查是否整行都是0.0
        if all(val == 0.0 for val in weighted_matrix[i]):
            # 使用该文件夹的可见性比率平均值填充整行
            for j in range(num_cols):
                weighted_matrix[i][j] = folder_avg_visibility[i]
            print(f"警告: 文件夹 {folder_names[i]} 的加权矩阵全为零，已用平均值 {folder_avg_visibility[i]:.4f} 填充整行")

    # 保存加权矩阵
    weighted_matrix_path = os.path.join(ROOT_DIR, "weighted_matrix.csv")
    save_matrix_to_csv(
        weighted_matrix,
        ["VideoFolderName"] + [str(vid) for vid in all_ids],
        folder_names,
        weighted_matrix_path
    )
    print(f"加权矩阵已保存至: {weighted_matrix_path}")


if __name__ == "__main__":
    main()