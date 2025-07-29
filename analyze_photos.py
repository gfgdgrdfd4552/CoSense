import os
import json
import re
import math
from collections import defaultdict

def calculate_visible_area(metadata):
    """计算考虑遮挡后的可见区域面积"""
    cam = metadata['camera']
    cam_x = cam['x']
    cam_y = cam['y']
    cam_yaw = cam['yaw']
    fov = cam['fov']
    max_range = cam['max_range']
    
    # 扇形区域参数
    start_angle = math.radians(cam_yaw - fov/2)
    end_angle = math.radians(cam_yaw + fov/2)
    sector_radius = max_range
    
    # 初始化为完整扇形
    blocked_angles = []
    
    # 处理每个车辆（使用'vehicles'键而不是'other_vehicles'）
    for vehicle in metadata.get('vehicles', []):
        vx = vehicle['x']
        vy = vehicle['y']
        
        # 计算相对坐标
        dx = vx - cam_x
        dy = vy - cam_y
        distance = math.hypot(dx, dy)
        
        # 超出探测范围则跳过
        if distance > sector_radius:
            continue
            
        # 计算相对角度
        angle = math.atan2(dy, dx)
        angle_deg = math.degrees(angle)
        
        # 转换为与扇形方向匹配的角度
        rel_angle = math.degrees(angle - math.radians(cam_yaw))
        rel_angle = (rel_angle + 180) % 360 - 180  # 规范到[-180, 180]
        
        # 判断是否在扇形视野内
        if abs(rel_angle) > fov/2:
            continue
            
        # 计算车辆占据的角度范围（简化模型）
        vehicle_length = vehicle.get('length', 4.5)  # 默认车长4.5米
        angular_width = math.degrees(2 * math.atan(vehicle_length/(2*distance))) if distance != 0 else 0
        start_block = angle_deg - angular_width/2
        end_block = angle_deg + angular_width/2
        
        blocked_angles.append((start_block, end_block))
    
    # 合并重叠的遮挡区间
    blocked_angles = merge_intervals(blocked_angles)
    
    # 计算剩余可见角度
    total_visible = fov
    for start, end in blocked_angles:
        total_visible -= (end - start)
    
    # 计算面积（扇形面积公式：0.5 * r² * θ）
    visible_area = 0.5 * sector_radius**2 * math.radians(max(total_visible, 0))
    
    return visible_area

def merge_intervals(intervals):
    """合并重叠的区间"""
    if not intervals:
        return []
    
    sorted_intervals = sorted(intervals, key=lambda x: x[0])
    merged = [sorted_intervals[0]]
    
    for current in sorted_intervals[1:]:
        last = merged[-1]
        if current[0] <= last[1]:
            merged[-1] = (last[0], max(last[1], current[1]))
        else:
            merged.append(current)
    
    return merged

def get_vehicle_id_from_path(filepath):
    """从文件路径中提取车辆ID"""
    match = re.search(r'vehicle_(\d+)', filepath)
    return int(match.group(1)) if match else None

def analyze_photos(input_dir, output_dir, top_n=3):
    """
    分析照片并评分
    :param input_dir: 输入目录路径
    :param output_dir: 输出目录路径
    :param top_n: 每个位置取前N名
    """
    position_data = defaultdict(lambda: defaultdict(float))  # {x_pos: {vehicle_id: area}}
    target_positions = [80, 70, 60, 50, 40, 30, 20, 10, 0]
    
    print("开始分析各车辆感知面积：")
    for root, _, files in os.walk(input_dir):
        for file in files:
            if file.endswith("_meta.json"):
                meta_path = os.path.join(root, file)
                
                try:
                    # 从路径提取车辆ID
                    vehicle_id = get_vehicle_id_from_path(root)
                    if vehicle_id is None:
                        continue

                    with open(meta_path, 'r') as f:
                        metadata = json.load(f)

                    # 提取x位置
                    try:
                        x_pos = int(file.split('_')[0][1:])  # 格式如"x80_..."
                    except:
                        continue

                    if x_pos not in target_positions:
                        continue

                    # 计算可见区域面积
                    visible_area = calculate_visible_area(metadata)
                    position_data[x_pos][vehicle_id] = visible_area

                    # 打印单个车辆单位置数据
                    print(f"  Vehicle {vehicle_id:4d} @ x={x_pos:2d} : {visible_area:8.2f} m²")

                except Exception as e:
                    print(f"处理文件 {meta_path} 时出错: {str(e)}")
                    continue
    # 评分系统
    vehicle_scores = defaultdict(int)
    
    # 对每个位置处理
    print("\n各位置评分结果：")
    for x_pos in target_positions:
        if x_pos not in position_data:
            continue
        
        # 获取该位置所有车辆数据
        vehicles_at_pos = position_data[x_pos]
        
        # 按面积排序
        sorted_vehicles = sorted(vehicles_at_pos.items(), 
                               key=lambda x: x[1], 
                               reverse=True)
        
        # 打印位置头信息
        print(f"\n位置 x={x_pos}:")
        print("-"*40)
        print("{:<12} {:<15} {:<10}".format("Vehicle ID", "Area (m²)", "Score"))
        
        # 给前N名赋分1
        for i, (vehicle_id, area) in enumerate(sorted_vehicles):
            score = 1 if i < top_n else 0
            vehicle_scores[vehicle_id] += score
            print("{:<12} {:<15.2f} {:<10}".format(vehicle_id, area, score))

    # 按总分排序车辆
    sorted_scores = sorted(vehicle_scores.items(), 
                         key=lambda x: x[1], 
                         reverse=True)

    # 输出最终结果
    print("\n最终得分排名：")
    print("="*50)
    print("{:<6} {:<12} {:<10}".format("Rank", "Vehicle ID", "Total Score"))
    for rank, (vehicle_id, score) in enumerate(sorted_scores[:top_n], 1):
        print("{:<6} {:<12} {:<10}".format(rank, vehicle_id, score))

if __name__ == "__main__":
    input_directory = "output/camera_images180"
    output_directory = "output/analysis_results"  # 仍需要输出目录存放报告
    top_n_vehicles = 3
    
    analyze_photos(input_directory, output_directory, top_n_vehicles)