'''中间车道行驶时计算的每帧的遮挡角度和可视比率'''

import os
import json
import math
import pandas as pd

# 目标文件夹路径
#folder_path = (r"D:\evaluation\vehicle_tracking\New\1\vehicle_1888")
import sys  # 添加这一行

# 从命令行参数读取目标文件夹路径
if len(sys.argv) < 2:
    print("请提供子文件夹路径作为参数")
    sys.exit(1)

folder_path = sys.argv[1]  # 替代原来的写死路径

detect_range_2 = 20#探测距离
length = 4.5#长
width = 2.0#宽
target_range = [(40, 90)]#可视角度范围
target_total_span = sum(end - start for start, end in target_range)
# 初始化结果数组
result = []

# 遍历文件夹中的所有 JSON 文件
for filename in os.listdir(folder_path):
    if filename.endswith(".json"):
        json_path = os.path.join(folder_path, filename)
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                timestamp = data.get("timestamp", None)
                camera = data.get("camera", {})
                cam_x = camera.get("x", None)
                cam_y = camera.get("y", None)

                # 检查字段有效性
                if timestamp is not None and cam_x is not None and cam_y is not None:
                    transformed_coords = [-cam_y, -cam_x]  # 对调并取反
                    result.append([filename, timestamp, transformed_coords])
        except Exception as e:
            print(f"Error reading {filename}: {e}")



def append_closest_vehicle_info(folder_path, result):
    for item in result:
        filename = item[0]
        cam_transformed = item[2]  # [new_x, new_y]

        json_path = os.path.join(folder_path, filename)
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)

                min_dist = float('inf')
                selected_vehicle = None

                for vehicle in data.get("vehicles", []):
                    vx, vy = vehicle.get("x"), vehicle.get("y")
                    if vx is None or vy is None:
                        continue

                    # 变换车辆坐标（x, y互换并取反）
                    new_x = -vy
                    new_y = -vx

                    # 条件1：在相机前方
                    if new_y <= cam_transformed[1]:
                        continue

                    # 条件2：横向接近
                    if abs(new_x - cam_transformed[0]) > 1:
                        continue

                    # 条件3：距离小于20
                    dist = math.sqrt((new_x - cam_transformed[0])**2 + (new_y - cam_transformed[1])**2)
                    if dist < detect_range_2 and dist < min_dist:
                        min_dist = dist
                        selected_vehicle = {
                            "id": vehicle["id"],
                            "coord": [new_x, new_y]
                        }

                # 如果找到车辆，则添加
                if selected_vehicle:
                    item.extend([selected_vehicle["id"], selected_vehicle["coord"]])
                else:
                    item.extend([None, None])  # 没找到时补None占位
                    # 判断第四个和第五个字段是否为None


        except Exception as e:
            print(f"Error processing {filename}: {e}")

def calculate_angle_ACB(result):
    for item in result:
        # 点 A：车辆末尾中心点
        A_x, A_y = item[4]

        # 点 B：车辆末尾右边（假设+1为右）
        B_x = A_x + width / 2
        B_y = A_y - length / 2

        # 点 C：照相机位置
        C_x, C_y = item[2]

        # 向量 CA 和 CB
        CA_x = A_x - C_x
        CA_y = A_y - C_y
        CB_x = B_x - C_x
        CB_y = B_y - C_y

        # 点积
        dot = CA_x * CB_x + CA_y * CB_y

        # 模长
        mag_CA = math.sqrt(CA_x**2 + CA_y**2)
        mag_CB = math.sqrt(CB_x**2 + CB_y**2)

        # 防止除以 0 或超出 [-1, 1]
        if mag_CA == 0 or mag_CB == 0:
            angle = 0.0
        else:
            cos_theta = max(-1, min(1, dot / (mag_CA * mag_CB)))
            angle = math.degrees(math.acos(cos_theta))

        # 添加角度
        item.append(angle)



def append_normalized_angle_score(result):
    for item in result:
        if len(item) < 6:
            continue  # 确保角度存在

        angle = item[5]  # 第6个元素为角度（单位：度）
        normalized = (target_total_span - angle) / target_total_span
        item.append(normalized)


append_closest_vehicle_info(folder_path, result)

for item in result:
    if item[3] is None or item[4] is None:
        item.append(0)  # 第六个字段设为None
        item.append(1)     # 第七个字段设为1

# 继续执行后续函数
if all(item[3] is not None and item[4] is not None for item in result):
    calculate_angle_ACB(result)
    append_normalized_angle_score(result)


# 输出结果
for row in result:#[json文件名字，时间，照相机坐标，遮挡车编号，遮挡车坐标，遮挡角度，可视角度/总观测角度值]
    print(row)
data = []

for entry in result:
    if len(entry) >= 6:
        time = round(entry[1], 2)
        occlusion = entry[5]
        occlusion_start = 90 - occlusion
        if occlusion_start < 40:
            occlusion_start = 40
        occlusion_range = f"[({occlusion_start}, 90)]"
        if entry[6] < 0:
            entry[6] = 0
        data.append([time, occlusion_range, entry[6]])

# 创建 DataFrame
df = pd.DataFrame(data, columns=["Time", "Occlusion Angle", "Visibility Ratio"])

# 保存路径（保存到与 path 变量定义的一致目录下）
save_path = os.path.join(folder_path, "visibility_ratio_log.csv")
df.to_csv(save_path, index=False)

print(f"📄 CSV 文件已成功保存到：{save_path}")