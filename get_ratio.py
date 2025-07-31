import os
import pandas as pd

base_folder = "D:/evaluation/vehicle_tracking/normal2"

# 存储结果：[(vehicle_id, avg_ratio), ...]
visibility_result = []

for folder_name in os.listdir(base_folder):
    folder_path = os.path.join(base_folder, folder_name)
    if os.path.isdir(folder_path) and folder_name.startswith("vehicle_"):
        csv_path = os.path.join(folder_path, "visibility_ratio_log.csv")
        if os.path.exists(csv_path):
            try:
                df = pd.read_csv(csv_path)  # 删除 sep="\t"，使用默认逗号
                # 清洗列名（有些可能多了空格或拼写不一致）
                df.columns = [col.strip() for col in df.columns]
                for possible_col in ["Visibility Ratio", "Visibility ratio", "visibility_ratio"]:
                    if possible_col in df.columns:
                        mean_ratio = df[possible_col].astype(float).mean()
                        vehicle_id = folder_name.replace("vehicle_", "")
                        visibility_result.append((vehicle_id, round(mean_ratio * 100, 2)))  # 百分比
                        break
            except Exception as e:
                print(f"读取失败: {csv_path}，错误: {e}")

# 打印结果
# 打印纯百分比列表
ratios = [r for _, r in visibility_result]
print(ratios)
print("各车辆平均可视率（百分比）:")
for vid, ratio in visibility_result:
    print(f"Vehicle {vid}: {ratio}%")
