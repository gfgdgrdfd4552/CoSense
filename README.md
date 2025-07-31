# 车辆跟踪与违停检测系统

本项目用于视频中的车辆跟踪、违停检测、视野遮挡分析，以及矩阵统计与EM推断。各模块通过 Python 实现，适用于交通视频数据的处理与分析。

## 目录结构与功能说明

### 1. 车辆检测与跟踪

- **tracking.py**  
  使用YOLOv5进行车辆检测，结合SORT算法实现多目标跟踪。输出每帧检测结果至 `detection_output.csv`，并生成带跟踪标识的视频。

- **sort.py**  
  SORT（Simple Online and Realtime Tracking）跟踪算法的实现，用于目标的实时跟踪与ID分配。

### 2. 车辆遮挡角度与可视比率计算

- **angle_calculate.py**  
  总控脚本，遍历指定目录下的子文件夹，自动根据摄像头位置选择合适的遮挡角度分析脚本（angle_calculate_1/2/3）。

- **angle_calculate_1.py**  
  针对中间车道，计算每帧主车的遮挡角度和可视比率，输出 `visibility_ratio_log.csv`。

- **angle_calculate_2.py**  
  针对右三车道，计算相应遮挡角度和可视比率。

- **angle_calculate_3.py**  
  针对左边车道，计算遮挡角度和可视比率。

- **angle_curve.py**  
  针对有弯道的场景，分析主车视野内的遮挡角度，输出遮挡区间和可见性比率。

### 3. 违停车辆处理与坐标统计

- **depth_output.py**  
  处理检测到的违停车辆，聚类其全局坐标并输出 `illegal_vehicles_global_avg_coords.csv` 和参考点 `reference(vehicle_id+time+coord).csv`。

### 4. 统计分析与矩阵构建

- **get_ratio.py**  
  统计每个车辆的平均可视比率。遍历所有车辆文件夹，汇总并输出可视率结果。

- **illegal_parking_matrix.py**  
  生成违停车辆矩阵（是否存在）、置信度矩阵和加权矩阵（结合可见性比率），分别输出到对应的CSV文件。

- **EM-PCIP-V2.py**  
  对违停矩阵和置信度矩阵进行EM推断，评估每辆车为违停的概率。

### 5. 数据文件

- **illegal_parking_matrix.csv**  
  违停车辆二进制矩阵，行代表视频/车辆，列为参考点/车辆ID，值为0或1。

- **detection_confidence_matrix.csv**  
  检测置信度矩阵，数值代表检测置信度。

### 其他说明

- 本项目依赖于 `torch`, `opencv-python`, `numpy`, `pandas`, `matplotlib`, `scikit-learn`, `filterpy` 等第三方库。
- 各脚本参数和路径需根据实际数据集进行调整。
- 推荐运行顺序：车辆检测与跟踪 → 遮挡角度分析 → 坐标聚类 → 矩阵统计 → EM推断。

## 快速开始

1. 准备好车辆视频和相关数据，放入指定目录结构。
2. 运行 `tracking.py` 进行车辆检测与跟踪。
3. 执行 `angle_calculate.py`/`angle_curve.py` 进行遮挡分析。
4. 使用 `depth_output.py` 处理违停车辆坐标。
5. 运行 `illegal_parking_matrix.py` 生成矩阵。
6. 用 `EM-PCIP-V2.py` 进行概率推断。

## 联系方式

如有疑问或建议，请联系xiang.chaocan@gmail.com。
