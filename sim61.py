import os
import time
import random
import weakref
import math
import carla
import json
import cv2
import numpy as np
import threading
import queue
from queue import Queue

# 常量定义
NUM_PARKED_VEHICLES = 9     # 违停车辆数量
NUM_SENSOR_VEHICLES = 5    # 感知车辆数量增加到20辆
NUM_NORMAL_VEHICLES = 0   # 普通车辆数量
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 640           # 摄像头图像宽度
IMAGE_HEIGHT = 480          # 摄像头图像高度
LANE_WIDTH = 3.5            # 单车道宽度(米)
FPS = 30                    # 视频帧率

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 7            # 道路总宽度
VEHICLE_LENGTH = 4.5        # 平均车长(米)
VEHICLE_WIDTH = 2.0         # 平均车宽(米)
VEHICLE_SPACING = 1.0       # 车辆间距(米)

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

# 车道中心线定义
LEFT_LANE_Y = 15.2 + LANE_WIDTH / 2   # 左车道中心线 y=17.95
RIGHT_LANE_Y = 15.2 - LANE_WIDTH / 2  # 右车道中心线 y=12.45

# 感知车辆间距（确保前方有车辆遮挡）
SENSOR_VEHICLE_SPACING = 5.0  # 感知车辆间距（米）

class VideoWriterThread(threading.Thread):
    """异步视频写入线程（仅支持RGB视频）"""
    
    def __init__(self, vehicle_id):
        super().__init__()
        self.vehicle_id = vehicle_id
        self.save_dir = f'output/camera_images/vehicle_{vehicle_id}'
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 初始化视频写入器
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # RGB视频写入器
        rgb_video_path = f'{self.save_dir}/rgb_recording_{timestamp}.mp4'
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.rgb_writer = cv2.VideoWriter(rgb_video_path, fourcc, FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))
        
        # 图像队列（存储rgb_frame）
        self.queue = queue.Queue(maxsize=30)  # 限制队列大小防止内存溢出
        self.stop_event = threading.Event()
        self.daemon = True  # 守护线程，主程序退出时自动结束
        self.frame_count = 0
        
    def run(self):
        """线程主函数"""
        print(f"Video writer thread started for vehicle {self.vehicle_id}")
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                # 从队列获取图像，最多等待1秒
                rgb_array = self.queue.get(timeout=1.0)
                
                # 写入RGB视频
                self.rgb_writer.write(rgb_array)
                
                self.queue.task_done()
                self.frame_count += 1
            except queue.Empty:
                continue
        
        # 释放资源
        self.rgb_writer.release()
        print(f"Video writer for vehicle {self.vehicle_id} released, wrote {self.frame_count} frames")
    
    def add_frame(self, rgb_array):
        """添加帧到队列"""
        if not self.stop_event.is_set():
            try:
                # 如果队列已满，丢弃旧帧
                if self.queue.full():
                    self.queue.get_nowait()
                
                # 存储RGB图像
                self.queue.put_nowait(rgb_array.copy())
            except queue.Full:
                pass  # 忽略队列满异常
            except Exception as e:
                print(f"Error adding frame to video queue: {e}")
    
    def stop(self):
        """停止线程"""
        self.stop_event.set()
        self.join()  # 等待线程结束

class StreetMonitor:
    """道路监控器"""

    @staticmethod
    def calculate_parking_positions():
        """在指定位置(x=15.39, y=9.76)沿着道路方向生成违停车辆"""
        parking_positions = []
        
        # 计算道路方向向量
        direction = carla.Vector3D(
            x=STREET_END.x - STREET_START.x,
            y=STREET_END.y - STREET_START.y,
            z=0
        )
        road_length = math.sqrt(direction.x ** 2 + direction.y ** 2)
        if road_length > 0:
            direction = direction / road_length  # 单位化向量
        
        # 计算朝向(与道路方向相同)
        yaw = math.degrees(math.atan2(direction.y, direction.x))
        
        # 沿着道路方向排列车辆
        for i in range(NUM_PARKED_VEHICLES):
            # 计算每辆车的偏移量
            offset = direction * (i * (VEHICLE_LENGTH + VEHICLE_SPACING))
            
            parking_pos = carla.Location(
                x=ILLEGAL_PARKING_POINT.x + offset.x,
                y=ILLEGAL_PARKING_POINT.y + offset.y,
                z=0.1
            )
            
            parking_positions.append(
                carla.Transform(parking_pos, carla.Rotation(yaw=yaw)))
            
            print(f"Created illegal parking spot at ({parking_pos.x:.2f}, {parking_pos.y:.2f}) "
                  f"with yaw: {yaw:.1f}°")

        return parking_positions

    @staticmethod
    def is_on_street(location):
        """检查位置是否在目标道路上"""
        # 计算点到线段的距离
        line_vec = carla.Vector3D(
            x=STREET_END.x - STREET_START.x,
            y=STREET_END.y - STREET_START.y,
            z=0
        )
        point_vec = carla.Vector3D(
            x=location.x - STREET_START.x,
            y=location.y - STREET_START.y,
            z=0
        )
        line_len = math.sqrt(line_vec.x ** 2 + line_vec.y ** 2)
        if line_len == 0:
            return False

        # 计算投影比例
        t = max(0, min(1, (point_vec.x * line_vec.x + point_vec.y * line_vec.y) / line_len ** 2))
        projection = carla.Location(
            x=STREET_START.x + t * line_vec.x,
            y=STREET_START.y + t * line_vec.y,
            z=0
        )

        # 计算垂直距离
        dist = math.sqrt((location.x - projection.x) ** 2 + (location.y - projection.y) ** 2)
        return dist <= STREET_WIDTH / 2 and t > 0 and t < 1  # 确保在起点终点之间

class VehicleRecorder:
    """车辆数据记录器（仅支持RGB数据）"""
    
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.save_dir = f'output/camera_images/vehicle_{vehicle_id}'
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 初始化视频写入线程
        self.video_thread = VideoWriterThread(vehicle_id)
        self.video_thread.start()
        
        # 初始化数据文件
        self.data_file = open(f'{self.save_dir}/vehicle_data.txt', 'w')
        self.data_file.write("frame_number,relative_time(s),x,y,z,speed(km/h)\n")
        
        # 计算相机内参
        self.camera_intrinsics = self._calculate_camera_intrinsics()
        
        # 保存相机内参到文件
        with open(f'{self.save_dir}/camera_intrinsics.txt', 'w') as f:
            f.write(f"Focal length (fx): {self.camera_intrinsics['fx']:.4f}\n")
            f.write(f"Focal length (fy): {self.camera_intrinsics['fy']:.4f}\n")
            f.write(f"Principal point (cx): {self.camera_intrinsics['cx']:.4f}\n")
            f.write(f"Principal point (cy): {self.camera_intrinsics['cy']:.4f}\n")
            f.write(f"Image width: {IMAGE_WIDTH}\n")
            f.write(f"Image height: {IMAGE_HEIGHT}\n")
            f.write(f"Field of View: 100 degrees\n")
        
        # 记录状态
        self.is_recording = False
        self.has_recorded = False
        self.start_time = None
        self.frame_count = 0
        self.data_count = 0
        self.last_frame_time = 0
        self.current_relative_time = 0.0  # 当前帧相对时间
        
    def _calculate_camera_intrinsics(self):
        """计算相机内参"""
        fov = 100  # 视野100度
        f = IMAGE_WIDTH / (2 * math.tan(math.radians(fov / 2)))
        
        return {
            'fx': f,
            'fy': f,
            'cx': IMAGE_WIDTH / 2,
            'cy': IMAGE_HEIGHT / 2
        }
    
    def record_frame(self, rgb_image, vehicle):
        """记录视频帧和车辆数据"""
        if not self.is_recording:
            return
            
        # 帧计数器递增
        self.frame_count += 1
        
        # 计算当前帧相对时间
        self.current_relative_time = self.frame_count / FPS
        
        # 转换RGB图像格式
        rgb_array = np.frombuffer(rgb_image.raw_data, dtype=np.dtype("uint8"))
        rgb_array = np.reshape(rgb_array, (rgb_image.height, rgb_image.width, 4))
        rgb_array = rgb_array[:, :, :3]  # 去除alpha通道
        rgb_array = rgb_array[:, :, ::-1]  # 从RGB转换为BGR
        
        # 添加帧到视频线程队列
        self.video_thread.add_frame(rgb_array)
        
        # 获取当前时间
        current_time = time.time()
        
        # 控制数据记录频率（每秒10次）
        if current_time - self.last_frame_time >= 0.1:
            self._record_vehicle_data(vehicle)
            self.last_frame_time = current_time
    
    def _record_vehicle_data(self, vehicle):
        """记录车辆位置和速度数据"""
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 转换为km/h
        
        self.data_count += 1
        self.data_file.write(
            f"{self.frame_count},"
            f"{self.current_relative_time:.3f},"  # 使用当前相对时间
            f"{location.x:.3f},"
            f"{location.y:.3f},"
            f"{location.z:.3f},"
            f"{speed:.2f}\n"
        )
        self.data_file.flush()
    
    def start_recording(self):
        """开始记录"""
        if not self.has_recorded:  # 仅第一次进入时记录
            self.is_recording = True
            self.has_recorded = True
            self.start_time = time.time()  # 记录开始时间
            self.frame_count = 0           # 重置帧计数器
            self.data_count = 0            # 重置数据计数器
            self.last_frame_time = time.time()
            print(f"Started recording for vehicle {self.vehicle_id}")
    
    def stop_recording(self):
        """停止记录"""
        if self.is_recording:
            self.is_recording = False
            duration = (time.time() - self.start_time) if self.start_time else 0
            print(f"Stopping recording for vehicle {self.vehicle_id}. "
                  f"Duration: {duration:.2f}s, Frames: {self.frame_count}, Data points: {self.data_count}")
            
            # 停止视频线程
            self.video_thread.stop()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器（仅支持RGB摄像机）"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.rgb_sensor = None
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        self.rgb_queue = Queue()
        
        # 记录状态控制
        self.was_on_street = False  # 上一帧是否在道路上
        self.consecutive_off_street = 0  # 连续不在道路上的帧数
        self.max_off_street_frames = 30  # 最大允许离开道路的帧数（1秒）
        self.last_save_time = time.time()  # 上次保存图像的时间
        self.save_interval = 1.0  # 保存图像的时间间隔（秒）
        
        # 初始化RGB摄像头
        rgb_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        rgb_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        rgb_bp.set_attribute('fov', '100')  # 视野100度

        # 安装位置
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))  # 摄像头安装位置
        
        # 生成RGB摄像头
        self.rgb_sensor = self.world.spawn_actor(rgb_bp, transform, attach_to=self.vehicle)

        # 设置图像回调函数
        weak_self = weakref.ref(self)
        self.rgb_sensor.listen(lambda image: self._parse_rgb_image(weak_self, image))

    @staticmethod
    def _parse_rgb_image(weak_self, image):
        """处理RGB图像"""
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return
            
        # 将图像放入队列
        self.rgb_queue.put(image)
        self._process_images()

    def _process_images(self):
        """处理RGB图像"""
        # 当队列有数据时处理
        while not self.rgb_queue.empty():
            rgb_image = self.rgb_queue.get()
            
            # 获取当前车辆位置
            vehicle_location = self.vehicle.get_location()
            
            # 检查是否在目标道路上
            on_street = StreetMonitor.is_on_street(vehicle_location)
            
            # 更新连续离开道路的帧数
            if not on_street:
                self.consecutive_off_street += 1
            else:
                self.consecutive_off_street = 0
            
            # 控制记录状态
            if on_street and not self.recorder.is_recording and not self.recorder.has_recorded:
                # 第一次进入道路，开始记录
                self.recorder.start_recording()
                self.was_on_street = True
            elif self.recorder.is_recording:
                if self.consecutive_off_street >= self.max_off_street_frames:
                    # 连续离开道路超过阈值，停止记录
                    self.recorder.stop_recording()
                    print(f"Vehicle {self.vehicle.id} left street for too long, stopped recording")
                    return
                # 否则继续记录（即使暂时离开道路）
            
            # 如果正在记录，处理帧
            if self.recorder.is_recording:
                self.recorder.record_frame(rgb_image, self.vehicle)
                
                # 定期保存图像（每秒保存1张）
                current_time = time.time()
                if current_time - self.last_save_time >= self.save_interval:
                    self._save_image_and_metadata(rgb_image, vehicle_location)
                    self.last_save_time = current_time
                    
            # 清理队列
            self.rgb_queue.task_done()

    def _save_image_and_metadata(self, rgb_image, vehicle_location):
        """保存图片和元数据，使用相对时间命名"""
        # 获取当前记录器的相对时间
        relative_time = self.recorder.current_relative_time
        
        # 创建时间戳字符串
        timestamp_str = f"{relative_time:.3f}"
        
        # 按车辆ID创建子目录
        save_dir = f'output/camera_images/vehicle_{self.vehicle.id}'
        os.makedirs(save_dir, exist_ok=True)

        try:
            # 获取所有车辆信息
            all_actors = self.world.get_actors().filter('vehicle.*')
            vehicle_data = []
            
            # 记录除自身外的所有车辆
            for actor in all_actors:
                if actor.id != self.vehicle.id:
                    loc = actor.get_location()
                    rotation = actor.get_transform().rotation
                    vehicle_data.append({
                        'id': actor.id,
                        'x': loc.x,
                        'y': loc.y,
                        'z': loc.z,
                        'yaw': rotation.yaw,
                        'length': 4.5,  # 假设车长4.5米
                        'width': 2.0     # 假设车宽2米
                    })

            # 获取相机参数
            camera_transform = self.rgb_sensor.get_transform()
            metadata = {
                'timestamp': relative_time,  # 使用相对时间
                'camera': {
                    'x': camera_transform.location.x,
                    'y': camera_transform.location.y,
                    'z': camera_transform.location.z,
                    'yaw': camera_transform.rotation.yaw,
                    'fov': 100,        # 与摄像头蓝图设置一致
                    'max_range': 30.0,  # 探测最大半径30米
                    'intrinsics': self.recorder.camera_intrinsics  # 添加内参
                },
                'vehicles': vehicle_data
            }

            # 保存元数据为JSON文件
            metadata_path = f'{save_dir}/{timestamp_str}_meta.json'
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f)
            
            # 保存RGB图片
            rgb_image.save_to_disk(
                f'{save_dir}/{timestamp_str}_rgb.png'
            )
            
            print(f"Saved image at time={timestamp_str}s for sensor vehicle {self.vehicle.id}")
        except Exception as e:
            print(f"Error saving image and metadata: {e}")

def generate_sensor_spawn_points():
    """为感知车辆生成优化的生成点（右侧车道优先，且前方无车辆）
    主要逻辑：将大部分感知车辆安排在右侧车道，分为前方、中间、后方区域，确保分布均匀
    """
    sensor_spawn_points = []
    
    # 将道路分为三个区域：前方、中间、后方
    road_length = STREET_START.x - STREET_END.x  # 90米
    
    # 前方区域：x=80到x=50（30米）- 右侧车道优先安排感知车辆
    front_section_start = 80
    front_section_end = 50
    front_vehicles_right_lane = 0  # 右侧车道安排更多感知车辆
    
    # 中间区域：x=50到x=20（30米）- 右侧车道安排感知车辆
    middle_section_start = 50
    middle_section_end = 20
    middle_vehicles_right_lane = 4
    
    # 后方区域：x=20到x=-10（30米）- 右侧车道安排少量感知车辆
    rear_section_start = 20
    rear_section_end = -10
    rear_vehicles_right_lane = 2
    
    # 右侧车道优先安排感知车辆
    # 前方区域 - 右侧车道
    for i in range(front_vehicles_right_lane):
        x = front_section_start - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),  # 右侧车道
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"前部感知车辆生成点 (右侧车道): x={x:.1f}, y={RIGHT_LANE_Y:.1f}")
    
    # 中间区域 - 右侧车道
    for i in range(middle_vehicles_right_lane):
        x = middle_section_start - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),  # 右侧车道
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"中间感知车辆生成点 (右侧车道): x={x:.1f}, y={RIGHT_LANE_Y:.1f}")
    
    # 后方区域 - 右侧车道
    for i in range(rear_vehicles_right_lane):
        x = rear_section_start - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),  # 右侧车道
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"后部感知车辆生成点 (右侧车道): x={x:.1f}, y={RIGHT_LANE_Y:.1f}")
    
    # 剩余感知车辆安排到左侧车道
    remaining_sensors = NUM_SENSOR_VEHICLES - (front_vehicles_right_lane + middle_vehicles_right_lane + rear_vehicles_right_lane)
    if remaining_sensors > 0:
        left_lane_sensors = min(remaining_sensors, 8)  # 限制左侧车道感知车辆数量
        spacing = road_length / (left_lane_sensors + 1)
        for i in range(left_lane_sensors):
            x = STREET_START.x - ((i + 1) * spacing) - random.uniform(0, 2)
            spawn_point = carla.Transform(
                carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),  # 左侧车道
                carla.Rotation(yaw=180)
            )
            sensor_spawn_points.append(spawn_point)
            print(f"感知车辆生成点 (左侧车道): x={x:.1f}, y={LEFT_LANE_Y:.1f}")
    
    print(f"生成了 {len(sensor_spawn_points)} 个优化的感知车辆生成点 (优先右侧车道)")
    return sensor_spawn_points

def generate_normal_spawn_points():
    """为普通车辆生成密集的生成点（集中在左侧车道，避免右侧车道前方区域）
    主要逻辑：将大部分普通车辆安排在左侧车道，右侧车道仅在后方放置少量车辆，避免挡住感知车辆视线
    """
    spawn_points = []
    
    # 道路长度
    road_length = STREET_START.x - STREET_END.x  # 90米
    
    # 计算需要的生成点数量（考虑车辆长度和间距）
    vehicle_unit_length = VEHICLE_LENGTH + 0.5  # 拥挤交通间距0.5米
    max_vehicles_per_lane = int(road_length / vehicle_unit_length)  # 每车道最大车辆数
    
    print(f"道路长度: {road_length}m, 每车道最大普通车辆数: {max_vehicles_per_lane}")
    
    # 为左车道生成密集的生成点（拥挤车辆）
    for i in range(max_vehicles_per_lane):
        x = STREET_START.x - (i * vehicle_unit_length) - random.uniform(0, 1)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),  # 左侧车道
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('LEFT', spawn_point))
    
    # 为右车道生成少量普通车辆，仅在后方区域
    right_lane_vehicles = 3  # 右侧车道只在后方放置少量车辆
    for i in range(right_lane_vehicles):
        # 在后方区域生成（x值较小）
        x = STREET_END.x + (i * (VEHICLE_LENGTH + 2)) + random.uniform(0, 1)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),  # 右侧车道后方
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('RIGHT', spawn_point))
    
    # 打乱顺序
    random.shuffle(spawn_points)
    
    print(f"生成了 {len(spawn_points)} 个普通车辆生成点 (大部分在左侧车道，右侧车道仅在后方)")
    return spawn_points

def spawn_parked_vehicles(world):
    """在指定位置(x=15.39, y=9.76)沿着道路方向生成违停车辆并保存坐标到文件"""
    parked_vehicles = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')

    # 只允许以下类型的小汽车
    allowed_types = [
        'vehicle.audi.a2',
        'vehicle.audi.etron',
        'vehicle.audi.tt',
        'vehicle.bmw.grandtourer',
        'vehicle.chevrolet.impala',
        'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020',
        'vehicle.ford.crown',
        'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon',
        'vehicle.lincoln.mkz_2020',
        'vehicle.mercedes.coupe_2020',
        'vehicle.micro.microlino',
        'vehicle.mini.cooperst',
        'vehicle.mustang.mustang',
        'vehicle.nissan.micra',
        'vehicle.nissan.patrol_2021',
        'vehicle.seat.leon',
        'vehicle.tesla.model3',
        'vehicle.toyota.prius',
        'vehicle.volkswagen.t2'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 获取计算好的停车位位置
    parking_spots = StreetMonitor.calculate_parking_positions()
    
    # 创建输出目录
    os.makedirs('output', exist_ok=True)
    
    # 创建违停车辆坐标文件
    coordinates_file = open('output/illegal_parking_coordinates.txt', 'w')
    coordinates_file.write("vehicle_id,x,y,z,yaw\n")  # 添加标题行

    for i, spot in enumerate(parking_spots):
        # 优选轿车类型
        sedan_blueprints = [bp for bp in vehicle_blueprints if 'sedan' in bp.id or 'audi' in bp.id]
        if not sedan_blueprints:
            sedan_blueprints = vehicle_blueprints

        blueprint = random.choice(sedan_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # 尝试生成车辆
        vehicle = world.try_spawn_actor(blueprint, spot)
        if vehicle is not None:
            parked_vehicles.append(vehicle)
            vehicle.set_autopilot(False)
            vehicle.set_simulate_physics(False)  # 禁用物理模拟

            print(f"生成了第 {i + 1} 辆违停车辆，位置: "
                  f"({spot.location.x:.2f}, {spot.location.y:.2f}) "
                  f"朝向: {spot.rotation.yaw:.1f}°")
        else:
            print(f"在位置 ({spot.location.x:.2f}, {spot.location.y:.2f}) 生成车辆失败")
    
    # 等待一帧让车辆位置更新
    if parked_vehicles:
        world.tick()
    
    # 获取所有车辆的实际位置并保存
    for vehicle in parked_vehicles:
        if vehicle.is_alive:
            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            
            coordinates_file.write(
                f"{vehicle.id},"
                f"{location.x:.4f},"
                f"{location.y:.4f},"
                f"{location.z:.4f},"
                f"{rotation.yaw:.2f}\n"
            )
    
    # 关闭坐标文件
    coordinates_file.close()
    print(f"已将违停车辆坐标保存到 output/illegal_parking_coordinates.txt")

    print(f"成功生成了 {len(parked_vehicles)} 辆违停车辆")
    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
    """在指定道路上生成运动车辆（感知车辆和普通车辆）
    主要逻辑：感知车辆优先在右侧车道，普通车辆主要在左侧车道，右侧车道普通车辆只在后方少量放置
    """
    moving_vehicles = []
    camera_managers = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')

    # 只允许小汽车类型
    allowed_types = [
        'vehicle.audi.a2',
        'vehicle.audi.etron',
        'vehicle.audi.tt',
        'vehicle.bmw.grandtourer',
        'vehicle.chevrolet.impala',
        'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020',
        'vehicle.ford.crown',
        'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon',
        'vehicle.lincoln.mkz_2020',
        'vehicle.mercedes.coupe_2020',
        'vehicle.micro.microlino',
        'vehicle.mini.cooperst',
        'vehicle.mustang.mustang',
        'vehicle.nissan.micra',
        'vehicle.nissan.patrol_2021',
        'vehicle.seat.leon',
        'vehicle.tesla.model3',
        'vehicle.toyota.prius',
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 生成感知车辆的优化生成点（优先右侧车道）
    sensor_spawn_points = generate_sensor_spawn_points()
    
    # 生成感知车辆 - 优先右侧车道，前方、中间、后方分布
    sensor_vehicles_spawned = 0
    for spawn_point in sensor_spawn_points:
        if sensor_vehicles_spawned >= NUM_SENSOR_VEHICLES:
            break
            
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 应用油门控制确保车辆移动
            vehicle.apply_control(carla.VehicleControl(
                throttle=0.7,  # 70%油门，确保较快速度
                steer=0.0      # 直行
            ))
            
            # 交通管理器设置
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)  # 正常速度
            traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)  # 增加安全距离到3米，确保前方车辆较远
            
            # 严格禁用变道行为
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            
            # 根据车道设置保持车道规则
            if spawn_point.location.y > 15.2:  # 左车道
                traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
            else:  # 右车道
                traffic_manager.keep_right_rule_percentage(vehicle, 100)  # 保持右车道
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            sensor_vehicles_spawned += 1
            
            # 判断车辆在哪个区域和车道
            x_pos = spawn_point.location.x
            y_pos = spawn_point.location.y
            lane = "左侧" if y_pos > 15.2 else "右侧"
            
            if x_pos >= 50:
                region = "前方"
            elif x_pos >= 20:
                region = "中间" 
            else:
                region = "后方"
                
            print(f"生成了感知车辆 {vehicle.id} 在 {lane} 车道, {region} 区域，位置 x={x_pos:.1f}, y={y_pos:.1f}")
        else:
            print(f"在位置 ({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f}) 生成感知车辆失败")

    # 生成普通车辆的生成点（大部分在左侧车道，右侧车道只在后方）
    normal_spawn_points = generate_normal_spawn_points()
    
    # 分离左右车道的生成点
    right_lane_points = [point for lane, point in normal_spawn_points if lane == 'RIGHT']
    left_lane_points = [point for lane, point in normal_spawn_points if lane == 'LEFT']
    
    print(f"可用普通车辆生成点 - 右侧: {len(right_lane_points)}, 左侧: {len(left_lane_points)}")

    # 生成普通车辆
    normal_vehicles_spawned = 0
    right_lane_vehicles = min(3, NUM_NORMAL_VEHICLES // 4)  # 右侧车道最多3辆普通车辆
    left_lane_vehicles = NUM_NORMAL_VEHICLES - right_lane_vehicles  # 其余在左车道
    
    # 在右车道生成少量车辆，只在后方
    right_spawned = 0
    for spawn_point in right_lane_points:
        if right_spawned >= right_lane_vehicles:
            break
            
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 应用油门控制
            vehicle.apply_control(carla.VehicleControl(
                throttle=0.3,  # 30%油门（较慢），确保不会追上感知车辆
                steer=0.0
            ))
            
            # 交通管理器设置
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)  # 2.0米安全距离
            
            # 严格禁用变道
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 100)  # 保持右车道
            
            right_spawned += 1
            normal_vehicles_spawned += 1

    # 在左车道生成剩余普通车辆（大部分）
    left_spawned = 0
    for spawn_point in left_lane_points:
        if left_spawned >= left_lane_vehicles:
            break
            
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 应用油门控制
            vehicle.apply_control(carla.VehicleControl(
                throttle=0.5,  # 50%油门（中等速度）
                steer=0.0
            ))
            
            # 交通管理器设置
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 1.5)  # 1.5米安全距离
            
            # 严格禁用变道
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
            
            left_spawned += 1
            normal_vehicles_spawned += 1

    print(f"\n成功生成了 {len(moving_vehicles)} 辆运动车辆:")
    print(f"  - {sensor_vehicles_spawned} 辆感知车辆 (大部分在右侧车道，分布均匀)")
    print(f"  - {right_spawned} 辆普通车辆 (右侧车道 - 仅后方)")
    print(f"  - {left_spawned} 辆普通车辆 (左侧车道 - 拥挤)")
    print(f"  - 总计: {sensor_vehicles_spawned + normal_vehicles_spawned}")
    
    return moving_vehicles, camera_managers

def main():
    # 初始化变量
    parked_vehicles = []
    moving_vehicles = []
    camera_managers = []
    
    try:
        # 创建输出目录
        os.makedirs('output/camera_images', exist_ok=True)

        # 连接CARLA服务端
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()

        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 固定时间步长0.05秒
        world.apply_settings(settings)

        # 设置天气（雨天，增加拥挤感和真实感）
        weather = carla.WeatherParameters(
            cloudiness=80,          # 高云量
            precipitation=800,       # 高降水量
            precipitation_deposits=80,  # 地面湿润度
            wetness=30,             # 物体表面湿润度
            sun_altitude_angle=15,  # 较低太阳角度
            fog_density=80,         # 添加雾气效果
            fog_distance=100,       # 雾气可见距离
            wind_intensity=20       # 添加风的效果
        )
        world.set_weather(weather)
        
        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(0.8)  # 全局安全距离0.8米
        traffic_manager.set_hybrid_physics_mode(True)  # 启用混合物理模式提高性能
        traffic_manager.set_random_device_seed(42)  # 固定随机种子

        # 生成违停车辆（9辆）
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(40):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（在指定道路上）
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        last_tick_time = time.time()
        last_fps_check = time.time()
        frame_count = 0
        
        print(f"\n仿真开始，道路范围从 x={STREET_START.x} 到 x={STREET_END.x}")
        print(f"左侧车道中心: y={LEFT_LANE_Y:.2f}, 右侧车道中心: y={RIGHT_LANE_Y:.2f}")
        print(f"感知车辆 ({NUM_SENSOR_VEHICLES}) 分布在右侧车道的前方、中间和后方区域")
        print(f"普通车辆 ({NUM_NORMAL_VEHICLES}) 大部分在左侧车道，右侧车道仅在后方")
        
        # 记录拥堵开始时间
        congestion_start_time = time.time()
        congestion_duration = 0
        is_congested = False
        
        # 统计变量
        total_recordings_started = 0
        active_recordings = 0
        
        while time.time() - start_time < SIMULATION_TIME:
            # 控制帧率 - 确保每帧间隔接近0.05秒
            current_time = time.time()
            elapsed = current_time - last_tick_time
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)
            
            # 记录本次tick时间
            last_tick_time = time.time()
            frame_count += 1
            
            # 每100帧检查一次FPS
            if frame_count % 100 == 0:
                fps = 100 / (time.time() - last_fps_check)
                print(f"当前FPS: {fps:.1f}")
                last_fps_check = time.time()
            
            # 推进仿真
            world.tick()
            
            # 每10秒检查一次交通状态（降低频率）
            if time.time() - congestion_start_time > 10:
                congestion_start_time = time.time()
                
                # 分别计算左右车道的平均速度
                left_lane_speed = 0
                right_lane_speed = 0
                left_count = 0
                right_count = 0
                
                # 统计当前活跃的记录
                current_active_recordings = 0
                sensor_positions = []
                
                for vehicle in moving_vehicles:
                    if vehicle.is_alive:
                        velocity = vehicle.get_velocity()
                        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                        location = vehicle.get_location()
                        
                        # 根据车道中心线判断车道
                        if location.y > STREET_START.y:  # 左车道
                            left_lane_speed += speed
                            left_count += 1
                        else:  # 右车道
                            right_lane_speed += speed
                            right_count += 1
                
                # 检查感知车辆的记录状态
                for manager in camera_managers:
                    if manager.vehicle.is_alive:
                        vehicle_location = manager.vehicle.get_location()
                        on_street = StreetMonitor.is_on_street(vehicle_location)
                        
                        if on_street and manager.recorder.is_recording:
                            current_active_recordings += 1
                            
                        # 记录感知车辆位置用于分析
                        if on_street:
                            region = "未知"
                            x_pos = vehicle_location.x
                            if x_pos >= 50:
                                region = "前方"
                            elif x_pos >= 20:
                                region = "中间"
                            else:
                                region = "后方"
                            lane = "左侧" if vehicle_location.y > 15.2 else "右侧"
                            sensor_positions.append((manager.vehicle.id, x_pos, region, lane))
                
                if left_count > 0 and right_count > 0:
                    avg_left_speed = 3.6 * left_lane_speed / left_count
                    avg_right_speed = 3.6 * right_lane_speed / right_count
                    
                    print(f"\n=== 交通状态 ===")
                    print(f"车道速度 - 左侧: {avg_left_speed:.1f} km/h ({left_count} 辆车), "
                          f"右侧: {avg_right_speed:.1f} km/h ({right_count} 辆车)")
                    print(f"活跃记录: {current_active_recordings}/{len(camera_managers)}")
                    
                    # 显示感知车辆分布
                    front_sensors = len([p for p in sensor_positions if p[2] == "前方"])
                    middle_sensors = len([p for p in sensor_positions if p[2] == "中间"])
                    rear_sensors = len([p for p in sensor_positions if p[2] == "后方"])
                    left_sensors = len([p for p in sensor_positions if p[3] == "左侧"])
                    right_sensors = len([p for p in sensor_positions if p[3] == "右侧"])
                    
                    print(f"道路上感知车辆分布:")
                    print(f"  - 前方: {front_sensors} (左侧: {len([p for p in sensor_positions if p[2]=='前方' and p[3]=='左侧'])}, 右侧: {len([p for p in sensor_positions if p[2]=='前方' and p[3]=='右侧'])})")
                    print(f"  - 中间: {middle_sensors} (左侧: {len([p for p in sensor_positions if p[2]=='中间' and p[3]=='左侧'])}, 右侧: {len([p for p in sensor_positions if p[2]=='中间' and p[3]=='右侧'])})")
                    print(f"  - 后方: {rear_sensors} (左侧: {len([p for p in sensor_positions if p[2]=='后方' and p[3]=='左侧'])}, 右侧: {len([p for p in sensor_positions if p[2]=='后方' and p[3]=='右侧'])})")
                    print(f"  - 总计 左侧: {left_sensors}, 右侧: {right_sensors}")
                    
                    # 如果右车道平均速度低于15km/h，认为拥堵
                    if avg_right_speed < 15 and not is_congested:
                        print(f"🚨 右侧车道拥堵检测到! 速度: {avg_right_speed:.1f} km/h")
                        is_congested = True
                    elif avg_right_speed >= 15 and is_congested:
                        print(f"✅ 右侧车道拥堵缓解. 速度: {avg_right_speed:.1f} km/h")
                        is_congested = False
                        
                    # 更新拥堵持续时间
                    if is_congested:
                        congestion_duration += 10
                        
                    print("=" * 50)

        print("\n🏁 仿真结束")
        print(f"📊 右侧车道总拥堵持续时间: {congestion_duration} 秒")
        print(f"📹 总记录开始次数: {total_recordings_started}")

    except KeyboardInterrupt:
        print("\n⏹️ 仿真被用户停止")
    except Exception as e:
        print(f"❌ 发生错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理阶段
        print("🧹 正在清理...")

        # 停止所有记录器
        for manager in camera_managers:
            try:
                if manager.recorder.is_recording:
                    manager.recorder.stop_recording()
                    print(f"停止了车辆 {manager.vehicle.id} 的记录")
            except Exception as e:
                print(f"停止记录器时出错: {e}")

        # 恢复异步模式
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
            print("恢复了异步模式")
        except:
            pass

        # 销毁所有摄像头
        for manager in camera_managers:
            try:
                if manager.rgb_sensor and manager.rgb_sensor.is_alive:
                    manager.rgb_sensor.destroy()
            except Exception as e:
                print(f"销毁传感器时出错: {e}")

        # 销毁所有车辆
        actor_list = parked_vehicles + moving_vehicles
        destroyed_count = 0
        for actor in actor_list:
            try:
                if actor.is_alive:
                    actor.destroy()
                    destroyed_count += 1
            except Exception as e:
                print(f"销毁车辆时出错: {e}")
        
        print(f"销毁了 {destroyed_count} 辆车")
        time.sleep(1.0)  # 确保资源释放
        print("✅ 清理完成")

if __name__ == '__main__':
    main()

