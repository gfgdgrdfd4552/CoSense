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

# 修改后的常量定义
NUM_PARKED_VEHICLES = 20     # 违停车辆数量
NUM_SENSOR_VEHICLES = 20    # 感知车辆数量增加到20辆
NUM_NORMAL_VEHICLES = 0     # 普通车辆数量
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 640           # 摄像头图像宽度
IMAGE_HEIGHT = 480          # 摄像头图像高度
LANE_WIDTH = 3.5            # 单车道宽度(米)
FPS = 30                    # 视频帧率
EDGE_OFFSET = 1.5           # 基础道路边缘偏移(米)
LANE_OFFSET_MULTIPLIER = 1.0 # 额外偏移车道数 (1.0=一个标准车道宽度)
DEBUG_MODE = False          # 调试可视化开关

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=-10, y=214, z=0)  # 道路起点
STREET_END = carla.Location(x=-60, y=365, z=0)    # 道路终点
VEHICLE_LENGTH = 4.5        # 平均车长(米)
VEHICLE_WIDTH = 2.0         # 平均车宽(米)
VEHICLE_SPACING = 10.0      # 车辆间距(米)

# 违停起始点（用户自定义坐标）
ILLEGAL_PARKING_POINT = carla.Location(x=-17.88, y=214.17, z=3.21)

# 车道中心线定义（左侧四条车道）
LEFT1_LANE_X = -11 - LANE_WIDTH * 1.5   # 最左侧车道
LEFT2_LANE_X = -11 - LANE_WIDTH * 0.5   # 左二车道
LEFT3_LANE_X = -11 + LANE_WIDTH * 0.5   # 左三车道
LEFT4_LANE_X = -11 + LANE_WIDTH * 1.5   # 左四车道

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
    def calculate_parking_positions(world):
        """沿道路边缘生成停车位，叠加车道偏移"""
        parking_positions = []
        carla_map = world.get_map()
        
        # 获取起始点道路信息
        start_waypoint = carla_map.get_waypoint(
            ILLEGAL_PARKING_POINT,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
       

        current_waypoint = start_waypoint
        
        for i in range(NUM_PARKED_VEHICLES):
            # 1. 计算道路方向向量
            yaw = current_waypoint.transform.rotation.yaw
            yaw_rad = math.radians(yaw)
            
            # 2. 计算基础边缘偏移（垂直于道路方向）
            edge_vector = carla.Vector3D(
                x = EDGE_OFFSET * math.sin(yaw_rad),
                y = -EDGE_OFFSET * math.cos(yaw_rad),
                z = 0
            )
            
            # 3. 计算车道宽度偏移（关键修改：叠加一个车道宽度）
            lane_width = current_waypoint.lane_width
            lane_offset = carla.Vector3D(
                x = lane_width * LANE_OFFSET_MULTIPLIER * math.sin(yaw_rad),
                y = -lane_width * LANE_OFFSET_MULTIPLIER * math.cos(yaw_rad),
                z = 0
            )
            
            # 4. 合并偏移量
            total_offset = edge_vector - lane_offset*1.25
            
            # 5. 计算生成位置
            spawn_location = current_waypoint.transform.location + total_offset
            spawn_location.z += 0.5  # Z轴缓冲防碰撞
            
            # 存储位置和朝向
            parking_positions.append(carla.Transform(
                spawn_location, 
                current_waypoint.transform.rotation  # 使用道路方向
            ))
            
            # 调试可视化
            if DEBUG_MODE:
                # 标记生成点（蓝色：基础边缘，红色：叠加车道偏移后）
                world.debug.draw_point(
                    current_waypoint.transform.location + edge_vector,
                    size=0.1, 
                    color=carla.Color(0, 0, 255),
                    life_time=SIMULATION_TIME
                )
                world.debug.draw_point(
                    spawn_location, 
                    size=0.1, 
                    color=carla.Color(255, 0, 0),
                    life_time=SIMULATION_TIME
                )
            
            # 获取下一个路径点（沿道路方向）
            next_waypoints = current_waypoint.next(VEHICLE_SPACING)
            current_waypoint = next_waypoints[0] if next_waypoints else current_waypoint
        
        return parking_positions

    @staticmethod
    def is_on_street(location):
        """检查位置是否在目标道路的X轴范围内且Y在25~40之间"""
        # 获取道路起点和终点的X坐标
        min_x = min(STREET_START.x, STREET_END.x)
        max_x = max(STREET_START.x, STREET_END.x)
    
        # 检查车辆坐标是否在指定范围内
        return (min_x <= location.x <= max_x) and (214 <= location.y <= 365)

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
        self.rgb_sensor.listen(lambda image: self._parse_rbg_image(weak_self, image))

    @staticmethod
    def _parse_rbg_image(weak_self, image):
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
    """为感知车辆生成优化的生成点（分布在左侧四条车道）"""
    sensor_spawn_points = []
    
    # 定义左侧四条车道
    lanes = [
        ('LEFT1', LEFT1_LANE_X),
        ('LEFT2', LEFT2_LANE_X),
        ('LEFT3', LEFT3_LANE_X),
        ('LEFT4', LEFT4_LANE_X)
    ]
    
    # 每车道生成5辆感知车辆（共20辆）
    vehicles_per_lane = NUM_SENSOR_VEHICLES // len(lanes)
    
    for lane_name, lane_x in lanes:
        for i in range(vehicles_per_lane):
            # 沿道路方向（Y轴）均匀分布
            y = STREET_START.y - i * SENSOR_VEHICLE_SPACING - random.uniform(0, 3)
            spawn_point = carla.Transform(
                carla.Location(x=lane_x, y=y, z=3),
                carla.Rotation(yaw=180)  # 朝向与道路方向一致
            )
            sensor_spawn_points.append(spawn_point)
            print(f"Sensor spawn point ({lane_name}): x={lane_x:.2f}, y={y:.2f}")
    
    return sensor_spawn_points

def generate_normal_spawn_points():
    """为普通车辆生成密集的生成点（集中在左侧四条车道）"""
    spawn_points = []
    
    # 定义左侧四条车道
    lanes = [
        ('LEFT1', LEFT1_LANE_X),
        ('LEFT2', LEFT2_LANE_X),
        ('LEFT3', LEFT3_LANE_X),
        ('LEFT4', LEFT4_LANE_X)
    ]
    
    # 每车道生成5辆普通车辆（共20辆）
    vehicles_per_lane = NUM_NORMAL_VEHICLES // len(lanes)
    
    for lane_name, lane_x in lanes:
        for i in range(vehicles_per_lane):
            y = STREET_START.y - i * SENSOR_VEHICLE_SPACING - random.uniform(0, 3)
            spawn_point = carla.Transform(
                carla.Location(x=lane_x, y=y, z=3),
                carla.Rotation(yaw=180)
            )
            spawn_points.append(spawn_point)
            print(f"Normal spawn point ({lane_name}): x={lane_x:.2f}, y={y:.2f}")
    
    return spawn_points

def spawn_parked_vehicles(world):
    """在计算位置生成违停车辆"""
    parked_vehicles = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # 过滤可用车型（轿车类）
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.etron', 
        'vehicle.tesla.model3', 'vehicle.toyota.prius',
        'vehicle.chevrolet.impala', 'vehicle.ford.mustang'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]
    
    # 获取停车位
    parking_spots = StreetMonitor.calculate_parking_positions(world)
    # 创建输出目录
    os.makedirs('output', exist_ok=True)
    with open('output/parking_coordinates.txt', 'w') as f:
        f.write("vehicle_id,x,y,z,yaw,lane_width\n")
    
    # 生成车辆
    for i, spot in enumerate(parking_spots):
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            colors = blueprint.get_attribute('color').recommended_values
            blueprint.set_attribute('color', random.choice(colors))
        
        # 三级高度防碰撞生成
        vehicle = None
        for z_offset in [0.0, 0.3, 0.6]:
            adjusted_spot = carla.Transform(
                carla.Location(spot.location.x, spot.location.y, spot.location.z + z_offset),
                spot.rotation
            )
            vehicle = world.try_spawn_actor(blueprint, adjusted_spot)
            if vehicle: 
                break
        
        if vehicle:
            # ==== 防漂移核心设置 ====
            vehicle.set_simulate_physics(True)      # 禁用物理引擎
            vehicle.apply_control(carla.VehicleControl(hand_brake=True))  # 强制手刹
            
            # 记录坐标（含车道宽度）
            transform = vehicle.get_transform()
            waypoint = world.get_map().get_waypoint(transform.location)
            lane_width = waypoint.lane_width if waypoint else 0.0
            with open('output/parking_coordinates.txt', 'a') as f:
                f.write(f"{vehicle.id},"
                        f"{transform.location.x:.4f},"
                        f"{transform.location.y:.4f},"
                        f"{transform.location.z:.4f},"
                        f"{transform.rotation.yaw:.2f},"
                        f"{lane_width:.2f}\n")
            
            # 可视化车辆朝向
            if DEBUG_MODE:
                arrow_start = transform.location
                arrow_end = arrow_start + carla.Location(
                    x=3 * math.cos(math.radians(transform.rotation.yaw)),
                    y=3 * math.sin(math.radians(transform.rotation.yaw)),
                    z=0
                )
                world.debug.draw_arrow(
                    arrow_start, arrow_end,
                    thickness=0.05,
                    arrow_size=0.1,
                    color=carla.Color(0, 255, 0),
                    life_time=SIMULATION_TIME
                )
            
            parked_vehicles.append(vehicle)
            print(f"✅ 生成车辆 {i+1}/{len(parking_spots)} 位置: ({spot.location.x:.2f}, {spot.location.y:.2f})")
        else:
            print(f"❌ 生成失败: ({spot.location.x:.2f}, {spot.location.y:.2f})")
    
    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
    """在指定道路上生成运动车辆（感知车辆和普通车辆）"""
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

    # 生成感知车辆的优化生成点（分布在左侧四条车道）
    sensor_spawn_points = generate_sensor_spawn_points()
    
    # 生成感知车辆
    sensor_vehicles_spawned = 0
    lanes = [
        ('LEFT1', LEFT1_LANE_X),
        ('LEFT2', LEFT2_LANE_X),
        ('LEFT3', LEFT3_LANE_X),
        ('LEFT4', LEFT4_LANE_X)
    ]
    
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
            traffic_manager.ignore_lights_percentage(vehicle, 100)  # 忽略红绿灯
            # 应用油门控制确保车辆移动
            vehicle.apply_control(carla.VehicleControl(
                throttle=0.3,  # 70%油门
                steer=0.0      # 直行
            ))
            
            # 交通管理器设置
            traffic_manager.distance_to_leading_vehicle(vehicle, 5.5)  # 1.5米安全距离
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 30)
            # 严格禁用变道行为
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            
            # 根据车道索引设置保持规则（左侧车道禁止向右变道）
            # 确定车道索引
            lane_x = spawn_point.location.x
            lane_name = None
            for name, x_val in lanes:
                if abs(x_val - lane_x) < 0.5:  # 允许微小误差
                    lane_name = name
                    break
            
            if lane_name:
                lane_index = lanes.index((lane_name, lane_x))
                if lane_index < 2:  # 左侧两条车道
                    traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
                else:  # 右侧两条车道
                    traffic_manager.keep_right_rule_percentage(vehicle, 50)  # 50%概率保持右车道
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            sensor_vehicles_spawned += 1
            
            # 判断车辆在哪个区域
            x_pos = spawn_point.location.x
            y_pos = spawn_point.location.y
            
            if y_pos >= 300:
                region = "FRONT"
            elif y_pos >= 250:
                region = "MIDDLE" 
            else:
                region = "REAR"
                
            print(f"Spawned sensor vehicle {vehicle.id} in {lane_name} lane, {region} region at x={x_pos:.1f}, y={y_pos:.1f}")
        else:
            print(f"Failed to spawn sensor vehicle at position ({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f})")

    # 生成普通车辆
    normal_spawn_points = generate_normal_spawn_points()
    normal_vehicles_spawned = 0
    for spawn_point in normal_spawn_points:
        if normal_vehicles_spawned >= NUM_NORMAL_VEHICLES:
            break
            
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            traffic_manager.ignore_lights_percentage(vehicle, 100)  # 忽略红绿灯
            # 应用油门控制确保车辆移动
            vehicle.apply_control(carla.VehicleControl(
                throttle=0.3,  # 70%油门
                steer=0.0      # 直行
            ))
            
            # 交通管理器设置
            traffic_manager.distance_to_leading_vehicle(vehicle, 5.5)  # 1.5米安全距离
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 30)
            # 严格禁用变道行为
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            
            # 根据车道索引设置保持规则（左侧车道禁止向右变道）
            # 确定车道索引
            lane_x = spawn_point.location.x
            lane_name = None
            for name, x_val in lanes:
                if abs(x_val - lane_x) < 0.5:  # 允许微小误差
                    lane_name = name
                    break
            
            if lane_name:
                lane_index = lanes.index((lane_name, lane_x))
                if lane_index < 2:  # 左侧两条车道
                    traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
                else:  # 右侧两条车道
                    traffic_manager.keep_right_rule_percentage(vehicle, 50)  # 50%概率保持右车道
            
            normal_vehicles_spawned += 1
            
            # 判断车辆在哪个区域
            x_pos = spawn_point.location.x
            y_pos = spawn_point.location.y
            
            if y_pos >= 300:
                region = "FRONT"
            elif y_pos >= 250:
                region = "MIDDLE" 
            else:
                region = "REAR"
                
            print(f"Spawned normal vehicle {vehicle.id} in {lane_name} lane, {region} region at x={x_pos:.1f}, y={y_pos:.1f}")
        else:
            print(f"Failed to spawn normal vehicle at position ({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f})")
    
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

        # 设置天气
        weather = carla.WeatherParameters(
            cloudiness=50,
            precipitation=0,
            precipitation_deposits=0,
            wetness=0,
            sun_altitude_angle=90,
            fog_density=0,
            wind_intensity=0
        )
        world.set_weather(weather)
        
        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(5.8)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_random_device_seed(42)

        # 生成违停车辆
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(40):
            world.tick()

        # 生成运动车辆
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)
        
        # 主循环
        start_time = time.time()
        last_tick_time = time.time()
        last_fps_check = time.time()
        frame_count = 0
        
        print(f"\nSimulation started on road from y={STREET_START.y} to y={STREET_END.y}")
        print(f"Lanes defined at: {LEFT1_LANE_X:.2f}, {LEFT2_LANE_X:.2f}, {LEFT3_LANE_X:.2f}, {LEFT4_LANE_X:.2f}")
        
        # 记录拥堵开始时间
        congestion_start_time = time.time()
        congestion_duration = 0
        is_congested = False
        
        # 统计变量
        total_recordings_started = 0
        active_recordings = 0
        
        while time.time() - start_time < SIMULATION_TIME:
            # 控制帧率
            current_time = time.time()
            elapsed = current_time - last_tick_time
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)
            
            last_tick_time = time.time()
            frame_count += 1
            
            # 每100帧检查一次FPS
            if frame_count % 100 == 0:
                fps = 100 / (time.time() - last_fps_check)
                print(f"Current FPS: {fps:.1f}")
                last_fps_check = time.time()
            
            # 推进仿真
            world.tick()
            
            # 每10秒检查一次交通状态
            if time.time() - congestion_start_time > 10:
                congestion_start_time = time.time()
                
                # 计算各车道平均速度
                lane_speeds = {lane: [] for lane in ['LEFT1', 'LEFT2', 'LEFT3', 'LEFT4']}
                
                # 统计当前活跃的记录
                current_active_recordings = 0
                sensor_positions = []
                
                for vehicle in moving_vehicles:
                    if vehicle.is_alive:
                        velocity = vehicle.get_velocity()
                        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                        location = vehicle.get_location()
                        
                        # 确定车道
                        for lane_name, lane_x in [('LEFT1', LEFT1_LANE_X), ('LEFT2', LEFT2_LANE_X),
                                                 ('LEFT3', LEFT3_LANE_X), ('LEFT4', LEFT4_LANE_X)]:
                            if abs(location.x - lane_x) < LANE_WIDTH/2:
                                lane_speeds[lane_name].append(speed)
                                break
                
                # 计算并打印车道速度
                print("\n=== Traffic Status ===")
                for lane, speeds in lane_speeds.items():
                    if speeds:
                        avg_speed = 3.6 * sum(speeds) / len(speeds)
                        print(f"{lane} lane: {avg_speed:.1f} km/h ({len(speeds)} vehicles)")
                    else:
                        print(f"{lane} lane: No vehicles")
                
                # 检查拥堵情况（以最左侧车道为例）
                if 'LEFT1' in lane_speeds and lane_speeds['LEFT1']:
                    avg_speed = 3.6 * sum(lane_speeds['LEFT1']) / len(lane_speeds['LEFT1'])
                    if avg_speed < 15 and not is_congested:
                        print(f"🚨 LEFT1 lane congestion detected! Speed: {avg_speed:.1f} km/h")
                        is_congested = True
                    elif avg_speed >= 15 and is_congested:
                        print(f"✅ LEFT1 lane congestion eased. Speed: {avg_speed:.1f} km/h")
                        is_congested = False
                
                # 更新拥堵持续时间
                if is_congested:
                    congestion_duration += 10
                    
                print("=" * 50)

        print("\n🏁 Simulation finished")
        print(f"📊 Total congestion duration: {congestion_duration} seconds")

    except KeyboardInterrupt:
        print("\n⏹️ Simulation stopped by user")
    except Exception as e:
        print(f"❌ Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理阶段
        print("🧹 Cleaning up...")

        # 停止所有记录器
        for manager in camera_managers:
            try:
                if manager.recorder.is_recording:
                    manager.recorder.stop_recording()
            except Exception as e:
                print(f"Error stopping recorder: {e}")

        # 恢复异步模式
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except:
            pass

        # 销毁所有摄像头
        for manager in camera_managers:
            try:
                if manager.rgb_sensor and manager.rgb_sensor.is_alive:
                    manager.rgb_sensor.destroy()
            except Exception as e:
                pass

        # 销毁所有车辆
        actor_list = parked_vehicles + moving_vehicles
        destroyed_count = 0
        for actor in actor_list:
            try:
                if actor.is_alive:
                    actor.destroy()
                    destroyed_count += 1
            except:
                pass
        
        print(f"Destroyed {destroyed_count} vehicles")
        print("✅ Cleanup completed")

if __name__ == '__main__':
    main()