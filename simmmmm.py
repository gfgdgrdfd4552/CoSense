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

# 常量定义优化
NUM_PARKED_VEHICLES = 5     # 违停车辆数量
NUM_SENSOR_VEHICLES = 10    # 感知车辆数量
NUM_NORMAL_VEHICLES = 20    # 普通车辆数量
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 640           # 摄像头图像宽度
IMAGE_HEIGHT = 480          # 摄像头图像高度
LANE_WIDTH = 3.5            # 单车道宽度(米)
FPS = 30                    # 视频帧率
MAX_VIDEO_QUEUE_SIZE = 60   # 视频队列最大容量（2秒）

# 道路坐标优化
STREET_START = carla.Location(x=80, y=15.2, z=0)
STREET_END = carla.Location(x=-10, y=15.2, z=0)
VEHICLE_LENGTH = 4.5        # 平均车长(米)
VEHICLE_WIDTH = 2.0         # 平均车宽(米)

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

# 车道中心线定义
LEFT_LANE_Y = 15.2 + LANE_WIDTH / 2   # 左车道中心线 y=17.95
RIGHT_LANE_Y = 15.2 - LANE_WIDTH / 2  # 右车道中心线 y=12.45

# 感知车辆间距（确保前方有车辆遮挡）
SENSOR_VEHICLE_SPACING = 5.0  # 感知车辆间距（米）

class VideoWriterThread(threading.Thread):
    """异步视频写入线程（支持RGB和深度视频）"""
    
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
        
        # 深度视频写入器
        depth_video_path = f'{self.save_dir}/depth_recording_{timestamp}.mp4'
        self.depth_writer = cv2.VideoWriter(depth_video_path, fourcc, FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))
        
        # 图像队列
        self.queue = queue.Queue(maxsize=MAX_VIDEO_QUEUE_SIZE)
        self.stop_event = threading.Event()
        self.daemon = True
        self.frame_count = 0
        
    def run(self):
        """线程主函数"""
        print(f"📹 Video writer started for vehicle {self.vehicle_id}")
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                # 从队列获取图像
                rgb_array, depth_vis = self.queue.get(timeout=1.0)
                
                # 写入RGB视频
                self.rgb_writer.write(rgb_array)
                
                # 写入深度视频（应用伪彩色映射）
                depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                self.depth_writer.write(depth_colored)
                
                self.queue.task_done()
                self.frame_count += 1
            except queue.Empty:
                continue
        
        # 释放资源
        self.rgb_writer.release()
        self.depth_writer.release()
        print(f"📹 Video writer for vehicle {self.vehicle_id} released, wrote {self.frame_count} frames")
    
    def add_frame(self, rgb_array, depth_vis):
        """添加帧到队列"""
        if not self.stop_event.is_set():
            try:
                # 如果队列已满，丢弃旧帧
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except queue.Empty:
                        pass
                
                # 存储RGB和深度图像
                self.queue.put_nowait((rgb_array.copy(), depth_vis.copy()))
            except Exception as e:
                print(f"⚠️ Error adding frame to video queue: {e}")

def setup_traffic_light(world):
    """在道路前方设置红绿灯并设为绿灯120秒"""
    try:
        # 获取所有红绿灯
        all_traffic_lights = world.get_actors().filter('traffic.traffic_light*')
        
        # 寻找最接近目标位置的红绿灯
        target_light = None
        min_distance = float('inf')
        
        for traffic_light in all_traffic_lights:
            light_location = traffic_light.get_location()
            distance = light_location.distance(carla.Location(x=90.0, y=5.0, z=5.0))
            
            # 寻找道路前方的红绿灯（x坐标大于80且距离最近）
            if light_location.x > 80 and distance < min_distance:
                min_distance = distance
                target_light = traffic_light
                
        if target_light:
            # 设置红绿灯状态为绿灯，持续120秒
            target_light.set_state(carla.TrafficLightState.Green)
            target_light.set_green_time(120.0)
            
            light_loc = target_light.get_location()
            print(f"✅ Traffic light set to GREEN for 120 seconds at location: x={light_loc.x:.1f}, y={light_loc.y:.1f}")
            return target_light
        else:
            print("⚠️ No suitable traffic light found near the target road section")
            return None
            
    except Exception as e:
        print(f"❌ Error setting up traffic light: {str(e)}")
        return None

class StreetMonitor:
    """道路监控器"""

    @staticmethod
    def calculate_parking_positions():
        """在指定位置(x=15.39, y=9.76)沿着道路方向生成5辆违停车辆"""
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
            offset = direction * (i * (VEHICLE_LENGTH + 1.0))
            
            parking_pos = carla.Location(
                x=ILLEGAL_PARKING_POINT.x + offset.x,
                y=ILLEGAL_PARKING_POINT.y + offset.y,
                z=0.1
            )
            
            parking_positions.append(
                carla.Transform(parking_pos, carla.Rotation(yaw=yaw)))
            
        return parking_positions

    @staticmethod
    def is_on_street(location):
        """检查位置是否在目标道路上"""
        # 简单边界检查（优化性能）
        if location.x < STREET_END.x or location.x > STREET_START.x:
            return False
        
        # 检查y坐标是否在道路范围内
        return abs(location.y - STREET_START.y) <= LANE_WIDTH * 1.5

class VehicleRecorder:
    """车辆数据记录器（优化版）"""
    
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.save_dir = f'output/camera_images/vehicle_{vehicle_id}'
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 初始化视频写入线程
        self.video_thread = VideoWriterThread(vehicle_id)
        self.video_thread.start()
        
        # 初始化数据文件
        self.data_file = open(f'{self.save_dir}/vehicle_data.txt', 'w')
        self.data_file.write("frame_number,relative_time(s),x,y,z,speed(km/h),avg_depth(m)\n")
        
        # 记录状态
        self.is_recording = False
        self.has_recorded = False
        self.start_time = None
        self.frame_count = 0
        self.data_count = 0
        self.last_record_time = 0
        
        # 深度数据缓存
        self.last_depth_data = None
        
    def record_frame(self, rgb_image, depth_image, vehicle):
        """记录视频帧和车辆数据"""
        if not self.is_recording:
            return
            
        # 帧计数器递增
        self.frame_count += 1
        
        # 转换RGB图像格式
        rgb_array = np.frombuffer(rgb_image.raw_data, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (rgb_image.height, rgb_image.width, 4))
        rgb_array = rgb_array[:, :, :3]  # 去除alpha通道
        rgb_array = rgb_array[:, :, ::-1]  # 从RGB转换为BGR
        
        # 处理深度图像
        depth_meters = self._convert_depth_image(depth_image)
        self.last_depth_data = depth_meters  # 缓存深度数据
        
        # 为视频生成优化的深度可视化
        depth_vis = self._create_depth_visualization(depth_meters)
        
        # 添加帧到视频线程队列
        self.video_thread.add_frame(rgb_array, depth_vis)
        
        # 控制数据记录频率（每秒5次）
        current_time = time.time()
        if current_time - self.last_record_time >= 0.2:
            self._record_vehicle_data(vehicle, depth_image)
            self.last_record_time = current_time
    
    def _convert_depth_image(self, depth_image):
        """优化深度图像转换逻辑"""
        # 直接使用CARLA的深度转换函数
        depth_array = np.frombuffer(depth_image.raw_data, dtype=np.float32)
        depth_array = np.reshape(depth_array, (depth_image.height, depth_image.width))
        return depth_array
    
    def _create_depth_visualization(self, depth_meters):
        """创建深度可视化图像"""
        # 限制深度范围在0-50米内，增强视觉效果
        depth_vis = np.clip(depth_meters, 0, 50)
        
        # 归一化到0-255范围
        depth_vis = (depth_vis / 50 * 255).astype(np.uint8)
        return depth_vis
    
    def _record_vehicle_data(self, vehicle, depth_image):
        """记录车辆位置和速度数据，包含深度信息"""
        try:
            location = vehicle.get_location()
            velocity = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2)  # 忽略z轴
            
            # 计算相对时间
            relative_time = self.frame_count / FPS
            
            # 计算平均深度
            avg_depth = -1.0
            if self.last_depth_data is not None:
                # 只计算图像中心区域的深度
                center_region = self.last_depth_data[
                    depth_image.height//4:3*depth_image.height//4,
                    depth_image.width//4:3*depth_image.width//4
                ]
                avg_depth = np.mean(center_region)
            
            self.data_count += 1
            self.data_file.write(
                f"{self.frame_count},"
                f"{relative_time:.3f},"
                f"{location.x:.3f},"
                f"{location.y:.3f},"
                f"{location.z:.3f},"
                f"{speed:.2f},"
                f"{avg_depth:.2f}\n"
            )
            self.data_file.flush()
        except Exception as e:
            print(f"⚠️ Error recording vehicle data: {e}")
    
    def start_recording(self):
        """开始记录"""
        if not self.has_recorded:
            self.is_recording = True
            self.has_recorded = True
            self.start_time = time.time()
            self.frame_count = 0
            self.data_count = 0
            self.last_record_time = time.time()
            print(f"⏺️ Started recording for vehicle {self.vehicle_id}")
    
    def stop_recording(self):
        """停止记录"""
        if self.is_recording:
            self.is_recording = False
            duration = (time.time() - self.start_time) if self.start_time else 0
            print(f"⏹️ Stopping recording for vehicle {self.vehicle_id}. Duration: {duration:.2f}s")
            
            # 停止视频线程
            self.video_thread.stop()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器（优化版）"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.rgb_sensor = None
        self.depth_sensor = None
        self.recorded_positions = set()
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        self.rgb_queue = Queue(maxsize=5)
        self.depth_queue = Queue(maxsize=5)
        
        # 记录状态控制
        self.was_on_street = False
        self.consecutive_off_street = 0
        self.max_off_street_frames = 30  # 最大允许离开道路的帧数（1秒）
        self.frame_counter = 0  # 用于控制检查频率
        
        # 初始化RGB摄像头
        rgb_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        rgb_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        rgb_bp.set_attribute('fov', '100')  # 视野100度

        # 初始化深度摄像头
        depth_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        depth_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        depth_bp.set_attribute('fov', '100')
        
        # 设置深度格式
        if depth_bp.has_attribute('pixel_format'):
            depth_bp.set_attribute('pixel_format', 'PF_Depth')
        elif depth_bp.has_attribute('format'):
            depth_bp.set_attribute('format', 'PF_Depth')

        # 安装位置
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        
        try:
            # 生成RGB摄像头
            self.rgb_sensor = self.world.spawn_actor(rgb_bp, transform, attach_to=self.vehicle)
            
            # 生成深度摄像头
            self.depth_sensor = self.world.spawn_actor(depth_bp, transform, attach_to=self.vehicle)

            # 设置图像回调函数
            weak_self = weakref.ref(self)
            self.rgb_sensor.listen(lambda image: self._parse_rgb_image(weak_self, image))
            self.depth_sensor.listen(lambda image: self._parse_depth_image(weak_self, image))
            print(f"📷 Cameras attached to vehicle {vehicle.id}")
        except Exception as e:
            print(f"❌ Failed to spawn cameras: {e}")

    @staticmethod
    def _parse_rgb_image(weak_self, image):
        """处理RGB图像"""
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return
            
        try:
            self.rgb_queue.put(image)
            self._process_images()
        except Exception as e:
            print(f"⚠️ Error processing RGB image: {e}")

    @staticmethod
    def _parse_depth_image(weak_self, image):
        """处理深度图像"""
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return
            
        try:
            self.depth_queue.put(image)
            self._process_images()
        except Exception as e:
            print(f"⚠️ Error processing depth image: {e}")

    def _process_images(self):
        """处理成对的RGB和深度图像"""
        # 只有当两个队列都有数据时才处理
        while not self.rgb_queue.empty() and not self.depth_queue.empty():
            rgb_image = self.rgb_queue.get()
            depth_image = self.depth_queue.get()
            
            # 获取当前车辆位置
            vehicle_location = self.vehicle.get_location()
            
            # 每5帧检查一次位置（优化性能）
            self.frame_counter += 1
            if self.frame_counter % 5 == 0:
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
                elif self.recorder.is_recording and self.consecutive_off_street >= self.max_off_street_frames:
                    # 连续离开道路超过阈值，停止记录
                    self.recorder.stop_recording()
                    print(f"🚫 Vehicle {self.vehicle.id} left street for too long, stopped recording")
            
            # 如果正在记录，处理帧
            if self.recorder.is_recording:
                self.recorder.record_frame(rgb_image, depth_image, self.vehicle)
                
            # 检查是否到达需要记录的x位置
            current_x = round(vehicle_location.x)
            target_positions = [80, 70, 60, 50, 40, 30, 20, 10, 0]
            for target_x in target_positions:
                if (abs(current_x - target_x) <= 0.5 and 
                    target_x not in self.recorded_positions):
                    self._save_image_and_metadata(rgb_image, depth_image, vehicle_location, target_x)
                    self.recorded_positions.add(target_x)
                    
            # 清理队列
            self.rgb_queue.task_done()
            self.depth_queue.task_done()

    def _save_image_and_metadata(self, rgb_image, depth_image, vehicle_location, target_x):
        """保存图片和元数据（包含深度信息）"""
        try:
            save_dir = f'output/camera_images/vehicle_{self.vehicle.id}'
            os.makedirs(save_dir, exist_ok=True)

            # 获取相机参数
            camera_transform = self.rgb_sensor.get_transform()
            metadata = {
                'timestamp': time.time(),
                'camera': {
                    'x': camera_transform.location.x,
                    'y': camera_transform.location.y,
                    'z': camera_transform.location.z,
                    'yaw': camera_transform.rotation.yaw,
                    'fov': 100,
                    'max_range': 30.0
                },
                'vehicle': {
                    'id': self.vehicle.id,
                    'x': vehicle_location.x,
                    'y': vehicle_location.y,
                    'z': vehicle_location.z,
                    'speed': 3.6 * math.sqrt(self.vehicle.get_velocity().x**2 + self.vehicle.get_velocity().y**2)
                }
            }

            # 保存元数据为JSON文件
            metadata_path = f'{save_dir}/x{target_x}_meta.json'
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            # 保存RGB图片
            rgb_image.save_to_disk(f'{save_dir}/x{target_x}_rgb.png')
            
            # 保存深度数据（转换为米）
            depth_meters = self.recorder._convert_depth_image(depth_image)
            np.save(f'{save_dir}/x{target_x}_depth.npy', depth_meters)
            
            print(f"💾 Saved data at x={target_x} for sensor vehicle {self.vehicle.id}")
        except Exception as e:
            print(f"⚠️ Error saving image and metadata: {e}")

def generate_sensor_spawn_points():
    """为感知车辆生成优化的生成点（分布在道路右侧车道）"""
    sensor_spawn_points = []
    
    # 将道路分为三个区域
    front_count = 4  # 前方区域
    middle_count = 4  # 中间区域
    rear_count = 2   # 后方区域
    
    # 前方区域：x=80到x=50
    for i in range(front_count):
        x = 80 - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
    
    # 中间区域：x=50到x=20
    for i in range(middle_count):
        x = 50 - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
    
    # 后方区域：x=20到x=-10
    for i in range(rear_count):
        x = 20 - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
    
    return sensor_spawn_points

def generate_normal_spawn_points():
    """为普通车辆生成密集的生成点（集中在左侧车道）"""
    spawn_points = []
    road_length = STREET_START.x - STREET_END.x  # 90米
    
    # 计算左车道最大车辆数（拥挤交通间距0.5米）
    max_left_vehicles = int(road_length / (VEHICLE_LENGTH + 0.5))
    
    # 为左车道生成密集的生成点
    for i in range(max_left_vehicles):
        x = STREET_START.x - (i * (VEHICLE_LENGTH + 0.5)) - random.uniform(0, 1)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        spawn_points.append(spawn_point)
    
    # 为右车道生成少量普通车辆（避免遮挡感知车辆）
    for i in range(5):  # 右车道最多5辆车
        x = STREET_START.x - (road_length * 0.8) - (i * (VEHICLE_LENGTH + 10))
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        spawn_points.append(spawn_point)
    
    return spawn_points

def spawn_parked_vehicles(world):
    """在指定位置生成5辆违停车辆"""
    parked_vehicles = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # 只允许小汽车类型
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.tt', 'vehicle.bmw.grandtourer',
        'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon', 'vehicle.mercedes.coupe_2020',
        'vehicle.mini.cooperst', 'vehicle.tesla.model3', 'vehicle.toyota.prius'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 获取停车位位置
    parking_spots = StreetMonitor.calculate_parking_positions()

    for i, spot in enumerate(parking_spots):
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # 尝试生成车辆
        try:
            vehicle = world.spawn_actor(blueprint, spot)
            if vehicle is not None:
                parked_vehicles.append(vehicle)
                vehicle.set_autopilot(False)
                vehicle.set_simulate_physics(False)
        except Exception as e:
            print(f"⚠️ Failed to spawn parked vehicle: {e}")

    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
    """在指定道路上生成运动车辆"""
    moving_vehicles = []
    camera_managers = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # 只允许小汽车类型
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.tt', 'vehicle.bmw.grandtourer',
        'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon', 'vehicle.mercedes.coupe_2020',
        'vehicle.mini.cooperst', 'vehicle.tesla.model3', 'vehicle.toyota.prius'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 生成感知车辆的优化生成点
    sensor_spawn_points = generate_sensor_spawn_points()
    
    # 生成感知车辆
    for spawn_point in sensor_spawn_points:
        if len(moving_vehicles) >= NUM_SENSOR_VEHICLES:
            break
            
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        try:
            vehicle = world.spawn_actor(blueprint, spawn_point)
            if vehicle is not None:
                moving_vehicles.append(vehicle)
                vehicle.set_autopilot(True)
                
                # 应用油门控制确保车辆移动
                vehicle.apply_control(carla.VehicleControl(throttle=0.7, steer=0.0))
                
                # 交通管理器设置
                traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
                traffic_manager.distance_to_leading_vehicle(vehicle, 1.5)
                traffic_manager.auto_lane_change(vehicle, False)
                traffic_manager.keep_right_rule_percentage(vehicle, 100)
                
                # 安装摄像头
                camera_managers.append(VehicleCameraManager(vehicle))
        except Exception as e:
            print(f"⚠️ Failed to spawn sensor vehicle: {e}")

    # 生成普通车辆的生成点
    normal_spawn_points = generate_normal_spawn_points()
    
    # 生成普通车辆
    for i, spawn_point in enumerate(normal_spawn_points):
        if len(moving_vehicles) >= NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES:
            break
            
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        try:
            vehicle = world.spawn_actor(blueprint, spawn_point)
            if vehicle is not None:
                moving_vehicles.append(vehicle)
                vehicle.set_autopilot(True)
                
                # 根据车道设置不同的交通行为
                if abs(spawn_point.location.y - LEFT_LANE_Y) < 1.0:  # 左车道
                    vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=0.0))
                    traffic_manager.distance_to_leading_vehicle(vehicle, 1.0)
                    traffic_manager.keep_right_rule_percentage(vehicle, 0)
                else:  # 右车道
                    vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))
                    traffic_manager.distance_to_leading_vehicle(vehicle, 3.0)
                    traffic_manager.keep_right_rule_percentage(vehicle, 100)
        except Exception as e:
            print(f"⚠️ Failed to spawn normal vehicle: {e}")

    return moving_vehicles, camera_managers

def main():
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
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # 设置天气
        weather = carla.WeatherParameters(cloudiness=80, precipitation=0, sun_altitude_angle=45)
        world.set_weather(weather)
        
        # 设置红绿灯
        setup_traffic_light(world)
        
        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_random_device_seed(42)

        # 生成违停车辆
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):
            world.tick()

        # 生成运动车辆
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        last_status_time = time.time()
        frame_count = 0
        
        print(f"\n🚦 Simulation started on road from x={STREET_START.x} to x={STREET_END.x}")
        
        while time.time() - start_time < SIMULATION_TIME:
            # 控制帧率
            current_time = time.time()
            
            # 每100帧显示一次状态
            frame_count += 1
            if frame_count % 100 == 0:
                print(f"⏱️ Frame: {frame_count}, Time: {current_time - start_time:.1f}s")
            
            # 每10秒显示交通状态
            if current_time - last_status_time > 10:
                last_status_time = current_time
                
                # 计算活跃的记录数量
                active_recordings = sum(1 for m in camera_managers 
                                       if m.recorder.is_recording and m.vehicle.is_alive)
                
                print(f"📊 Active recordings: {active_recordings}/{len(camera_managers)}")
            
            # 推进仿真
            world.tick()
            
        print("\n🏁 Simulation finished")

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
            except:
                pass

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
                if manager.depth_sensor and manager.depth_sensor.is_alive:
                    manager.depth_sensor.destroy()
            except:
                pass

        # 销毁所有车辆
        actor_list = parked_vehicles + moving_vehicles
        for actor in actor_list:
            try:
                if actor.is_alive:
                    actor.destroy()
            except:
                pass
        
        time.sleep(1.0)
        print("✅ Cleanup completed")

if __name__ == '__main__':
    main()