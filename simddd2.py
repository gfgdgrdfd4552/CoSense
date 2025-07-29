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
NUM_SENSOR_VEHICLES = 10    # 感知车辆数量
NUM_NORMAL_VEHICLES = 30    # 普通车辆数量
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 1920          # 摄像头图像宽度
IMAGE_HEIGHT = 1080         # 摄像头图像高度
LANE_WIDTH = 3.5            # 单车道宽度(米)
FPS = 20                    # 视频帧率
MAX_VIDEO_QUEUE_SIZE = 10   # 视频队列大小限制

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 7            # 道路总宽度
VEHICLE_LENGTH = 4.5        # 平均车长(米)
VEHICLE_WIDTH = 2.0         # 平均车宽(米)
VEHICLE_SPACING = 8.0       # 车辆间距

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

# 车道中心线定义
LEFT_LANE_Y = 15.2 + LANE_WIDTH / 2   # 左车道中心线 y=17.95
RIGHT_LANE_Y = 15.2 - LANE_WIDTH / 2  # 右车道中心线 y=12.45

class VideoWriterThread(threading.Thread):
    """异步视频写入线程（优化内存使用）"""
    
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
        self.queue = queue.Queue(maxsize=MAX_VIDEO_QUEUE_SIZE)
        self.stop_event = threading.Event()
        self.daemon = True  # 守护线程
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
        
        print(f"Video writer for vehicle {self.vehicle_id} released, wrote {self.frame_count} frames")
    
    def add_frame(self, rgb_array):
        """添加帧到队列（避免不必要的拷贝）"""
        if not self.stop_event.is_set() and not self.queue.full():
            try:
                # 直接使用原始数组避免拷贝
                self.queue.put_nowait(rgb_array)
            except queue.Full:
                pass  # 忽略队列满异常
    
    def stop(self):
        """停止线程并处理剩余帧"""
        self.stop_event.set()
        
        # 处理队列中剩余帧
        while not self.queue.empty():
            try:
                frame = self.queue.get_nowait()
                self.rgb_writer.write(frame)
                self.frame_count += 1
            except queue.Empty:
                break
        
        # 释放资源
        if self.rgb_writer.isOpened():
            self.rgb_writer.release()
        self.join(timeout=2.0)

class StreetMonitor:
    """道路监控器"""

    @staticmethod
    def calculate_parking_positions():
        """在指定位置沿着道路方向生成违停车辆"""
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
            offset = direction * (i * (VEHICLE_LENGTH + 1.2))
            
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
        return dist <= STREET_WIDTH / 2 and t > 0 and t < 1

class SmoothControl:
    """控制指令平滑处理类"""
    
    def __init__(self, vehicle, max_throttle_change=0.05, max_steer_change=0.01):
        self.vehicle = vehicle
        self.max_throttle_change = max_throttle_change
        self.max_steer_change = max_steer_change
        self.last_throttle = 0
        self.last_steer = 0
        self.last_brake = 0
        
    def apply_smooth_control(self, throttle, steer, brake=0):
        """应用平滑控制指令"""
        # 油门平滑
        smooth_throttle = self.last_throttle + max(
            -self.max_throttle_change, 
            min(throttle - self.last_throttle, self.max_throttle_change)
        )
        # 转向平滑
        smooth_steer = self.last_steer + max(
            -self.max_steer_change,
            min(steer - self.last_steer, self.max_steer_change)
        )
        # 刹车平滑
        smooth_brake = self.last_brake + max(
            -self.max_throttle_change,
            min(brake - self.last_brake, self.max_throttle_change)
        )
        
        # 应用控制
        self.vehicle.apply_control(carla.VehicleControl(
            throttle=smooth_throttle,
            steer=smooth_steer,
            brake=smooth_brake
        ))
        
        # 更新最后值
        self.last_throttle = smooth_throttle
        self.last_steer = smooth_steer
        self.last_brake = smooth_brake

class VehicleRecorder:
    """车辆数据记录器（优化内存使用）"""
    
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
        self.frame_count = 0
        self.data_count = 0
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
        """记录视频帧和车辆数据（优化内存使用）"""
        if not self.is_recording:
            return
            
        # 帧计数器递增
        self.frame_count += 1
        
        # 计算当前帧相对时间
        self.current_relative_time = self.frame_count / FPS
        
        # 优化图像转换：避免完整拷贝
        rgb_array = np.reshape(
            np.frombuffer(rgb_image.raw_data, dtype=np.uint8, 
                          count=rgb_image.height * rgb_image.width * 4),
            (rgb_image.height, rgb_image.width, 4)
        )
        rgb_array = rgb_array[:, :, :3]  # 去除alpha通道
        rgb_array = np.ascontiguousarray(rgb_array[:, :, ::-1])  # 原地操作避免额外拷贝
        
        # 添加帧到视频线程队列
        self.video_thread.add_frame(rgb_array)
        
        # 使用帧计数器控制数据记录频率（每秒10次）
        if self.frame_count % (FPS // 10) == 0:
            self._record_vehicle_data(vehicle)
    
    def _record_vehicle_data(self, vehicle):
        """记录车辆位置和速度数据"""
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 转换为km/h
        
        self.data_count += 1
        self.data_file.write(
            f"{self.frame_count},"
            f"{self.current_relative_time:.3f},"
            f"{location.x:.3f},"
            f"{location.y:.3f},"
            f"{location.z:.3f},"
            f"{speed:.2f}\n"
        )
        self.data_file.flush()
    
    def start_recording(self):
        """开始记录"""
        if not self.has_recorded:
            self.is_recording = True
            self.has_recorded = True
            self.frame_count = 0
            self.data_count = 0
            print(f"Started recording for vehicle {self.vehicle_id}")
    
    def stop_recording(self):
        """停止记录"""
        if self.is_recording:
            self.is_recording = False
            print(f"Stopping recording for vehicle {self.vehicle_id}. "
                  f"Frames: {self.frame_count}, Data points: {self.data_count}")
            
            # 停止视频线程
            self.video_thread.stop()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器（优化元数据处理）"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.rgb_sensor = None
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        self.rgb_queue = Queue()
        self.smooth_controller = SmoothControl(vehicle)
        
        # 记录状态控制
        self.was_on_street = False
        self.consecutive_off_street = 0
        self.max_off_street_frames = 30  # 最大允许离开道路的帧数（1秒）
        self.last_save_time = 0  # 上次保存图像的时间
        self.save_interval = 1.0  # 保存图像的时间间隔（秒）
        
        # 初始化RGB摄像头
        rgb_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        rgb_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        rgb_bp.set_attribute('fov', '100')  # 视野100度

        # 安装位置
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        
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
            
            # 如果正在记录，处理帧
            if self.recorder.is_recording:
                self.recorder.record_frame(rgb_image, self.vehicle)
                
                # 定期保存图像（每秒保存1张）
                current_time = time.time()
                if current_time - self.last_save_time >= self.save_interval:
                    self._save_image_and_metadata(rgb_image, vehicle_location)
                    self.last_save_time = current_time
                    
            # 应用平滑控制
            self._apply_smooth_control()
            
            # 清理队列
            self.rgb_queue.task_done()

    def _apply_smooth_control(self):
        """应用平滑控制指令"""
        # 获取当前控制状态
        current_control = self.vehicle.get_control()
        
        # 应用平滑控制
        self.smooth_controller.apply_smooth_control(
            throttle=current_control.throttle,
            steer=current_control.steer,
            brake=current_control.brake
        )

    def _save_image_and_metadata(self, rgb_image, vehicle_location):
        """优化元数据保存（仅收集附近车辆）"""
        # 获取当前记录器的相对时间
        relative_time = self.recorder.current_relative_time
        
        # 创建时间戳字符串
        timestamp_str = f"{relative_time:.3f}"
        
        # 按车辆ID创建子目录
        save_dir = f'output/camera_images/vehicle_{self.vehicle.id}'
        os.makedirs(save_dir, exist_ok=True)

        try:
            # 仅收集附近车辆（50米范围内）
            all_actors = self.world.get_actors().filter('vehicle.*')
            vehicle_data = []
            self_location = self.vehicle.get_location()
            
            for actor in all_actors:
                if actor.id != self.vehicle.id:
                    loc = actor.get_location()
                    distance = loc.distance(self_location)
                    if distance < 50:  # 50米范围
                        rotation = actor.get_transform().rotation
                        vehicle_data.append({
                            'id': actor.id,
                            'x': loc.x,
                            'y': loc.y,
                            'z': loc.z,
                            'yaw': rotation.yaw
                        })

            # 获取相机参数
            camera_transform = self.rgb_sensor.get_transform()
            metadata = {
                't': relative_time,  # 使用相对时间
                'cam': {
                    'x': camera_transform.location.x,
                    'y': camera_transform.location.y,
                    'z': camera_transform.location.z,
                    'yaw': camera_transform.rotation.yaw,
                    'fov': 100,
                    'intr': self.recorder.camera_intrinsics  # 添加内参
                },
                'vehicles': vehicle_data
            }

            # 保存元数据为紧凑JSON
            metadata_path = f'{save_dir}/{timestamp_str}_meta.json'
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, separators=(',', ':'))  # 最小化JSON大小
            
            # 保存RGB图片
            rgb_image.save_to_disk(f'{save_dir}/{timestamp_str}_rgb.png')
            
        except Exception as e:
            print(f"Error saving image: {e}")

def optimize_physics(vehicle):
    """优化车辆物理参数减少抖动（增强稳定性）"""
    physics_control = vehicle.get_physics_control()
    
    # 增加质量减少高频抖动
    physics_control.mass = 1500
    
    # 增加空气阻力系数
    physics_control.drag_coefficient = 0.3
    
    # 启用更精确的碰撞检测
    physics_control.use_sweep_wheel_collision = True
    
    # 增加车辆惯性稳定性
    physics_control.moi = 1.5 * physics_control.moi  # 增加转动惯量
    physics_control.center_of_mass.z = -0.5  # 降低重心
    
    # 调整悬挂参数
    for wheel in physics_control.wheels:
        wheel.damping_rate = 0.25
        wheel.max_steer_angle = 70.0
        wheel.radius = 30.0
        wheel.max_brake_torque = 1000.0
        wheel.max_handbrake_torque = 3000.0
    
    # 应用修改
    vehicle.apply_physics_control(physics_control)

def generate_sensor_spawn_points():
    """为感知车辆生成生成点（全部在左车道）"""
    sensor_spawn_points = []
    
    # 在整个左车道均匀分布车辆
    for i in range(NUM_SENSOR_VEHICLES):
        x = STREET_START.x - (i * (VEHICLE_LENGTH + VEHICLE_SPACING)) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
    
    return sensor_spawn_points

def generate_normal_spawn_points():
    """为普通车辆生成生成点，在两条车道均匀分布"""
    spawn_points = []
    
    # 计算最大可容纳车辆数
    max_vehicles_per_lane = min(NUM_NORMAL_VEHICLES // 2, 
                               int((STREET_START.x - STREET_END.x) / (VEHICLE_LENGTH + VEHICLE_SPACING)))
    
    # 左车道
    for i in range(max_vehicles_per_lane):
        x = STREET_START.x - (i * (VEHICLE_LENGTH + VEHICLE_SPACING)) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('LEFT', spawn_point))
    
    # 右车道
    for i in range(max_vehicles_per_lane):
        x = STREET_START.x - (i * (VEHICLE_LENGTH + VEHICLE_SPACING)) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('RIGHT', spawn_point))
    
    return spawn_points

def spawn_parked_vehicles(world):
    """生成违停车辆并保存坐标到文件"""
    parked_vehicles = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')

    # 只允许以下类型的小汽车
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt',
        'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020', 'vehicle.ford.crown', 'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon', 'vehicle.lincoln.mkz_2020', 'vehicle.mercedes.coupe_2020',
        'vehicle.micro.microlino', 'vehicle.mini.cooperst', 'vehicle.mustang.mustang',
        'vehicle.nissan.micra', 'vehicle.nissan.patrol_2021', 'vehicle.seat.leon',
        'vehicle.tesla.model3', 'vehicle.toyota.prius', 'vehicle.volkswagen.t2'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 获取计算好的停车位位置
    parking_spots = StreetMonitor.calculate_parking_positions()
    
    # 创建输出目录
    os.makedirs('output', exist_ok=True)
    
    # 创建违停车辆坐标文件
    coordinates_file = open('output/illegal_parking_coordinates.txt', 'w')
    coordinates_file.write("vehicle_id,x,y,z,yaw\n")

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
    print(f"Successfully spawned {len(parked_vehicles)} illegal parked vehicles")
    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
    """生成运动车辆（感知车辆和普通车辆）"""
    moving_vehicles = []
    camera_managers = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')

    # 只允许小汽车类型
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt',
        'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020', 'vehicle.ford.crown', 'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon', 'vehicle.lincoln.mkz_2020', 'vehicle.mercedes.coupe_2020',
        'vehicle.micro.microlino', 'vehicle.mini.cooperst', 'vehicle.mustang.mustang',
        'vehicle.nissan.micra', 'vehicle.nissan.patrol_2021', 'vehicle.seat.leon',
        'vehicle.tesla.model3', 'vehicle.toyota.prius',
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 生成感知车辆的生成点（全部在左车道）
    sensor_spawn_points = generate_sensor_spawn_points()
    
    # 生成感知车辆 - 全部在左车道
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
            
            # 优化物理参数
            optimize_physics(vehicle)
            
            # 设置自动导航
            vehicle.set_autopilot(True, traffic_manager.get_port())
            
            # 交通管理器设置（优化参数）
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            traffic_manager.ignore_signs_percentage(vehicle, 100)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 20.0)  # 增加安全距离
            traffic_manager.set_desired_speed(vehicle, 30)
            
            # 严格禁用变道行为
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            
            # 保持左车道
            traffic_manager.keep_right_rule_percentage(vehicle, 0)
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            sensor_vehicles_spawned += 1
            
    # 生成普通车辆的生成点（两条车道均匀分布）
    normal_spawn_points = generate_normal_spawn_points()
    
    # 分离左右车道的生成点
    right_lane_points = [point for lane, point in normal_spawn_points if lane == 'RIGHT']
    left_lane_points = [point for lane, point in normal_spawn_points if lane == 'LEFT']
    
    # 生成普通车辆
    normal_vehicles_spawned = 0
    
    # 左车道普通车辆
    for spawn_point in left_lane_points:
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            optimize_physics(vehicle)
            vehicle.set_autopilot(True, traffic_manager.get_port())
            
            # 交通管理器设置
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            traffic_manager.ignore_signs_percentage(vehicle, 100)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 20.0)  # 增加安全距离
            traffic_manager.set_desired_speed(vehicle, 30)
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
            
            normal_vehicles_spawned += 1

    # 右车道普通车辆
    for spawn_point in right_lane_points:
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            optimize_physics(vehicle)
            vehicle.set_autopilot(True, traffic_manager.get_port())
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            traffic_manager.ignore_signs_percentage(vehicle, 100)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 20.0)  # 增加安全距离
            traffic_manager.set_desired_speed(vehicle, 30)
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 100)  # 保持右车道
            
            normal_vehicles_spawned += 1

    print(f"\nSuccessfully spawned {len(moving_vehicles)} moving vehicles:")
    print(f"  - {sensor_vehicles_spawned} sensor vehicles (ALL in LEFT lane)")
    print(f"  - {normal_vehicles_spawned} normal vehicles")
    
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
        settings.fixed_delta_seconds = 0.05  # 20Hz
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        world.apply_settings(settings)

        # 设置天气
        weather = carla.WeatherParameters(
            cloudiness=0, precipitation=0, precipitation_deposits=0,
            wetness=0, sun_altitude_angle=15, fog_density=0, wind_intensity=0
        )
        world.set_weather(weather)
        
        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        
        # 设置交通参数（优化）
        traffic_manager.set_global_distance_to_leading_vehicle(20.0)  # 增加安全距离
        traffic_manager.global_percentage_speed_difference(0)
        traffic_manager.set_random_device_seed(42)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_hybrid_physics_radius(100.0)  # 扩大混合物理范围

        # 生成违停车辆
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):
            world.tick()

        # 生成运动车辆
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环（优化帧率控制）
        start_time = time.time()
        last_tick_time = time.time()
        last_fps_check = time.time()
        frame_count = 0
        
        print(f"\nSimulation started on road from x={STREET_START.x} to x={STREET_END.x}")
        print(f"Left lane center: y={LEFT_LANE_Y:.2f}, Right lane center: y={RIGHT_LANE_Y:.2f}")
        
        while time.time() - start_time < SIMULATION_TIME:
            # 精确帧率控制
            current_time = time.time()
            elapsed = current_time - last_tick_time
            sleep_time = max(0, 0.05 - elapsed)
            time.sleep(sleep_time)
            
            # 记录本次tick时间
            last_tick_time = time.time()
            frame_count += 1
            
            # 减少性能监控频率（每15秒）
            if frame_count % 300 == 0:
                fps = 300 / (time.time() - last_fps_check)
                print(f"Current FPS: {fps:.1f}, Vehicles: {len(moving_vehicles)}")
                last_fps_check = time.time()
            
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
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        except:
            pass

        # 销毁所有摄像头
        for manager in camera_managers:
            try:
                if manager.rgb_sensor and manager.rgb_sensor.is_alive:
                    manager.rgb_sensor.destroy()
            except:
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
        time.sleep(1.0)
        print("✅ Cleanup completed")

if __name__ == '__main__':
    main()