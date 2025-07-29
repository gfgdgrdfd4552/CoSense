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
import psutil  # 用于监控内存使用

# 常量定义
NUM_PARKED_VEHICLES = 9     # 违停车辆数量
NUM_SENSOR_VEHICLES = 10    # 感知车辆数量
NUM_NORMAL_VEHICLES = 30    # 普通车辆数量
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 1920          # 摄像头图像宽度
IMAGE_HEIGHT = 1080         # 摄像头图像高度
LANE_WIDTH = 3.5            # 单车道宽度(米)
FPS = 20                    # 视频帧率（降低至20Hz提高稳定性）
MAX_QUEUE_SIZE = 30         # 图像队列最大大小（高分辨率需要减小队列大小）
SAVE_INTERVAL = 5.0         # 元数据保存间隔（秒）减少磁盘IO

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 7            # 道路总宽度
VEHICLE_LENGTH = 4.5        # 平均车长(米)
VEHICLE_WIDTH = 2.0         # 平均车宽(米)
VEHICLE_SPACING = 8.0       # 增大车辆间距减少碰撞概率

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

# 车道中心线定义
LEFT_LANE_Y = 15.2 + LANE_WIDTH / 2   # 左车道中心线 y=17.95
RIGHT_LANE_Y = 15.2 - LANE_WIDTH / 2  # 右车道中心线 y=12.45

class VideoWriterThread(threading.Thread):
    """异步视频写入线程（优化高分辨率性能）"""
    
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
        
        # 图像队列（存储rgb_frame） - 减小队列大小以适应高分辨率
        self.queue = queue.Queue(maxsize=MAX_QUEUE_SIZE)  # 限制队列大小防止内存溢出
        self.stop_event = threading.Event()
        self.daemon = True  # 守护线程，主程序退出时自动结束
        self.frame_count = 0
        
    def run(self):
        """线程主函数"""
        print(f"Video writer thread started for vehicle {self.vehicle_id}")
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                # 从队列获取图像，最多等待0.5秒
                rgb_array = self.queue.get(timeout=0.5)
                
                # 写入RGB视频
                self.rgb_writer.write(rgb_array)
                
                self.queue.task_done()
                self.frame_count += 1
                
                # 每100帧报告一次性能
                if self.frame_count % 100 == 0:
                    mem_usage = psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024)
                    print(f"Vehicle {self.vehicle_id} video thread: Frames written: {self.frame_count}, "
                          f"Queue size: {self.queue.qsize()}, Mem usage: {mem_usage:.1f}MB")
            except queue.Empty:
                continue
        
        # 释放资源
        self.rgb_writer.release()
        print(f"Video writer for vehicle {self.vehicle_id} released, wrote {self.frame_count} frames")
    
    def add_frame(self, rgb_array):
        """添加帧到队列（高分辨率优化）"""
        if not self.stop_event.is_set():
            try:
                # 如果队列已满，丢弃最旧的帧
                if self.queue.full():
                    try:
                        self.queue.get_nowait()  # 丢弃一帧
                    except queue.Empty:
                        pass
                
                # 存储RGB图像（避免复制，直接使用传入的数组）
                self.queue.put_nowait(rgb_array)
            except queue.Full:
                pass  # 忽略队列满异常
            except Exception as e:
                print(f"Error adding frame to video queue: {e}")
    
    def stop(self):
        """停止线程"""
        self.stop_event.set()
        self.join(timeout=2.0)  # 最多等待2秒

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
            offset = direction * (i * (VEHICLE_LENGTH + 1.2))  # 增加间距
            
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
    """车辆数据记录器（高分辨率优化）"""
    
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
        self.last_data_time = 0
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
        """记录视频帧和车辆数据（高分辨率优化）"""
        if not self.is_recording:
            return
            
        # 帧计数器递增
        self.frame_count += 1
        
        # 计算当前帧相对时间
        self.current_relative_time = self.frame_count / FPS
        
        # 转换RGB图像格式 (优化内存使用)
        try:
            # 使用更高效的内存处理方式
            rgb_array = np.frombuffer(rgb_image.raw_data, dtype=np.uint8)
            rgb_array = np.reshape(rgb_array, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))
            
            # 直接在原数组上操作，避免额外复制
            # 去除alpha通道并转换为BGR
            bgr_array = np.empty((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
            bgr_array[:, :, 0] = rgb_array[:, :, 2]  # R -> B
            bgr_array[:, :, 1] = rgb_array[:, :, 1]  # G
            bgr_array[:, :, 2] = rgb_array[:, :, 0]  # B -> R
        except Exception as e:
            print(f"Error processing image: {e}")
            return
        
        # 添加帧到视频线程队列
        self.video_thread.add_frame(bgr_array)
        
        # 获取当前时间
        current_time = time.time()
        
        # 控制数据记录频率（每秒最多10次）
        if current_time - self.last_data_time >= 0.1:
            self._record_vehicle_data(vehicle)
            self.last_data_time = current_time
    
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
        # 减少flush频率，每10次数据写操作flush一次
        if self.data_count % 10 == 0:
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
            self.last_data_time = time.time()
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
            
            # 确保数据写入完成
            self.data_file.flush()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器（高分辨率优化）"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.rgb_sensor = None
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        self.rgb_queue = Queue(maxsize=5)  # 减小队列大小以适应高分辨率
        self.smooth_controller = SmoothControl(vehicle)
        
        # 记录状态控制
        self.was_on_street = False  # 上一帧是否在道路上
        self.consecutive_off_street = 0  # 连续不在道路上的帧数
        self.max_off_street_frames = 30  # 最大允许离开道路的帧数（1秒）
        self.last_save_time = time.time()  # 上次保存图像的时间
        
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
        """处理RGB图像（优化内存）"""
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return
            
        try:
            # 将图像放入队列（不处理，直接传递原始图像）
            self.rgb_queue.put(image)
        except queue.Full:
            # 队列满时丢弃帧
            pass

    def _process_images(self):
        """处理RGB图像（优化性能）"""
        # 当队列有数据时处理
        while not self.rgb_queue.empty():
            try:
                rgb_image = self.rgb_queue.get_nowait()
            except queue.Empty:
                break
            
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
                
                # 定期保存图像（降低保存频率）
                current_time = time.time()
                if current_time - self.last_save_time >= SAVE_INTERVAL:
                    self._save_image_and_metadata(rgb_image, vehicle_location)
                    self.last_save_time = current_time
                    
            # 应用平滑控制（替代交通管理器控制）
            self._apply_smooth_control()
            
            # 清理队列
            self.rgb_queue.task_done()

    def _apply_smooth_control(self):
        """应用平滑控制指令"""
        # 获取当前控制状态
        current_control = self.vehicle.get_control()
        
        # 应用平滑控制（保持直线行驶）
        self.smooth_controller.apply_smooth_control(
            throttle=current_control.throttle,
            steer=current_control.steer,
            brake=current_control.brake
        )

    def _save_image_and_metadata(self, rgb_image, vehicle_location):
        """保存图片和元数据，使用相对时间命名（减少IO频率）"""
        # 获取当前记录器的相对时间
        relative_time = self.recorder.current_relative_time
        
        # 创建时间戳字符串
        timestamp_str = f"{relative_time:.3f}"
        
        # 按车辆ID创建子目录
        save_dir = f'output/camera_images/vehicle_{self.vehicle.id}'
        os.makedirs(save_dir, exist_ok=True)

        try:
            # 获取所有车辆信息（只记录100米范围内的车辆）
            all_actors = self.world.get_actors().filter('vehicle.*')
            vehicle_data = []
            
            for actor in all_actors:
                if actor.id == self.vehicle.id:
                    continue
                    
                # 计算与主车的距离
                loc = actor.get_location()
                distance = math.sqrt(
                    (loc.x - vehicle_location.x) ** 2 +
                    (loc.y - vehicle_location.y) ** 2
                )
                
                # 只记录100米范围内的车辆
                if distance < 100.0:
                    rotation = actor.get_transform().rotation
                    vehicle_data.append({
                        'id': actor.id,
                        'x': loc.x,
                        'y': loc.y,
                        'z': loc.z,
                        'yaw': rotation.yaw,
                        'distance': distance,
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
                    'max_range': 100.0,  # 探测最大半径100米
                    'intrinsics': self.recorder.camera_intrinsics  # 添加内参
                },
                'vehicles': vehicle_data
            }

            # 保存元数据为JSON文件
            metadata_path = f'{save_dir}/{timestamp_str}_meta.json'
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f)
            
            # 保存RGB图片（降低保存频率）
            rgb_image.save_to_disk(
                f'{save_dir}/{timestamp_str}_rgb.png'
            )
            
            # 减少日志输出频率
            if random.random() < 0.1:  # 10%的概率输出日志
                print(f"Saved image at time={timestamp_str}s for sensor vehicle {self.vehicle.id}")
        except Exception as e:
            print(f"Error saving image and metadata: {e}")

def optimize_physics(vehicle):
    """优化车辆物理参数减少抖动"""
    physics_control = vehicle.get_physics_control()
    
    # 增加质量减少高频抖动
    physics_control.mass = 1500
    
    # 增加空气阻力系数
    physics_control.drag_coefficient = 0.3
    
    # 启用更精确的碰撞检测
    physics_control.use_sweep_wheel_collision = True
    
    # 调整悬挂参数
    for wheel in physics_control.wheels:
        wheel.damping_rate = 0.25
        wheel.max_steer_angle = 70.0
        wheel.radius = 30.0
        wheel.max_brake_torque = 1000.0
        wheel.max_handbrake_torque = 3000.0
    
    # 应用修改
    vehicle.apply_physics_control(physics_control)
    print(f"Optimized physics for vehicle {vehicle.id}")

def generate_sensor_spawn_points():
    """为感知车辆生成生成点（全部在左车道）"""
    sensor_spawn_points = []
    
    # 在整个左车道均匀分布车辆
    for i in range(NUM_SENSOR_VEHICLES):
        x = STREET_START.x - (i * (VEHICLE_LENGTH + VEHICLE_SPACING)) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),  # 左车道
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"Sensor spawn point (LEFT): x={x:.1f}, y={LEFT_LANE_Y:.1f}")
    
    print(f"Generated {len(sensor_spawn_points)} sensor spawn points (all in LEFT lane)")
    return sensor_spawn_points

def generate_normal_spawn_points():
    """为普通车辆生成生成点，在两条车道均匀分布"""
    spawn_points = []
    
    # 计算最大可容纳车辆数（每条车道）
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
        print(f"Normal spawn point (LEFT): x={x:.1f}, y={LEFT_LANE_Y:.1f}")
    
    # 右车道
    for i in range(max_vehicles_per_lane):
        x = STREET_START.x - (i * (VEHICLE_LENGTH + VEHICLE_SPACING)) - random.uniform(0, 2)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),  # 右侧车道
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('RIGHT', spawn_point))
        print(f"Normal spawn point (RIGHT): x={x:.1f}, y={RIGHT_LANE_Y:.1f}")
    
    print(f"Generated {len(spawn_points)} normal vehicle spawn points "
          f"({max_vehicles_per_lane} in LEFT, {max_vehicles_per_lane} in RIGHT)")
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

            print(f"Spawned illegal parked vehicle {i + 1} at position: "
                  f"({spot.location.x:.2f}, {spot.location.y:.2f}) "
                  f"orientation: {spot.rotation.yaw:.1f}°")
        else:
            print(f"Failed to spawn vehicle at position ({spot.location.x:.2f}, {spot.location.y:.2f})")
    
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
    print(f"Saved illegal parking coordinates to output/illegal_parking_coordinates.txt")

    print(f"Successfully spawned {len(parked_vehicles)} illegal parked vehicles")
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
            vehicle.set_autopilot(True, traffic_manager.get_port())  # 关键修复：添加交通管理器端口
            
            # 交通管理器设置
            traffic_manager.ignore_lights_percentage(vehicle, 100)  # 忽略红绿灯
            traffic_manager.ignore_signs_percentage(vehicle, 100)   # 忽略交通标志
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)  # 正常速度
            traffic_manager.distance_to_leading_vehicle(vehicle, 15.0)  # 15米安全距离
            traffic_manager.set_desired_speed(vehicle, 30)  # 目标速度30km/h
            
            # 严格禁用变道行为
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            
            # 所有感知车辆都在左车道，设置保持左车道
            traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 0%靠右，即保持左车道
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            sensor_vehicles_spawned += 1
            
            print(f"Spawned sensor vehicle {vehicle.id} in LEFT lane at x={spawn_point.location.x:.1f}, y={spawn_point.location.y:.1f}")
        else:
            print(f"Failed to spawn sensor vehicle at position ({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f})")

    # 生成普通车辆的生成点（两条车道均匀分布）
    normal_spawn_points = generate_normal_spawn_points()
    
    # 分离左右车道的生成点
    right_lane_points = [point for lane, point in normal_spawn_points if lane == 'RIGHT']
    left_lane_points = [point for lane, point in normal_spawn_points if lane == 'LEFT']
    
    print(f"Available normal spawn points - RIGHT: {len(right_lane_points)}, LEFT: {len(left_lane_points)}")

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
            
            # 优化物理参数
            optimize_physics(vehicle)
            
            # 设置自动导航
            vehicle.set_autopilot(True, traffic_manager.get_port())  # 关键修复：添加交通管理器端口
            
            # 交通管理器设置
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            traffic_manager.ignore_signs_percentage(vehicle, 100)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 15.0)  # 15米安全距离
            traffic_manager.set_desired_speed(vehicle, 30)  # 目标速度30km/h
            
            # 严格禁用变道
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
            
            # 优化物理参数
            optimize_physics(vehicle)
            
            # 设置自动导航
            vehicle.set_autopilot(True, traffic_manager.get_port())  # 关键修复：添加交通管理器端口
            
            # 交通管理器设置
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            traffic_manager.ignore_signs_percentage(vehicle, 100)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 15.0)  # 15米安全距离
            traffic_manager.set_desired_speed(vehicle, 30)  # 目标速度30km/h
            
            # 严格禁用变道
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 100)  # 保持右车道
            
            normal_vehicles_spawned += 1

    print(f"\nSuccessfully spawned {len(moving_vehicles)} moving vehicles:")
    print(f"  - {sensor_vehicles_spawned} sensor vehicles (ALL in LEFT lane)")
    print(f"  - {normal_vehicles_spawned} normal vehicles ({len(left_lane_points)} in LEFT, {len(right_lane_points)} in RIGHT)")
    print(f"  - Total: {sensor_vehicles_spawned + normal_vehicles_spawned}")
    
    return moving_vehicles, camera_managers

def main():
    # 初始化变量
    parked_vehicles = []
    moving_vehicles = []
    camera_managers = []
    last_perf_check = time.time()
    frame_count = 0
    
    try:
        # 创建输出目录
        os.makedirs('output/camera_images', exist_ok=True)

        # 连接CARLA服务端
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()

        # ==== 关键修复1：同步模式设置 ====
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20Hz更稳定
        settings.max_substep_delta_time = 0.01  # 最大子步长
        settings.max_substeps = 10  # 最大子步数
        world.apply_settings(settings)

        # 设置天气
        weather = carla.WeatherParameters(
            cloudiness=0,          
            precipitation=0,       
            precipitation_deposits=0,
            wetness=0,             
            sun_altitude_angle=15,  
            fog_density=0,         
            wind_intensity=0       
        )
        world.set_weather(weather)
        
        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        
        # ==== 关键修复2：交通管理器同步设置 ====
        traffic_manager.set_synchronous_mode(True)  # 必须与主世界同步
        
        # 设置交通参数
        traffic_manager.set_global_distance_to_leading_vehicle(15.0)  # 全局安全距离15米
        traffic_manager.global_percentage_speed_difference(0)  # 正常速度30km/h
        traffic_manager.set_random_device_seed(42)  # 固定随机种子
        traffic_manager.set_hybrid_physics_mode(True)  # 启用混合物理模式提高性能
        traffic_manager.set_hybrid_physics_radius(70.0)  # 混合物理半径70米

        # 生成违停车辆（9辆）
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（在指定道路上）
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        last_tick_time = time.time()
        last_fps_check = time.time()
        frame_count = 0
        
        print(f"\nSimulation started on road from x={STREET_START.x} to x={STREET_END.x}")
        print(f"Left lane center: y={LEFT_LANE_Y:.2f}, Right lane center: y={RIGHT_LANE_Y:.2f}")
        print(f"Sensor vehicles ({NUM_SENSOR_VEHICLES}) distributed in LEFT lane only")
        print(f"Normal vehicles ({NUM_NORMAL_VEHICLES}) - evenly distributed in both lanes")
        print(f"High resolution mode: {IMAGE_WIDTH}x{IMAGE_HEIGHT} @ {FPS} FPS")
        
        while time.time() - start_time < SIMULATION_TIME:
            # 控制帧率 - 确保每帧间隔接近0.05秒
            current_time = time.time()
            elapsed = current_time - last_tick_time
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)
            
            # 记录本次tick时间
            last_tick_time = time.time()
            frame_count += 1
            
            # 每100帧检查一次FPS和内存使用
            if frame_count % 100 == 0:
                current_time = time.time()
                elapsed = current_time - last_fps_check
                fps = 100 / elapsed
                mem_usage = psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024)  # MB
                print(f"Frame: {frame_count}, FPS: {fps:.1f}, Memory: {mem_usage:.1f}MB")
                last_fps_check = current_time
            
            # 推进仿真
            world.tick()
            
            # 处理摄像头图像
            for manager in camera_managers:
                manager._process_images()
            
            # 每30秒报告一次平均速度
            if frame_count % 600 == 0:  # 30秒（30*20fps）
                total_speed = 0
                count = 0
                for vehicle in moving_vehicles:
                    if vehicle.is_alive:
                        velocity = vehicle.get_velocity()
                        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                        total_speed += speed
                        count += 1
                
                if count > 0:
                    avg_speed = total_speed / count
                    print(f"Average vehicle speed: {avg_speed:.1f} km/h ({count} vehicles)")

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
                    print(f"Stopped recording for vehicle {manager.vehicle.id}")
            except Exception as e:
                print(f"Error stopping recorder: {e}")

        # 恢复异步模式
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            print("Restored asynchronous mode")
        except:
            pass

        # 销毁所有摄像头
        for manager in camera_managers:
            try:
                if manager.rgb_sensor and manager.rgb_sensor.is_alive:
                    manager.rgb_sensor.destroy()
            except Exception as e:
                print(f"Error destroying sensor: {e}")

        # 销毁所有车辆
        actor_list = parked_vehicles + moving_vehicles
        destroyed_count = 0
        for actor in actor_list:
            try:
                if actor.is_alive:
                    actor.destroy()
                    destroyed_count += 1
            except Exception as e:
                print(f"Error destroying vehicle: {e}")
        
        print(f"Destroyed {destroyed_count} vehicles")
        time.sleep(1.0)  # 确保资源释放
        print("✅ Cleanup completed")

if __name__ == '__main__':
    main()