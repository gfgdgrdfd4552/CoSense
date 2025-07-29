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
import collections

# 常量定义
NUM_PARKED_VEHICLES = 9     # 违停车辆数量
NUM_SENSOR_VEHICLES = 10    # 感知车辆数量
NUM_NORMAL_VEHICLES = 30    # 普通车辆数量
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 640           # 摄像头图像宽度
IMAGE_HEIGHT = 480          # 摄像头图像高度
LANE_WIDTH = 3.5            # 单车道宽度(米)
FPS = 20                    # 视频帧率（降低至20Hz提高稳定性）
MIN_ROAD_LENGTH = 40.0      # 道路最小长度(米)用于拍摄视频

# 道路坐标(单向二车道) - 仅用于初始违停车辆
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 7            # 道路总宽度
VEHICLE_LENGTH = 4.5        # 平均车长(米)
VEHICLE_WIDTH = 2.0         # 平均车宽(米)
VEHICLE_SPACING = 8.0       # 增大车辆间距减少碰撞概率

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

class VideoWriterThread(threading.Thread):
    """异步视频写入线程（仅支持RGB视频）"""
    
    def __init__(self, vehicle_id, road_id=None):
        super().__init__()
        self.vehicle_id = vehicle_id
        self.road_id = road_id or "unknown_road"
        
        # 创建按道路ID和车辆ID分级的存储路径
        self.save_dir = f'output/camera_images/road_{self.road_id}/vehicle_{vehicle_id}'
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
        print(f"Video writer thread started for vehicle {self.vehicle_id} on road {self.road_id}")
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
        print(f"Video writer for vehicle {self.vehicle_id} on road {self.road_id} released, wrote {self.frame_count} frames")
    
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
    def get_current_road_id(vehicle, world):
        """获取车辆当前所在道路ID"""
        location = vehicle.get_location()
        waypoint = world.get_map().get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
        return waypoint.road_id if waypoint else -1
    
    @staticmethod
    def calculate_road_length(waypoint):
        """计算道路长度"""
        if waypoint is None:
            return 0.0
        
        # 使用BFS计算道路长度
        visited = set()
        queue = collections.deque()
        queue.append(waypoint)
        visited.add(waypoint.id)
        
        total_length = 0.0
        current_road_id = waypoint.road_id
        
        while queue:
            current = queue.popleft()
            
            # 获取所有连接的道路段
            for next_wp in current.next(2.0):  # 2米间隔
                if next_wp.id not in visited and next_wp.road_id == current_road_id:
                    # 计算当前点到下一个点的距离
                    distance = current.transform.location.distance(next_wp.transform.location)
                    total_length += distance
                    visited.add(next_wp.id)
                    queue.append(next_wp)
            
            # 检查反向道路
            for prev_wp in current.previous(2.0):  # 2米间隔
                if prev_wp.id not in visited and prev_wp.road_id == current_road_id:
                    # 计算当前点到上一个点的距离
                    distance = current.transform.location.distance(prev_wp.transform.location)
                    total_length += distance
                    visited.add(prev_wp.id)
                    queue.append(prev_wp)
        
        return total_length

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
    """车辆数据记录器（仅支持RGB数据）"""
    
    def __init__(self, vehicle_id, world):
        self.vehicle_id = vehicle_id
        self.world = world
        self.current_road_id = -1
        self.video_thread = None
        self.data_file = None
        self.road_length_cache = {}  # 道路长度缓存
        
        # 初始化数据文件
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
        
        # 获取当前道路ID
        location = vehicle.get_location()
        waypoint = self.world.get_map().get_waypoint(
            location, 
            project_to_road=True, 
            lane_type=carla.LaneType.Driving
        )
        
        # 检查道路长度
        if waypoint is not None:
            road_id = waypoint.road_id
            
            # 检查道路长度是否已知
            if road_id not in self.road_length_cache:
                # 计算并缓存道路长度
                road_length = StreetMonitor.calculate_road_length(waypoint)
                self.road_length_cache[road_id] = road_length
                print(f"Vehicle {self.vehicle_id}: Calculated road {road_id} length: {road_length:.2f}m")
            
            # 检查道路长度是否足够
            if self.road_length_cache[road_id] < MIN_ROAD_LENGTH:
                # 道路太短，不录制视频
                return
            
            # 检查道路是否变化，需要切换视频线程
            if road_id != self.current_road_id:
                if self.video_thread:
                    # 停止旧道路的视频线程
                    self.video_thread.stop()
                    self.video_thread = None
                
                # 创建新道路的视频线程
                self.current_road_id = road_id
                self.video_thread = VideoWriterThread(self.vehicle_id, self.current_road_id)
                self.video_thread.start()
                print(f"Vehicle {self.vehicle_id} switched to road {self.current_road_id}, started new video thread")
        else:
            # 不在道路上
            self.current_road_id = -1
            return
        
        # 添加帧到视频线程队列
        if self.video_thread:
            self.video_thread.add_frame(rgb_array)
        
        # 获取当前时间
        current_time = time.time()
        
        # 控制数据记录频率（每秒10次）
        if current_time - self.last_frame_time >= 0.1:
            self._record_vehicle_data(vehicle)
            self.last_frame_time = current_time
    
    def _record_vehicle_data(self, vehicle):
        """记录车辆位置和速度数据"""
        if not self.data_file:
            # 创建按道路ID和车辆ID分级的存储路径
            save_dir = f'output/camera_images/road_{self.current_road_id}/vehicle_{self.vehicle_id}'
            os.makedirs(save_dir, exist_ok=True)
            
            # 初始化数据文件
            self.data_file = open(f'{save_dir}/vehicle_data.txt', 'w')
            self.data_file.write("frame_number,relative_time(s),x,y,z,speed(km/h),road_id\n")
            
            # 保存相机内参到文件
            camera_intrinsics = self._calculate_camera_intrinsics()
            with open(f'{save_dir}/camera_intrinsics.txt', 'w') as f:
                f.write(f"Focal length (fx): {camera_intrinsics['fx']:.4f}\n")
                f.write(f"Focal length (fy): {camera_intrinsics['fy']:.4f}\n")
                f.write(f"Principal point (cx): {camera_intrinsics['cx']:.4f}\n")
                f.write(f"Principal point (cy): {camera_intrinsics['cy']:.4f}\n")
                f.write(f"Image width: {IMAGE_WIDTH}\n")
                f.write(f"Image height: {IMAGE_HEIGHT}\n")
                f.write(f"Field of View: 100 degrees\n")
        
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
            f"{speed:.2f},"
            f"{self.current_road_id}\n"  # 记录道路ID
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
            
            # 停止视频线程
            if self.video_thread:
                self.video_thread.stop()
            
            # 关闭数据文件
            if self.data_file:
                self.data_file.close()
                
            print(f"Stopping recording for vehicle {self.vehicle_id}. "
                  f"Duration: {duration:.2f}s, Frames: {self.frame_count}, Data points: {self.data_count}")

class VehicleCameraManager:
    """车辆摄像头管理器（仅支持RGB摄像机）"""

    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.rgb_sensor = None
        self.recorder = VehicleRecorder(vehicle.id, world)
        self.rgb_queue = Queue()
        self.smooth_controller = SmoothControl(vehicle)
        
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
            
            # 检查是否在道路上
            waypoint = self.world.get_map().get_waypoint(
                vehicle_location, 
                project_to_road=True, 
                lane_type=carla.LaneType.Driving
            )
            on_street = (waypoint is not None)
            
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
        """保存图片和元数据，使用相对时间命名"""
        # 获取当前记录器的相对时间
        relative_time = self.recorder.current_relative_time
        
        # 创建时间戳字符串
        timestamp_str = f"{relative_time:.3f}"
        
        # 按道路ID和车辆ID创建子目录
        save_dir = f'output/camera_images/road_{self.recorder.current_road_id}/vehicle_{self.vehicle.id}'
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
                'road_id': self.recorder.current_road_id,  # 添加道路ID
                'camera': {
                    'x': camera_transform.location.x,
                    'y': camera_transform.location.y,
                    'z': camera_transform.location.z,
                    'yaw': camera_transform.rotation.yaw,
                    'fov': 100,        # 与摄像头蓝图设置一致
                    'max_range': 30.0,  # 探测最大半径30米
                    'intrinsics': self.recorder._calculate_camera_intrinsics()  # 添加内参
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
            
            print(f"Saved image at time={timestamp_str}s for sensor vehicle {self.vehicle.id} on road {self.recorder.current_road_id}")
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
    """在整个地图上生成运动车辆（感知车辆和普通车辆）"""
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

    # 获取所有可能的生成点
    all_spawn_points = world.get_map().get_spawn_points()
    print(f"Total available spawn points: {len(all_spawn_points)}")
    
    # 随机选择生成点
    if len(all_spawn_points) < (NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES):
        print(f"Warning: Not enough spawn points ({len(all_spawn_points)}) for {NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES} vehicles")
        num_to_spawn = min(len(all_spawn_points), NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES)
    else:
        num_to_spawn = NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES
    
    # 随机打乱生成点
    random.shuffle(all_spawn_points)
    
    # 生成感知车辆
    sensor_vehicles_spawned = 0
    for i in range(num_to_spawn):
        if sensor_vehicles_spawned >= NUM_SENSOR_VEHICLES:
            break
            
        spawn_point = all_spawn_points[i]
        
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
            
            # 交通管理器设置
            traffic_manager.ignore_lights_percentage(vehicle, 100)  # 忽略红绿灯
            traffic_manager.ignore_signs_percentage(vehicle, 100)   # 忽略交通标志
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)  # 正常速度
            traffic_manager.distance_to_leading_vehicle(vehicle, 15.0)  # 15米安全距离
            traffic_manager.set_desired_speed(vehicle, 30)  # 目标速度30km/h
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle, world))
            sensor_vehicles_spawned += 1
            
            # 获取道路ID
            waypoint = world.get_map().get_waypoint(
                spawn_point.location, 
                project_to_road=True, 
                lane_type=carla.LaneType.Driving
            )
            road_id = waypoint.road_id if waypoint else -1
            
            print(f"Spawned sensor vehicle {vehicle.id} at position ({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f}) on road {road_id}")
        else:
            print(f"Failed to spawn sensor vehicle at position ({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f})")

    # 生成普通车辆
    normal_vehicles_spawned = 0
    for i in range(sensor_vehicles_spawned, num_to_spawn):
        spawn_point = all_spawn_points[i]
        
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
            
            # 交通管理器设置
            traffic_manager.ignore_lights_percentage(vehicle, 100)
            traffic_manager.ignore_signs_percentage(vehicle, 100)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.distance_to_leading_vehicle(vehicle, 15.0)  # 15米安全距离
            traffic_manager.set_desired_speed(vehicle, 30)  # 目标速度30km/h
            
            normal_vehicles_spawned += 1
            
            # 获取道路ID
            waypoint = world.get_map().get_waypoint(
                spawn_point.location, 
                project_to_road=True, 
                lane_type=carla.LaneType.Driving
            )
            road_id = waypoint.road_id if waypoint else -1
            print(f"Spawned normal vehicle {vehicle.id} at position ({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f}) on road {road_id}")

    print(f"\nSuccessfully spawned {len(moving_vehicles)} moving vehicles:")
    print(f"  - {sensor_vehicles_spawned} sensor vehicles")
    print(f"  - {normal_vehicles_spawned} normal vehicles")
    print(f"  - Total: {sensor_vehicles_spawned + normal_vehicles_spawned}")
    
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

        # 生成运动车辆（在整个地图上）
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        last_tick_time = time.time()
        last_fps_check = time.time()
        frame_count = 0
        
        print(f"\nSimulation started with {len(moving_vehicles)} vehicles on map")
        print(f"Only roads longer than {MIN_ROAD_LENGTH} meters will be recorded")
        
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
                print(f"Current FPS: {fps:.1f}")
                last_fps_check = time.time()
            
            # 推进仿真
            world.tick()
            
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