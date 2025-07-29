import os
import time
import random
import weakref
import math
import carla
import json
import cv2
import numpy as np
from queue import Queue

# 修改后的常量定义
NUM_PARKED_VEHICLES = 5    # 违停车辆数量减少到5
NUM_SENSOR_VEHICLES = 15   # 感知车辆数量设置为15
NUM_NORMAL_VEHICLES = 85   # 普通车辆数量设为85（总共100辆运动车辆）
SIMULATION_TIME = 3600     # 模拟时长(秒)
IMAGE_WIDTH = 800          # 摄像头图像宽度
IMAGE_HEIGHT = 600         # 摄像头图像高度
LANE_WIDTH = 3.5           # 单车道宽度(米)
FPS = 30                   # 视频帧率

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 10          # 道路总宽度
VEHICLE_LENGTH = 4.5       # 平均车长(米)
VEHICLE_WIDTH = 2.0        # 平均车宽(米)
VEHICLE_SPACING = 1.0      # 车辆间距(米)

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

# 目标路径关键点（让车辆更容易经过目标道路）
TARGET_WAYPOINTS = [
    carla.Location(x=80, y=15.2, z=0),   # 道路起点
    carla.Location(x=60, y=15.2, z=0),   # 道路中段1
    carla.Location(x=40, y=15.2, z=0),   # 道路中段2
    carla.Location(x=20, y=15.2, z=0),   # 道路中段3
    carla.Location(x=0, y=15.2, z=0),    # 道路中段4
    carla.Location(x=-10, y=15.2, z=0),  # 道路终点
]

def clear_existing_vehicles(world):
    """清除地图上所有已存在的车辆"""
    print("Clearing existing vehicles from the map...")
    
    # 获取所有车辆actor
    vehicles = world.get_actors().filter('vehicle.*')
    
    print(f"Found {len(vehicles)} existing vehicles to remove")
    
    # 销毁所有车辆
    destroyed_count = 0
    for vehicle in vehicles:
        try:
            vehicle.destroy()
            destroyed_count += 1
        except Exception as e:
            print(f"Failed to destroy vehicle {vehicle.id}: {e}")
    
    print(f"Successfully destroyed {destroyed_count} vehicles")
    
    # 等待一段时间确保车辆被完全清除
    time.sleep(2.0)
    
    # 验证清除结果
    remaining_vehicles = world.get_actors().filter('vehicle.*')
    print(f"Remaining vehicles after cleanup: {len(remaining_vehicles)}")
    
    return destroyed_count

def clear_existing_actors(world, actor_types=None):
    """清除地图上指定类型的actors"""
    if actor_types is None:
        actor_types = ['vehicle.*', 'walker.*']  # 默认清除车辆和行人
    
    print(f"Clearing existing actors: {actor_types}")
    
    total_destroyed = 0
    
    for actor_type in actor_types:
        actors = world.get_actors().filter(actor_type)
        print(f"Found {len(actors)} {actor_type} to remove")
        
        destroyed_count = 0
        for actor in actors:
            try:
                actor.destroy()
                destroyed_count += 1
            except Exception as e:
                print(f"Failed to destroy {actor_type} {actor.id}: {e}")
        
        print(f"Successfully destroyed {destroyed_count} {actor_type}")
        total_destroyed += destroyed_count
    
    # 等待一段时间确保actors被完全清除
    time.sleep(2.0)
    
    # 验证清除结果
    for actor_type in actor_types:
        remaining = world.get_actors().filter(actor_type)
        print(f"Remaining {actor_type} after cleanup: {len(remaining)}")
    
    return total_destroyed

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
    """车辆数据记录器"""
    
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.save_dir = f'output/camera_images/vehicle_{vehicle_id}'
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 初始化视频写入器
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        video_path = f'{self.save_dir}/recording_{timestamp}.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(video_path, fourcc, FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))
        
        # 初始化数据文件
        self.data_file = open(f'{self.save_dir}/vehicle_data.txt', 'w')
        self.data_file.write("frame_number,relative_time(s),x,y,z,speed(km/h)\n")
        
        # 记录状态
        self.is_recording = False
        self.has_recorded = False  # 标记是否已经记录过
        self.start_time = None     # 记录开始时间
        self.frame_count = 0       # 帧计数器
        self.data_count = 0        # 数据记录计数器
        
    def record_frame(self, image, vehicle):
        """记录视频帧和车辆数据"""
        if not self.is_recording:
            return
            
        # 转换图像格式并写入视频
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # 去除alpha通道
        array = array[:, :, ::-1]  # 从RGB转换为BGR
        self.video_writer.write(array)
        
        # 帧计数器递增
        self.frame_count += 1
        
        # 每3帧记录一次数据（30fps/3 = 10Hz），确保与视频同步
        if self.frame_count % 3 == 0:
            self._record_vehicle_data(vehicle)
    
    def _record_vehicle_data(self, vehicle):
        """记录车辆位置和速度数据"""
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 转换为km/h
        
        # 计算相对时间（基于帧数计算，确保与视频同步）
        relative_time = self.frame_count / FPS
        
        self.data_count += 1
        self.data_file.write(
            f"{self.frame_count},"
            f"{relative_time:.3f},"
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
            print(f"Started recording for vehicle {self.vehicle_id}")
    
    def stop_recording(self):
        """停止记录"""
        if self.is_recording:
            self.is_recording = False
            duration = self.frame_count / FPS  # 基于帧数计算实际时长
            print(f"Stopped recording for vehicle {self.vehicle_id}. "
                  f"Duration: {duration:.2f}s, Frames: {self.frame_count}, Data points: {self.data_count}")
            self.video_writer.release()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器（仅用于感知车辆）"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.sensor = None
        self.recorded_positions = set()  # 初始化已记录的x位置集合
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        self.image_queue = Queue()
        
        # 记录状态控制
        self.was_on_street = False  # 上一帧是否在道路上
        self.consecutive_off_street = 0  # 连续不在道路上的帧数
        self.max_off_street_frames = 30  # 最大允许离开道路的帧数（1秒）
        
        # 初始化摄像头
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        bp.set_attribute('fov', '100')  # 视野100度

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))  # 摄像头安装位置
        self.sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)

        # 设置图像回调函数
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: self._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return

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
            self.recorder.record_frame(image, self.vehicle)
            
        # 修改：只有在目标道路上才检查拍照位置
        if on_street:
            current_x = round(vehicle_location.x)
            target_positions = [80, 70, 60, 50, 40, 30, 20, 10, 0]
            for target_x in target_positions:
                if (abs(current_x - target_x) <= 0.5 and 
                    target_x not in self.recorded_positions):
                    self._save_image_and_metadata(image, vehicle_location, target_x)
                    self.recorded_positions.add(target_x)
                    print(f"Saved image at x={target_x} for sensor vehicle {self.vehicle.id} (ON TARGET STREET)")

    def _save_image_and_metadata(self, image, vehicle_location, target_x):
        """保存图片和元数据（保留原有功能）"""
        # 按车辆ID创建子目录
        save_dir = f'output/camera_images/vehicle_{self.vehicle.id}'
        os.makedirs(save_dir, exist_ok=True)

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
        camera_transform = self.sensor.get_transform()
        metadata = {
            'timestamp': time.time(),
            'camera': {
                'x': camera_transform.location.x,
                'y': camera_transform.location.y,
                'z': camera_transform.location.z,
                'yaw': camera_transform.rotation.yaw,
                'fov': 100,        # 与摄像头蓝图设置一致
                'max_range': 30.0   # 探测最大半径30米
            },
            'vehicles': vehicle_data
        }

        # 保存元数据为JSON文件
        metadata_path = f'{save_dir}/x{target_x}_y{vehicle_location.y:.2f}_z{vehicle_location.z:.2f}_meta.json'
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f)
        
        # 保存图片（文件名与metadata对应）
        image.save_to_disk(
            f'{save_dir}/x{target_x}_y{vehicle_location.y:.2f}_z{vehicle_location.z:.2f}.png'
        )

class VehicleNavigator:
    """车辆导航器 - 引导车辆经过目标道路（CARLA 0.9.15兼容版本）"""
    
    def __init__(self, vehicle, world, traffic_manager):
        self.vehicle = vehicle
        self.world = world
        self.traffic_manager = traffic_manager
        self.target_reached = False
        self.current_target_index = 0
        self.update_interval = 100  # 每100帧更新一次目标
        self.frame_count = 0
        
        # 为CARLA 0.9.15设置导航参数
        self.setup_navigation()
        
    def setup_navigation(self):
        """设置导航参数（CARLA 0.9.15兼容）"""
        try:
            # 设置车辆行为参数
            self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, -10)  # 比限速快10%
            self.traffic_manager.distance_to_leading_vehicle(self.vehicle, 2.0)  # 跟车距离2米
            
            # 设置变道参数 - 完全禁止变道
            self.traffic_manager.random_left_lanechange_percentage(self.vehicle, 0)
            self.traffic_manager.random_right_lanechange_percentage(self.vehicle, 0)
            self.traffic_manager.auto_lane_change(self.vehicle, False)  # 禁止自动变道
            
            # 设置目标点（如果API支持）
            if hasattr(self.traffic_manager, 'set_path'):
                self.traffic_manager.set_path(self.vehicle, TARGET_WAYPOINTS)
                print(f"Set navigation path for vehicle {self.vehicle.id}")
            
        except Exception as e:
            print(f"Navigation setup failed for vehicle {self.vehicle.id}: {e}")
        
    def update_navigation(self):
        """更新导航目标（CARLA 0.9.15兼容）"""
        self.frame_count += 1
        
        if self.frame_count % self.update_interval != 0:
            return
            
        if self.target_reached:
            return
            
        current_location = self.vehicle.get_location()
        
        # 检查是否接近当前目标点
        if self.current_target_index < len(TARGET_WAYPOINTS):
            target = TARGET_WAYPOINTS[self.current_target_index]
            distance = math.sqrt(
                (current_location.x - target.x) ** 2 + 
                (current_location.y - target.y) ** 2
            )
            
            # 如果接近目标点，切换到下一个目标点
            if distance < 15.0:  # 15米内算作到达
                self.current_target_index += 1
                if self.current_target_index >= len(TARGET_WAYPOINTS):
                    self.target_reached = True
                    print(f"Vehicle {self.vehicle.id} completed target route")
                    return
                    
                print(f"Vehicle {self.vehicle.id} reached waypoint {self.current_target_index-1}, "
                      f"heading to waypoint {self.current_target_index}")

def spawn_parked_vehicles(world):
    """在指定位置(x=15.39, y=9.76)沿着道路方向生成5辆违停车辆"""
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

    print(f"Successfully spawned {len(parked_vehicles)} illegal parked vehicles at ({ILLEGAL_PARKING_POINT.x}, {ILLEGAL_PARKING_POINT.y})")
    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
    """生成运动车辆，优先选择会经过目标道路的生成点"""
    moving_vehicles = []
    camera_managers = []
    navigators = []
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

    # 获取所有生成点
    all_spawn_points = world.get_map().get_spawn_points()
    
    # 优先选择目标道路上游的生成点（x坐标较大的位置）
    preferred_spawn_points = []
    backup_spawn_points = []
    
    for spawn_point in all_spawn_points:
        # 优先选择x坐标大于目标道路起点的生成点
        if spawn_point.location.x > STREET_START.x:
            # 选择y坐标接近目标道路的生成点
            if abs(spawn_point.location.y - STREET_START.y) < 20:
                preferred_spawn_points.append(spawn_point)
        else:
            backup_spawn_points.append(spawn_point)
    
    # 合并生成点列表，优先使用preferred点
    target_spawn_points = preferred_spawn_points + backup_spawn_points
    
    print(f"Found {len(preferred_spawn_points)} preferred spawn points, {len(backup_spawn_points)} backup points")
    
    random.shuffle(target_spawn_points)  # 随机打乱生成点顺序

    # 总共生成 NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES 辆运动车辆
    total_vehicles = NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES
    
    # 如果生成点不够，循环使用
    if len(target_spawn_points) < total_vehicles:
        print(f"Warning: Only {len(target_spawn_points)} spawn points available, "
              f"but need {total_vehicles}. Reusing spawn points.")
        # 复制生成点直到满足数量要求
        while len(target_spawn_points) < total_vehicles:
            target_spawn_points.extend(target_spawn_points.copy())
        target_spawn_points = target_spawn_points[:total_vehicles]

    # 生成车辆
    for i in range(total_vehicles):
        spawn_point = target_spawn_points[i]

        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 创建导航器引导车辆经过目标道路（所有车辆都需要）
            navigator = VehicleNavigator(vehicle, world, traffic_manager)
            navigators.append(navigator)
            
            # 仅为前15辆（感知车辆）安装摄像头
            if i < NUM_SENSOR_VEHICLES:
                camera_managers.append(VehicleCameraManager(vehicle))
                spawn_type = "SENSOR"
            else:
                spawn_type = "NORMAL"
                
            is_preferred = i < len(preferred_spawn_points)
            print(f"Spawned {spawn_type} vehicle {vehicle.id} ({vehicle.type_id}) at ({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f})")

    print(f"Successfully spawned {len(moving_vehicles)} moving vehicles ({NUM_SENSOR_VEHICLES} sensor, {NUM_NORMAL_VEHICLES} normal)")
    return moving_vehicles, camera_managers, navigators

def main():
    # 初始化变量
    parked_vehicles = []
    moving_vehicles = []
    camera_managers = []
    navigators = []
    
    try:
        # 创建输出目录
        os.makedirs('output/camera_images', exist_ok=True)

        # 连接CARLA服务端
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()

        # 清除地图上已存在的车辆和行人
        print("=== CLEARING EXISTING ACTORS ===")
        clear_existing_actors(world, ['vehicle.*', 'walker.*'])
        
        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 固定时间步长0.05秒
        world.apply_settings(settings)

        # 设置天气
        weather = carla.WeatherParameters(
            cloudiness=30, precipitation=0, sun_altitude_angle=70)
        world.set_weather(weather)

        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)  # 与前车保持距离
        traffic_manager.set_random_device_seed(42)  # 固定随机种子

        # 生成违停车辆（5辆）
        print("=== SPAWNING PARKED VEHICLES ===")
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（15辆感知车辆 + 85辆普通车辆）
        print("=== SPAWNING MOVING VEHICLES ===")
        moving_vehicles, camera_managers, navigators = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        print("=== SIMULATION STARTED ===")
        print(f"Simulation started with {NUM_SENSOR_VEHICLES} sensor vehicles and {NUM_NORMAL_VEHICLES} normal vehicles")
        print(f"Target street: from ({STREET_START.x}, {STREET_START.y}) to ({STREET_END.x}, {STREET_END.y})")
        print("Vehicles will be guided to pass through the target street (no lane changing allowed)")

        while time.time() - start_time < SIMULATION_TIME:
            # 更新所有车辆的导航
            for navigator in navigators:
                try:
                    navigator.update_navigation()
                except:
                    pass  # 忽略导航错误
            
            world.tick()  # 推进仿真

        print("Simulation finished")

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        # 清理阶段
        print("=== CLEANING UP ===")

        # 停止所有记录器
        for manager in camera_managers:
            try:
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
                if manager.sensor and manager.sensor.is_alive:
                    manager.sensor.destroy()
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

        time.sleep(1.0)  # 确保资源释放
        print("Cleanup completed")

if __name__ == '__main__':
    main()