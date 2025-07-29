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
NUM_PARKED_VEHICLES = 5     # 违停车辆数量
NUM_SENSOR_VEHICLES = 15    # 感知车辆数量
NUM_NORMAL_VEHICLES = 85    # 普通车辆数量增加以制造拥挤
SIMULATION_TIME = 3600      # 模拟时长(秒)
IMAGE_WIDTH = 800           # 摄像头图像宽度
IMAGE_HEIGHT = 600          # 摄像头图像高度
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

# 红绿灯位置（道路前方）
TRAFFIC_LIGHT_LOCATION = carla.Location(x=90.0, y=5.0, z=5.0)

# 感知车辆间距（确保前方无遮挡）
SENSOR_VEHICLE_SPACING = 30.0  # 感知车辆间距（米）

def setup_traffic_light(world):
    """在道路前方设置红绿灯并设为绿灯120秒"""
    try:
        # 获取地图和所有红绿灯
        carla_map = world.get_map()
        all_traffic_lights = world.get_actors().filter('traffic.traffic_light*')
        
        print(f"Found {len(all_traffic_lights)} traffic lights in the world")
        
        # 寻找最接近目标位置的红绿灯
        target_light = None
        min_distance = float('inf')
        
        for traffic_light in all_traffic_lights:
            light_location = traffic_light.get_location()
            distance = TRAFFIC_LIGHT_LOCATION.distance(light_location)
            
            # 寻找道路前方的红绿灯（x坐标大于80且距离最近）
            if light_location.x > 80 and distance < min_distance:
                min_distance = distance
                target_light = traffic_light
                
        if target_light:
            # 设置红绿灯状态为绿灯，持续120秒
            target_light.set_state(carla.TrafficLightState.Green)
            target_light.set_green_time(120.0)  # 设置绿灯时间为120秒
            
            light_loc = target_light.get_location()
            print(f"✅ Traffic light set to GREEN for 120 seconds at location: x={light_loc.x:.1f}, y={light_loc.y:.1f}")
            
            # 获取受影响的车道路径点
            affected_waypoints = target_light.get_affected_lane_waypoints()
            print(f"Traffic light affects {len(affected_waypoints)} lane waypoints")
            
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
    """车辆摄像头管理器"""

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
            
        # 检查是否到达需要记录的x位置
        current_x = round(vehicle_location.x)
        target_positions = [80, 70, 60, 50, 40, 30, 20, 10, 0]
        for target_x in target_positions:
            if (abs(current_x - target_x) <= 0.5 and 
                target_x not in self.recorded_positions):
                self._save_image_and_metadata(image, vehicle_location, target_x)
                self.recorded_positions.add(target_x)

    def _save_image_and_metadata(self, image, vehicle_location, target_x):
        """保存图片和元数据"""
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
        print(f"Saved image at x={target_x} for sensor vehicle {self.vehicle.id}")

def generate_sensor_spawn_points():
    """为感知车辆生成优化的生成点（分布在道路左侧车道，确保前方无遮挡）"""
    sensor_spawn_points = []
    
    # 将道路分为三个区域：前方、中间、后方
    road_length = STREET_START.x - STREET_END.x  # 90米
    
    # 前方区域：x=80到x=50（30米）- 5辆感知车辆
    front_section_start = 80
    front_section_end = 50
    front_vehicles = 5
    
    # 中间区域：x=50到x=20（30米）- 7辆感知车辆  
    middle_section_start = 50
    middle_section_end = 20
    middle_vehicles = 7
    
    # 后方区域：x=20到x=-10（30米）- 3辆感知车辆
    rear_section_start = 20
    rear_section_end = -10
    rear_vehicles = 3
    
    # 生成前方区域的感知车辆（确保间距足够大）
    for i in range(front_vehicles):
        x = front_section_start - (i * SENSOR_VEHICLE_SPACING) - random.uniform(0, 3)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"Front sensor spawn point: x={x:.1f}, y={LEFT_LANE_Y:.1f}")
    
    # 生成中间区域的感知车辆
    for i in range(middle_vehicles):
        x = middle_section_start - (i * (SENSOR_VEHICLE_SPACING/2)) - random.uniform(0, 3)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"Middle sensor spawn point: x={x:.1f}, y={LEFT_LANE_Y:.1f}")
    
    # 生成后方区域的感知车辆
    for i in range(rear_vehicles):
        x = rear_section_start - (i * (SENSOR_VEHICLE_SPACING/3)) - random.uniform(0, 3)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        sensor_spawn_points.append(spawn_point)
        print(f"Rear sensor spawn point: x={x:.1f}, y={LEFT_LANE_Y:.1f}")
    
    print(f"Generated {len(sensor_spawn_points)} optimized sensor spawn points (all in left lane)")
    return sensor_spawn_points

def generate_normal_spawn_points():
    """为普通车辆生成密集的生成点（集中在右侧车道）"""
    spawn_points = []
    
    # 道路长度
    road_length = STREET_START.x - STREET_END.x  # 90米
    
    # 计算需要的生成点数量（考虑车辆长度和间距）
    vehicle_unit_length = VEHICLE_LENGTH + 0.5  # 拥挤交通间距0.5米
    max_vehicles_per_lane = int(road_length / vehicle_unit_length)  # 每车道最大车辆数
    
    print(f"Road length: {road_length}m, Max normal vehicles per lane: {max_vehicles_per_lane}")
    
    # 为右车道生成密集的生成点（拥挤车辆）
    for i in range(max_vehicles_per_lane):
        x = STREET_START.x - (i * vehicle_unit_length) - random.uniform(0, 1)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=RIGHT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('RIGHT', spawn_point))
    
    # 为左车道生成少量普通车辆（避免遮挡感知车辆）
    left_lane_vehicles = 5
    for i in range(left_lane_vehicles):
        # 在后方区域生成（x值较小）
        x = STREET_START.x - (road_length * 0.8) - (i * (VEHICLE_LENGTH + 10)) - random.uniform(0, 3)
        spawn_point = carla.Transform(
            carla.Location(x=x, y=LEFT_LANE_Y, z=0.1),
            carla.Rotation(yaw=180)
        )
        spawn_points.append(('LEFT', spawn_point))
    
    # 打乱顺序
    random.shuffle(spawn_points)
    
    print(f"Generated {len(spawn_points)} normal vehicle spawn points (mostly in right lane)")
    return spawn_points

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

    # 生成感知车辆的优化生成点（全部在左侧车道）
    sensor_spawn_points = generate_sensor_spawn_points()
    
    # 生成感知车辆（15辆）- 分布在道路的前方、中间、后方
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
            
            # 感知车辆设置较高速度（相对通畅）
            traffic_manager.vehicle_percentage_speed_difference(vehicle, -30)  # 比限速快30%
            traffic_manager.set_desired_speed(vehicle, 50)  # 50km/h
            traffic_manager.distance_to_leading_vehicle(vehicle, 10.0)  # 10米安全距离
            
            # 严格禁用变道行为
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            sensor_vehicles_spawned += 1
            
            # 判断车辆在哪个区域
            x_pos = spawn_point.location.x
            if x_pos >= 50:
                region = "FRONT"
            elif x_pos >= 20:
                region = "MIDDLE" 
            else:
                region = "REAR"
                
            print(f"Spawned sensor vehicle {vehicle.id} in {region} region at x={x_pos:.1f}, y={spawn_point.location.y:.1f}")

    # 生成普通车辆的生成点（大部分在右侧车道）
    normal_spawn_points = generate_normal_spawn_points()
    
    # 分离左右车道的生成点
    left_lane_points = [point for lane, point in normal_spawn_points if lane == 'LEFT']
    right_lane_points = [point for lane, point in normal_spawn_points if lane == 'RIGHT']
    
    print(f"Available normal spawn points - LEFT: {len(left_lane_points)}, RIGHT: {len(right_lane_points)}")

    # 生成普通车辆
    normal_vehicles_spawned = 0
    right_lane_vehicles = int(NUM_NORMAL_VEHICLES * 0.9)  # 90%在右车道
    left_lane_vehicles = NUM_NORMAL_VEHICLES - right_lane_vehicles  # 10%在左车道
    
    # 在右车道生成拥挤车辆
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
            
            # 右车道拥挤设置
            speed_setting = random.uniform(5, 12)  # 5-12km/h，拥挤
            distance_setting = random.uniform(0.5, 1.5)  # 0.5-1.5米，紧密
            
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, speed_setting)
            traffic_manager.distance_to_leading_vehicle(vehicle, distance_setting)
            
            # 严格禁用变道
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 100)  # 保持右车道
            
            right_spawned += 1
            normal_vehicles_spawned += 1

    # 在左车道生成剩余普通车辆（少量，避免遮挡感知车辆）
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
            
            # 左车道通畅设置
            speed_setting = random.uniform(30, 40)  # 30-40km/h
            distance_setting = random.uniform(5.0, 10.0)  # 5-10米
            
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, speed_setting)
            traffic_manager.distance_to_leading_vehicle(vehicle, distance_setting)
            
            # 严格禁用变道
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            traffic_manager.keep_right_rule_percentage(vehicle, 0)  # 保持左车道
            
            left_spawned += 1
            normal_vehicles_spawned += 1

    print(f"\nSuccessfully spawned {len(moving_vehicles)} moving vehicles:")
    print(f"  - {sensor_vehicles_spawned} sensor vehicles (LEFT lane, distributed)")
    print(f"  - {right_spawned} normal vehicles (RIGHT lane - congested)")
    print(f"  - {left_spawned} normal vehicles (LEFT lane - smooth)")
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

        # 设置同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 固定时间步长0.05秒
        world.apply_settings(settings)

        # 设置天气（阴天，增加拥挤感）
        weather = carla.WeatherParameters(
            cloudiness=1, precipitation=0, sun_altitude_angle=30)
        world.set_weather(weather)
        
        # 设置红绿灯为绿灯120秒
        traffic_light = setup_traffic_light(world)
        
        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(0.8)  # 全局安全距离0.8米
        traffic_manager.set_hybrid_physics_mode(True)  # 启用混合物理模式提高性能
        traffic_manager.set_random_device_seed(42)  # 固定随机种子

        # 生成违停车辆（5辆）
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(40):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（在指定道路上）
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        print(f"\nSimulation started on road from x={STREET_START.x} to x={STREET_END.x}")
        print(f"Left lane center: y={LEFT_LANE_Y:.2f}, Right lane center: y={RIGHT_LANE_Y:.2f}")
        print(f"Sensor vehicles ({NUM_SENSOR_VEHICLES}) distributed across FRONT, MIDDLE, and REAR regions in LEFT lane")
        print(f"Normal vehicles ({NUM_NORMAL_VEHICLES}) mostly in RIGHT lane (congested)")
        
        # 记录拥堵开始时间
        congestion_start_time = time.time()
        congestion_duration = 0
        is_congested = False
        
        # 统计变量
        total_recordings_started = 0
        active_recordings = 0
        
        while time.time() - start_time < SIMULATION_TIME:
            world.tick()  # 推进仿真
            
            # 定期检查交通拥堵情况和记录状态
            if time.time() - congestion_start_time > 10:  # 每10秒检查一次
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
                            region = "UNKNOWN"
                            x_pos = vehicle_location.x
                            if x_pos >= 50:
                                region = "FRONT"
                            elif x_pos >= 20:
                                region = "MIDDLE"
                            else:
                                region = "REAR"
                            sensor_positions.append((manager.vehicle.id, x_pos, region))
                
                if left_count > 0 and right_count > 0:
                    avg_left_speed = 3.6 * left_lane_speed / left_count
                    avg_right_speed = 3.6 * right_lane_speed / right_count
                    
                    print(f"\n=== Traffic Status ===")
                    print(f"Lane speeds - LEFT: {avg_left_speed:.1f} km/h ({left_count} vehicles), "
                          f"RIGHT: {avg_right_speed:.1f} km/h ({right_count} vehicles)")
                    print(f"Active recordings: {current_active_recordings}/{len(camera_managers)}")
                    
                    # 显示感知车辆分布
                    front_sensors = len([p for p in sensor_positions if p[2] == "FRONT"])
                    middle_sensors = len([p for p in sensor_positions if p[2] == "MIDDLE"])
                    rear_sensors = len([p for p in sensor_positions if p[2] == "REAR"])
                    
                    print(f"Sensor distribution on road - FRONT: {front_sensors}, MIDDLE: {middle_sensors}, REAR: {rear_sensors}")
                    
                    # 如果右车道平均速度低于15km/h，认为拥堵
                    if avg_right_speed < 15 and not is_congested:
                        print(f"🚨 RIGHT lane congestion detected! Speed: {avg_right_speed:.1f} km/h")
                        is_congested = True
                    elif avg_right_speed >= 15 and is_congested:
                        print(f"✅ RIGHT lane congestion eased. Speed: {avg_right_speed:.1f} km/h")
                        is_congested = False
                        
                    # 更新拥堵持续时间
                    if is_congested:
                        congestion_duration += 10
                        
                    print("=" * 50)

        print("\n🏁 Simulation finished")
        print(f"📊 Total RIGHT lane congestion duration: {congestion_duration} seconds")
        print(f"📹 Total recordings started: {total_recordings_started}")

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
            world.apply_settings(settings)
            print("Restored asynchronous mode")
        except:
            pass

        # 销毁所有摄像头
        for manager in camera_managers:
            try:
                if manager.sensor and manager.sensor.is_alive:
                    manager.sensor.destroy()
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