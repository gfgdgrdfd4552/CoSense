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

# 常量定义
NUM_PARKED_VEHICLES = 0
NUM_SENSOR_VEHICLES = 10     # 减少感知车辆数量以降低负载
NUM_NORMAL_VEHICLES = 0
SIMULATION_TIME = 3600        # 模拟时长(秒)
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
LANE_WIDTH = 3.5
FPS = 30

# 目标道路参数
STREET_START = carla.Location(x=80, y=15.2, z=0)
STREET_END = carla.Location(x=-10, y=15.2, z=0)
STREET_WIDTH = 7
PARKING_OFFSET = 5.0
VEHICLE_LENGTH = 4.5

# 停放车辆位置（固定）
PARKED_VEHICLE_LOCATION = carla.Location(x=35.0, y=15.2 - PARKING_OFFSET, z=0.1)
PARKED_VEHICLE_ROTATION = carla.Rotation(yaw=180.0)  # 车头朝西

class StreetMonitor:
    @staticmethod
    def is_on_street(location):
        line_vec = carla.Vector3D(x=STREET_END.x - STREET_START.x, y=STREET_END.y - STREET_START.y, z=0)
        point_vec = carla.Vector3D(x=location.x - STREET_START.x, y=location.y - STREET_START.y, z=0)
        line_len = math.sqrt(line_vec.x ** 2 + line_vec.y ** 2)
        if line_len == 0:
            return False
        t = max(0, min(1, (point_vec.x * line_vec.x + point_vec.y * line_vec.y) / line_len ** 2))
        projection = carla.Location(x=STREET_START.x + t * line_vec.x, y=STREET_START.y + t * line_vec.y, z=0)
        dist = math.sqrt((location.x - projection.x) ** 2 + (location.y - projection.y) ** 2)
        return dist <= STREET_WIDTH / 2 and 0 < t < 1

class VehicleRecorder:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.save_dir = f'output/camera_images/vehicle_{vehicle_id}'
        os.makedirs(self.save_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        video_path = f'{self.save_dir}/recording_{timestamp}.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(video_path, fourcc, FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.data_file = open(f'{self.save_dir}/vehicle_data.txt', 'w')
        self.data_file.write("timestamp,x,y,z,speed(km/h)\n")
        self.is_recording = False
        self.has_recorded = False
        self.last_record_time = 0

    def record_frame(self, image, vehicle):
        if not self.is_recording:
            return
        
        # 转换图像格式
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))[:, :, :3][:, :, ::-1]
        self.video_writer.write(array)
        
        # 每秒记录一次车辆数据
        current_time = time.time()
        if current_time - self.last_record_time >= 1.0:
            location = vehicle.get_location()
            velocity = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            self.data_file.write(f"{current_time:.3f},{location.x:.3f},{location.y:.3f},{location.z:.3f},{speed:.2f}\n")
            self.data_file.flush()
            self.last_record_time = current_time

    def start_recording(self):
        if not self.has_recorded:
            self.is_recording = True
            self.has_recorded = True
            self.last_record_time = time.time()

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            self.video_writer.release()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器（仅用于感知车辆）"""

    def __init__(self, vehicle, parked_vehicle_ref):
        self.vehicle = vehicle
        self.parked_vehicle_ref = parked_vehicle_ref  # 弱引用到停放车辆
        self.sensor = None
        self.recorded_positions = set()
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        self.image_queue = Queue()
        
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
        
        # 获取停放车辆（如果还存在）
        parked_vehicle = self.parked_vehicle_ref()
        
        # 控制记录状态
        if on_street and parked_vehicle:
            # 计算到停放车辆的距离
            parked_location = parked_vehicle.get_location()
            distance = math.sqrt((vehicle_location.x - parked_location.x)**2 + 
                                (vehicle_location.y - parked_location.y)**2)
            
            # 当接近停放车辆时开始录制
            if distance < 50.0:  # 50米范围内开始录制
                if not self.recorder.is_recording and not self.recorder.has_recorded:
                    self.recorder.start_recording()
                    print(f"Started recording for vehicle {self.vehicle.id} (distance to parked: {distance:.1f}m)")
            else:
                if self.recorder.is_recording:
                    self.recorder.stop_recording()
                    print(f"Stopped recording for vehicle {self.vehicle.id} (too far: {distance:.1f}m)")
        else:
            if self.recorder.is_recording:
                self.recorder.stop_recording()
                print(f"Stopped recording for vehicle {self.vehicle.id} (off street or parked vehicle gone)")
            return
        
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
        
        # 记录所有车辆（包括停放车辆）
        for actor in all_actors:
            loc = actor.get_location()
            rotation = actor.get_transform().rotation
            vehicle_data.append({
                'id': actor.id,
                'type': 'parked' if actor.id == self.parked_vehicle_ref().id else 'moving',
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
            'camera_vehicle_id': self.vehicle.id,
            'camera': {
                'x': camera_transform.location.x,
                'y': camera_transform.location.y,
                'z': camera_transform.location.z,
                'yaw': camera_transform.rotation.yaw,
                'pitch': camera_transform.rotation.pitch,
                'roll': camera_transform.rotation.roll,
                'fov': 100,        # 与摄像头蓝图设置一致
                'max_range': 100.0   # 探测最大半径100米
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

def spawn_parked_vehicle(world):
    """在固定位置生成停放车辆"""
    transform = carla.Transform(PARKED_VEHICLE_LOCATION, PARKED_VEHICLE_ROTATION)
    blueprint_library = world.get_blueprint_library()
    
    # 选择停放车辆类型
    parked_vehicle_types = [
        'vehicle.audi.etron',
        'vehicle.mercedes.coupe',
        'vehicle.tesla.model3',
        'vehicle.bmw.grandtourer'
    ]
    
    # 尝试生成停放车辆
    for vehicle_type in parked_vehicle_types:
        blueprint = blueprint_library.find(vehicle_type)
        if blueprint:
            if blueprint.has_attribute('color'):
                colors = blueprint.get_attribute('color').recommended_values
                blueprint.set_attribute('color', random.choice(colors))
            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle:
                vehicle.set_autopilot(False)
                vehicle.set_simulate_physics(False)
                print(f"Spawned parked vehicle at ({PARKED_VEHICLE_LOCATION.x}, {PARKED_VEHICLE_LOCATION.y})")
                return vehicle
    
    # 如果找不到特定车辆类型，使用任意车辆
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    sedan_blueprints = [bp for bp in vehicle_blueprints if 'sedan' in bp.id or 'audi' in bp.id]
    if not sedan_blueprints:
        sedan_blueprints = vehicle_blueprints
    blueprint = random.choice(sedan_blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    vehicle = world.try_spawn_actor(blueprint, transform)
    if vehicle:
        vehicle.set_autopilot(False)
        vehicle.set_simulate_physics(False)
        print(f"Spawned parked vehicle (fallback) at ({PARKED_VEHICLE_LOCATION.x}, {PARKED_VEHICLE_LOCATION.y})")
        return vehicle
    else:
        print("Failed to spawn parked vehicle.")
        return None

def spawn_moving_vehicles(world, traffic_manager, parked_vehicle_ref):
    """生成移动车辆，并传入停放车辆的弱引用"""
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt',
        'vehicle.bmw.grandtourer', 'vehicle.chevrolet.impala', 'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020', 'vehicle.ford.crown', 'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon', 'vehicle.lincoln.mkz_2020', 'vehicle.mercedes.coupe_2020',
        'vehicle.micro.microlino', 'vehicle.mini.cooperst', 'vehicle.mustang.mustang',
        'vehicle.nissan.micra', 'vehicle.nissan.patrol_2021', 'vehicle.seat.leon',
        'vehicle.tesla.model3', 'vehicle.toyota.prius'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]

    # 在目标道路上创建生成点（不同车道）
    spawn_points = []
    
    # 创建多个车道
    num_lanes = 3
    base_y = STREET_START.y
    start_x = STREET_START.x
    end_x = STREET_END.x
    
    # 为每个车道创建生成点
    for lane in range(num_lanes):
        # 计算当前车道的y偏移
        y_offset = (lane - (num_lanes-1)/2) * LANE_WIDTH
        
        # 创建该车道的多个生成点
        for i in range(3):  # 每个车道3辆车
            x_pos = start_x - i * 20  # 间隔20米
            location = carla.Location(x=x_pos, y=base_y + y_offset, z=0.1)
            rotation = carla.Rotation(yaw=180.0)  # 车辆朝西行驶
            spawn_points.append(carla.Transform(location, rotation))
    
    random.shuffle(spawn_points)
    vehicles = []
    camera_managers = []
    
    for i in range(min(NUM_SENSOR_VEHICLES, len(spawn_points))):
        spawn_point = spawn_points[i]
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle:
            vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, -20)  # 稍微提高速度
            traffic_manager.set_desired_speed(vehicle, 40)  # 40 km/h
            # 传入停放车辆的弱引用
            camera_managers.append(VehicleCameraManager(vehicle, parked_vehicle_ref))
    
    return vehicles, camera_managers

def change_weather(world, weather_index):
    """改变天气条件"""
    weather_presets = [
        carla.WeatherParameters.ClearNoon,
        carla.WeatherParameters.CloudyNoon,
        carla.WeatherParameters.WetNoon,
        carla.WeatherParameters.WetCloudyNoon,
        carla.WeatherParameters.MidRainyNoon,
        carla.WeatherParameters.HardRainNoon,
        carla.WeatherParameters.SoftRainNoon,
        carla.WeatherParameters.ClearSunset,
        carla.WeatherParameters.CloudySunset,
        carla.WeatherParameters.WetSunset
    ]
    
    # 确保索引在范围内
    weather_index = weather_index % len(weather_presets)
    weather = weather_presets[weather_index]
    
    # 设置天气并添加一些变化
    weather.cloudiness = random.uniform(20, 80)
    weather.precipitation = random.uniform(0, 80)
    weather.wetness = random.uniform(0, 80)
    world.set_weather(weather)
    
    print(f"Weather changed to preset {weather_index}: {weather}")
    return weather_index + 1

def main():
    parked_vehicle = None
    parked_vehicle_ref = None
    moving_vehicles = []
    camera_managers = []
    
    try:
        os.makedirs('output/camera_images', exist_ok=True)
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # 初始化天气
        weather_index = 5
        weather_index = change_weather(world, weather_index)
        last_weather_change = time.time()

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        traffic_manager.set_random_device_seed(42)
        traffic_manager.set_hybrid_physics_mode(True)

        # 生成停放车辆
        parked_vehicle = spawn_parked_vehicle(world)
        if parked_vehicle:
            parked_vehicle_ref = weakref.ref(parked_vehicle)
        else:
            print("Warning: No parked vehicle spawned")
            parked_vehicle_ref = lambda: None  # 空引用

        # 等待世界更新
        for _ in range(20):
            world.tick()

        # 生成移动车辆
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager, parked_vehicle_ref)
        
        start_time = time.time()
        while time.time() - start_time < SIMULATION_TIME:
            # 每分钟更换天气
            current_time = time.time()
            if current_time - last_weather_change > 60:  # 60秒
                weather_index = change_weather(world, weather_index)
                last_weather_change = current_time
            
            world.tick()
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        # 停止所有记录器
        for manager in camera_managers:
            try:
                manager.recorder.stop_recording()
                if manager.sensor and manager.sensor.is_alive:
                    manager.sensor.destroy()
            except Exception as e:
                print(f"Error cleaning camera manager: {str(e)}")
        
        # 销毁所有车辆
        for actor in [parked_vehicle] + moving_vehicles:
            try:
                if actor and actor.is_alive:
                    actor.destroy()
            except Exception as e:
                print(f"Error destroying actor: {str(e)}")
        
        # 恢复异步模式
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(False)
        except:
            pass
        
        print("Simulation completed")


if __name__ == '__main__':
    main()