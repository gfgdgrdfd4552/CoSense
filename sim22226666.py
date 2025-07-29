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
NUM_PARKED_VEHICLES = 5    # 违停车辆数量
NUM_SENSOR_VEHICLES = 15   # 感知车辆数量
NUM_NORMAL_VEHICLES = 85   # 普通车辆数量
SIMULATION_TIME = 3600     # 模拟时长(秒)
IMAGE_WIDTH = 800          # 摄像头图像宽度
IMAGE_HEIGHT = 600         # 摄像头图像高度
FPS = 30                   # 视频帧率
GREEN_LIGHT_DURATION = 120.0  # 绿灯持续时间

# 车辆参数（使用更现实的值）
VEHICLE_LENGTH = 4.5       # 平均车长(米)
VEHICLE_WIDTH = 2.0        # 平均车宽(米)
VEHICLE_SPAWN_MIN_DISTANCE = 5.0  # 车辆生成最小间距（合理值）
VEHICLE_FOLLOW_DISTANCE = 2.5     # 车辆跟随距离（合理值）
VEHICLE_MIN_SPEED = 10.0   # 最低车速(km/h)

# 道路坐标
STREET_START = carla.Location(x=80, y=15.2, z=0)
STREET_END = carla.Location(x=-10, y=15.2, z=0)
STREET_DIRECTION = carla.Vector3D(
    x=STREET_END.x - STREET_START.x,
    y=STREET_END.y - STREET_START.y,
    z=0
)
road_length = math.sqrt(STREET_DIRECTION.x**2 + STREET_DIRECTION.y**2)
if road_length > 0:
    STREET_DIRECTION = STREET_DIRECTION / road_length  # 单位化

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

# 目标路径关键点
TARGET_WAYPOINTS = [
    carla.Location(x=80, y=15.2, z=0),
    carla.Location(x=60, y=15.2, z=0),
    carla.Location(x=40, y=15.2, z=0),
    carla.Location(x=20, y=15.2, z=0),
    carla.Location(x=0, y=15.2, z=0),
    carla.Location(x=-10, y=15.2, z=0),
]

# 目标道路前方的交通信号灯位置
TARGET_STREET_LIGHT_POSITIONS = [
    carla.Location(x=70, y=15.2, z=0),
    carla.Location(x=60, y=15.2, z=0),
]

def clear_existing_actors(world, actor_types=None):
    """清除地图上指定类型的actors"""
    if actor_types is None:
        actor_types = ['vehicle.*', 'walker.*']
    
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
    
    time.sleep(2.0)
    return total_destroyed

class StreetMonitor:
    """道路监控器"""

    @staticmethod
    def calculate_parking_positions():
        """生成违停车辆位置"""
        parking_positions = []
        yaw = math.degrees(math.atan2(STREET_DIRECTION.y, STREET_DIRECTION.x))
        
        # 使用合理的间距
        for i in range(NUM_PARKED_VEHICLES):
            offset = STREET_DIRECTION * (i * (VEHICLE_LENGTH + 1.0))  # 1米间距
            
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
        # 简化实现
        point_vec = carla.Vector3D(
            x=location.x - STREET_START.x,
            y=location.y - STREET_START.y,
            z=0
        )
        
        # 计算在道路方向上的投影
        projection_length = point_vec.x * STREET_DIRECTION.x + point_vec.y * STREET_DIRECTION.y
        perpendicular_dist = math.sqrt(abs(point_vec.x**2 + point_vec.y**2 - projection_length**2))
        
        return 0 < projection_length < road_length and perpendicular_dist < 5.0

class VehicleRecorder:
    """车辆数据记录器"""
    
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.save_dir = f'output/camera_images/vehicle_{vehicle_id}'
        os.makedirs(self.save_dir, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        video_path = f'{self.save_dir}/recording_{timestamp}.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(video_path, fourcc, FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))
        
        self.data_file = open(f'{self.save_dir}/vehicle_data.txt', 'w')
        self.data_file.write("frame_number,relative_time(s),x,y,z,speed(km/h)\n")
        
        self.is_recording = False
        self.has_recorded = False
        self.start_time = None
        self.frame_count = 0
        self.data_count = 0
        
    def record_frame(self, image, vehicle):
        if not self.is_recording:
            return
            
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.video_writer.write(array)
        
        self.frame_count += 1
        
        if self.frame_count % 3 == 0:
            self._record_vehicle_data(vehicle)
    
    def _record_vehicle_data(self, vehicle):
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        relative_time = self.frame_count / FPS
        
        self.data_count += 1
        self.data_file.write(
            f"{self.frame_count},{relative_time:.3f},"
            f"{location.x:.3f},{location.y:.3f},{location.z:.3f},"
            f"{speed:.2f}\n"
        )
        self.data_file.flush()
    
    def start_recording(self):
        if not self.has_recorded:
            self.is_recording = True
            self.has_recorded = True
            self.start_time = time.time()
            self.frame_count = 0
            self.data_count = 0
            print(f"Started recording for vehicle {self.vehicle_id}")
    
    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            duration = self.frame_count / FPS
            print(f"Stopped recording for vehicle {self.vehicle_id}. "
                  f"Duration: {duration:.2f}s, Frames: {self.frame_count}")
            self.video_writer.release()
            self.data_file.close()

class VehicleCameraManager:
    """车辆摄像头管理器"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.sensor = None
        self.recorder = VehicleRecorder(vehicle.id)
        self.world = vehicle.get_world()
        
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        bp.set_attribute('fov', '100')

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        self.sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: self._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return

        vehicle_location = self.vehicle.get_location()
        on_street = StreetMonitor.is_on_street(vehicle_location)
        
        # 简化记录逻辑
        if on_street and not self.recorder.is_recording and not self.recorder.has_recorded:
            self.recorder.start_recording()
        
        if self.recorder.is_recording:
            self.recorder.record_frame(image, self.vehicle)
            
            # 在特定位置保存图像
            current_x = round(vehicle_location.x)
            if current_x in [80, 60, 40, 20, 0] and not hasattr(self, f'saved_{current_x}'):
                self._save_image_and_metadata(image, vehicle_location, current_x)
                setattr(self, f'saved_{current_x}', True)

    def _save_image_and_metadata(self, image, vehicle_location, position_x):
        save_dir = f'output/camera_images/vehicle_{self.vehicle.id}'
        os.makedirs(save_dir, exist_ok=True)
        
        image.save_to_disk(f'{save_dir}/position_{position_x}.png')

class VehicleNavigator:
    """车辆导航器"""
    
    def __init__(self, vehicle, world, traffic_manager):
        self.vehicle = vehicle
        self.traffic_manager = traffic_manager
        self.current_target_index = 0
        self.setup_navigation()
        
    def setup_navigation(self):
        try:
            # 设置更合理的车辆行为
            # 使用负值表示比限速快
            self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, -30)  # 比限速快30%
            self.traffic_manager.distance_to_leading_vehicle(self.vehicle, VEHICLE_FOLLOW_DISTANCE)
            self.traffic_manager.auto_lane_change(self.vehicle, False)
            
            # 设置最小速度限制（CARLA 0.9.15支持）
            if hasattr(self.traffic_manager, 'set_vehicle_min_speed'):
                self.traffic_manager.set_vehicle_min_speed(self.vehicle, VEHICLE_MIN_SPEED)
            
        except Exception as e:
            print(f"Navigation setup failed for vehicle {self.vehicle.id}: {e}")
        
    def update_navigation(self):
        # 简化导航更新
        pass

def spawn_parked_vehicles(world):
    parked_vehicles = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # 简化车辆类型过滤
    sedan_blueprints = [bp for bp in vehicle_blueprints if 'sedan' in bp.id or 'audi' in bp.id]
    if not sedan_blueprints:
        sedan_blueprints = vehicle_blueprints

    parking_spots = StreetMonitor.calculate_parking_positions()
    
    for spot in parking_spots:
        blueprint = random.choice(sedan_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spot)
        if vehicle is not None:
            parked_vehicles.append(vehicle)
            vehicle.set_autopilot(False)
            vehicle.set_simulate_physics(False)

    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
    moving_vehicles = []
    camera_managers = []
    navigators = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # 简化车辆类型
    allowed_types = ['vehicle.audi.', 'vehicle.tesla.', 'vehicle.toyota.']
    vehicle_blueprints = [bp for bp in vehicle_blueprints 
                          if any(t in bp.id for t in allowed_types)]

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    
    # 使用所有可用生成点
    for i, spawn_point in enumerate(spawn_points[:NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES]):
        blueprint = random.choice(vehicle_blueprints)
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        
        if vehicle:
            vehicle.set_autopilot(True)
            
            navigator = VehicleNavigator(vehicle, world, traffic_manager)
            navigators.append(navigator)
            
            if i < NUM_SENSOR_VEHICLES:
                camera_managers.append(VehicleCameraManager(vehicle))

    return moving_vehicles, camera_managers, navigators

def extend_green_light_for_target_street(world):
    traffic_lights = world.get_actors().filter('traffic.traffic_light*')
    
    for light in traffic_lights:
        try:
            light_location = light.get_location()
            is_target_light = False
            
            for target_pos in TARGET_STREET_LIGHT_POSITIONS:
                distance = math.sqrt(
                    (light_location.x - target_pos.x)**2 +
                    (light_location.y - target_pos.y)**2
                )
                if distance < 10.0:  # 放宽匹配范围
                    is_target_light = True
                    break
            
            if is_target_light:
                light.set_green_time(GREEN_LIGHT_DURATION)
                light.set_state(carla.TrafficLightState.Green)
                
        except Exception:
            pass

def main():
    parked_vehicles = []
    moving_vehicles = []
    camera_managers = []
    navigators = []
    
    try:
        os.makedirs('output/camera_images', exist_ok=True)
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        map_name = world.get_map().name
        print(f"Connected to CARLA world: {map_name}")

        print("=== CLEARING EXISTING ACTORS ===")
        clear_existing_actors(world)
        
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        weather = carla.WeatherParameters(cloudiness=30, precipitation=0, sun_altitude_angle=70)
        world.set_weather(weather)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(VEHICLE_FOLLOW_DISTANCE)
        traffic_manager.set_random_device_seed(42)
        
        # 启用混合物理模式以提高性能
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_hybrid_physics_radius(70.0)
        
        print("=== EXTENDING GREEN LIGHT DURATION ===")
        extend_green_light_for_target_street(world)

        print("=== SPAWNING PARKED VEHICLES ===")
        parked_vehicles = spawn_parked_vehicles(world)
        
        for _ in range(20):
            world.tick()

        print("=== SPAWNING MOVING VEHICLES ===")
        moving_vehicles, camera_managers, navigators = spawn_moving_vehicles(world, traffic_manager)

        # 替代方法：设置目标道路上车辆的速度差异
        # 通过为每辆车设置速度差异来实现
        for vehicle in moving_vehicles:
            try:
                # 检查车辆是否在目标道路上
                location = vehicle.get_location()
                if StreetMonitor.is_on_street(location):
                    # 设置比限速快30%
                    traffic_manager.vehicle_percentage_speed_difference(vehicle, -30)
            except:
                pass

        print("=== SIMULATION STARTED ===")
        start_time = time.time()
        
        while time.time() - start_time < SIMULATION_TIME:
            world.tick()

        print("Simulation finished")

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        print("=== CLEANING UP ===")
        
        for manager in camera_managers:
            try:
                manager.recorder.stop_recording()
                if manager.sensor.is_alive:
                    manager.sensor.destroy()
            except:
                pass

        for actor in parked_vehicles + moving_vehicles:
            try:
                if actor.is_alive:
                    actor.destroy()
            except:
                pass
        
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except:
            pass
        
        time.sleep(1.0)
        print("Cleanup completed")

if __name__ == '__main__':
    main()