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
NUM_SENSOR_VEHICLES = 10   # 感知车辆数量减少到10
NUM_NORMAL_VEHICLES = 0    # 普通车辆数量设为0（确保道路空旷）
SIMULATION_TIME = 3600     # 模拟时长(秒)
IMAGE_WIDTH = 800          # 摄像头图像宽度
IMAGE_HEIGHT = 600         # 摄像头图像高度
LANE_WIDTH = 3.5           # 单车道宽度(米)
FPS = 30                   # 视频帧率

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 7           # 道路总宽度
VEHICLE_LENGTH = 4.5       # 平均车长(米)
VEHICLE_WIDTH = 2.0        # 平均车宽(米)
VEHICLE_SPACING = 1.0      # 车辆间距(米)

# 违停位置
ILLEGAL_PARKING_POINT = carla.Location(x=15.39, y=9.76, z=0)

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
        self.data_file.write("timestamp,x,y,z,speed(km/h)\n")
        
        # 记录状态
        self.is_recording = False
        self.has_recorded = False  # 标记是否已经记录过
        
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
        
        # 每秒记录一次数据
        current_time = time.time()
        if hasattr(self, 'last_record_time'):
            if current_time - self.last_record_time >= 1.0:
                self._record_vehicle_data(vehicle)
                self.last_record_time = current_time
        else:
            self.last_record_time = current_time
            self._record_vehicle_data(vehicle)
    
    def _record_vehicle_data(self, vehicle):
        """记录车辆位置和速度数据"""
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 转换为km/h
        
        self.data_file.write(
            f"{time.time():.3f},"
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
    
    def stop_recording(self):
        """停止记录"""
        if self.is_recording:
            self.is_recording = False
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
        
        # 控制记录状态
        if on_street:
            if not self.recorder.is_recording and not self.recorder.has_recorded:
                self.recorder.start_recording()
                print(f"Started recording for vehicle {self.vehicle.id}")
        else:
            if self.recorder.is_recording:
                self.recorder.stop_recording()
                print(f"Stopped recording for vehicle {self.vehicle.id}")
            return
        
        # 如果正在记录，处理帧
        if self.recorder.is_recording:
            self.recorder.record_frame(image, self.vehicle)
            
        # 检查是否到达需要记录的x位置（保留原有拍照功能）
        current_x = round(vehicle_location.x)
        target_positions = [80, 70, 60, 50, 40, 30, 20, 10, 0]
        for target_x in target_positions:
            if (abs(current_x - target_x) <= 0.5 and 
                target_x not in self.recorded_positions):  # 现在recorded_positions已定义
                self._save_image_and_metadata(image, vehicle_location, target_x)
                self.recorded_positions.add(target_x)  # 添加记录的位置

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
        print(f"Saved image at x={target_x} for sensor vehicle {self.vehicle.id}")
        
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
    """在地图上生成运动车辆（10辆感知车辆，0辆普通车辆）"""
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

    # 获取所有生成点
    all_spawn_points = world.get_map().get_spawn_points()
    random.shuffle(all_spawn_points)  # 随机打乱生成点顺序

    # 计算道路方向向量
    direction = carla.Vector3D(
        x=STREET_END.x - STREET_START.x,
        y=STREET_END.y - STREET_START.y,
        z=0
    )
    road_length = math.sqrt(direction.x ** 2 + direction.y ** 2)
    if road_length > 0:
        direction = direction / road_length  # 单位化向量
    
    # 计算垂直向量（道路右侧方向）
    right_vector = carla.Vector3D(
        x=-direction.y,
        y=direction.x,
        z=0
    )
    
    # 车道偏移距离（半个车道宽度）
    lane_offset_distance = LANE_WIDTH / 2

    # 只生成感知车辆（10辆）
    for i in range(min(NUM_SENSOR_VEHICLES, len(all_spawn_points))):
        spawn_point = all_spawn_points[i]

        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # 随机选择车道（0表示左车道，1表示右车道）
        lane_choice = random.choice([0, 1])
        
        # 根据车道选择调整生成位置
        spawn_location = carla.Location(
            x=spawn_point.location.x,
            y=spawn_point.location.y,
            z=spawn_point.location.z
        )
        
        # 如果是右车道，向道路右侧偏移
        if lane_choice == 1:
            spawn_location.x += right_vector.x * lane_offset_distance
            spawn_location.y += right_vector.y * lane_offset_distance
        
        # 创建新的生成点
        new_spawn_point = carla.Transform(
            spawn_location,
            spawn_point.rotation
        )

        vehicle = world.try_spawn_actor(blueprint, new_spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 设置固定速度
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, 30)
            traffic_manager.ignore_lights_percentage(vehicle, 0)  # 完全遵守红绿灯
            
            # 设置变道行为（允许变道但频率较低）
            traffic_manager.random_left_lanechange_percentage(vehicle, 0)  # 10%的概率左变道
            traffic_manager.random_right_lanechange_percentage(vehicle, 0)  # 10%的概率右变道
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            print(f"Spawned sensor vehicle {vehicle.id} ({vehicle.type_id}) with camera on {'right' if lane_choice == 1 else 'left'} lane")

    print(f"Successfully spawned {len(moving_vehicles)} moving vehicles "
          f"({NUM_SENSOR_VEHICLES} sensor, {NUM_NORMAL_VEHICLES} normal)")
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
            cloudiness=30, precipitation=0, sun_altitude_angle=70)
        world.set_weather(weather)

        # 初始化交通管理器
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(1.5)  # 与前车保持距离
        traffic_manager.set_random_device_seed(42)  # 固定随机种子

        # 生成违停车辆（5辆）
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（10辆感知车辆，0辆普通车辆）
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager)

        # 主循环
        start_time = time.time()
        print(f"Simulation started with {NUM_SENSOR_VEHICLES} sensor and {NUM_NORMAL_VEHICLES} normal vehicles")

        while time.time() - start_time < SIMULATION_TIME:
            world.tick()  # 推进仿真

        print("Simulation finished")

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        # 清理阶段
        print("Cleaning up...")

        # 停止所有记录器
        for manager in camera_managers:
            try:
                manager.recorder.stop_recording()
            except:
                pass

        # 恢复异步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # 销毁所有摄像头
        for manager in camera_managers:
            try:
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

if __name__ == '__main__':
    main()