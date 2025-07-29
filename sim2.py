import os
import time
import random
import weakref
import math
import carla
import json

# 常量定义
NUM_PARKED_VEHICLES = 20        # 路边停放车辆数量
NUM_SENSOR_VEHICLES = 1        # 感知车辆数量（安装摄像头）
NUM_NORMAL_VEHICLES = 99       # 普通车辆数量（不安装摄像头） 
SIMULATION_TIME = 7200         # 模拟时长(秒)
IMAGE_WIDTH = 800              # 摄像头图像宽度
IMAGE_HEIGHT = 600             # 摄像头图像高度
DETECTION_DISTANCE = 30.0      # 检测停止车辆的距离(米)
RIGHT_ANGLE_RANGE = 60         # 右前方角度范围(度)
PHOTO_INTERVAL = 1.0           # 拍照最小间隔时间(秒)
VEHICLE_SPEED = 10.0           # 车辆速度(m/s)

Detection_Times = {}          # 记录停止车辆的首次检测时间 {vehicle_id: detection_time}

# 停止车辆的固定位置列表
PARKED_VEHICLE_POSITIONS = [
    carla.Transform(carla.Location(x=-0.75, y=31.55, z=0.1), carla.Rotation(yaw=0)),
    carla.Transform(carla.Location(x=-38.60, y=-23.40, z=0.1), carla.Rotation(yaw=-90)),
    carla.Transform(carla.Location(x=44.90, y=10.30, z=0.1), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=-71.70, y=10.10, z=0.1), carla.Rotation(yaw=0)),
    carla.Transform(carla.Location(x=6.05, y=72.90, z=0.1), carla.Rotation(yaw=180)),

    carla.Transform(carla.Location(x=-72.45, y=30.90, z=0.1), carla.Rotation(yaw=0)),
    carla.Transform(carla.Location(x=-55.40, y=-19.60, z=0.1), carla.Rotation(yaw=90)),
    carla.Transform(carla.Location(x=15.65, y=10.30, z=0.1), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=66.70, y=31.20, z=0.1), carla.Rotation(yaw=0)),
    carla.Transform(carla.Location(x=12.30, y=62.90, z=0.1), carla.Rotation(yaw=180)),

    carla.Transform(carla.Location(x=95.95, y=96.70, z=0.1), carla.Rotation(yaw=90)),
    carla.Transform(carla.Location(x=67.45, y=63.25, z=0.1), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=42.60, y=127.95, z=0.1), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=-38.95, y=92.60, z=0.1), carla.Rotation(yaw=-90)),
    carla.Transform(carla.Location(x=-101.25, y=56.60, z=0.1), carla.Rotation(yaw=-90)),

    carla.Transform(carla.Location(x=112.05, y=77.45, z=0.1), carla.Rotation(yaw=-90)),
    carla.Transform(carla.Location(x=77.55, y=73.05, z=0.1), carla.Rotation(yaw=0)),
    carla.Transform(carla.Location(x=50.40, y=144.05, z=0.1), carla.Rotation(yaw=0)),
    carla.Transform(carla.Location(x=-55.05, y=77.00, z=0.1), carla.Rotation(yaw=90)),
    carla.Transform(carla.Location(x=-117.00, y=53.70, z=0.1), carla.Rotation(yaw=90)),
]

class VehicleCameraManager:
    """车辆摄像头管理器（仅用于感知车辆）"""

    def __init__(self, vehicle, simulation_start_time):
        self.vehicle = vehicle
        self.sensor = None
        self.world = vehicle.get_world()
        self.last_photo_time = 0  # 上次拍照时间
        self.simulation_start_time = simulation_start_time  # 模拟开始时间
      
        
        # 初始化摄像头
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        bp.set_attribute('fov', '110')  # 视野110度

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))  # 摄像头安装位置
        self.sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)

        # 设置图像回调函数
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: self._parse_image(weak_self, image))

    def _is_on_right_front(self, sensor_vehicle, parked_vehicle):
        """检查停止车辆是否在感知车辆的右前方"""
        # 获取感知车辆的位置和朝向
        sensor_transform = sensor_vehicle.get_transform()
        sensor_location = sensor_transform.location
        sensor_yaw = sensor_transform.rotation.yaw
        
        # 获取停止车辆的位置
        parked_location = parked_vehicle.get_location()
        
        # 计算从感知车辆指向停止车辆的向量
        vector = carla.Vector3D(
            x=parked_location.x - sensor_location.x,
            y=parked_location.y - sensor_location.y,
            z=0
        )
        
        # 计算向量长度
        distance = math.sqrt(vector.x**2 + vector.y**2)
        if distance == 0:
            return False
        
        # 单位化向量
        vector.x /= distance
        vector.y /= distance
        
        # 计算向量与感知车辆前进方向的夹角
        forward_vector = carla.Vector3D(
            x=math.cos(math.radians(sensor_yaw)),
            y=math.sin(math.radians(sensor_yaw)),
            z=0
        )
        
        # 计算叉积确定左右关系 (z分量为正表示在右侧)
        cross_product = forward_vector.x * vector.y - forward_vector.y * vector.x
        
        # 计算点积确定前后关系
        dot_product = forward_vector.x * vector.x + forward_vector.y * vector.y
        
        # 计算角度
        angle = math.degrees(math.acos(dot_product))
        
        # 在右前方条件：在右侧(叉积z>0)且在前方(角度<90)且在角度范围内
        return (cross_product > 0 and 
                angle < 90 and 
                angle <= RIGHT_ANGLE_RANGE/2)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self or not self.vehicle.is_alive:
            return

        current_time = time.time()
        # 使用相对于模拟开始的时间（秒）
        elapsed_time = current_time - self.simulation_start_time
        
        # 检查拍照间隔
        if elapsed_time - self.last_photo_time < PHOTO_INTERVAL:
            return

        # 获取感知车辆
        sensor_vehicle = self.vehicle
        
        # 获取所有停止车辆
        parked_vehicles = [actor for actor in self.world.get_actors().filter('vehicle.*') 
                          if not actor.is_alive or actor.attributes.get('role_name') == 'parked']
        
        # 检查是否接近任何停止车辆且在右前方
        for parked_vehicle in parked_vehicles:
            parked_location = parked_vehicle.get_location()
            distance = sensor_vehicle.get_location().distance(parked_location)
            
            if (distance <= DETECTION_DISTANCE and 
                self._is_on_right_front(sensor_vehicle, parked_vehicle)):
                # 满足拍照条件
                self.last_photo_time = elapsed_time
                
                # 检查是否是第一次检测到该停止车辆
                if parked_vehicle.id not in Detection_Times:
                    detection_time = elapsed_time
                    Detection_Times[parked_vehicle.id] = detection_time
                    print(f"First detection: Sensor vehicle {sensor_vehicle.id} detected parked vehicle {parked_vehicle.id} "
                          f"at {detection_time:.2f}s, distance {distance:.2f}m")
                    
                    # 检查是否80%停止车辆都已被检测到
                    if len(Detection_Times) >= NUM_PARKED_VEHICLES*0.8:
                        avg_detection_time = sum(Detection_Times.values()) / NUM_PARKED_VEHICLES
                        print(f"{len(Detection_Times)} parked vehicles detected! Average detection time: {avg_detection_time:.2f}s")
                
                # # 创建保存目录
                # save_dir = 'output/camera_images'
                # os.makedirs(save_dir, exist_ok=True)
                
                # # 生成文件名 - 使用经过的秒数（保留2位小数）
                # elapsed_seconds = round(elapsed_time, 2)
                # filename = f"sensor_{sensor_vehicle.id}_parked_{parked_vehicle.id}_time_{elapsed_seconds}"
                
                # # 保存图像
                # image_path = os.path.join(save_dir, f"{filename}.png")
                # image.save_to_disk(image_path)
                
                # # 保存元数据
                # metadata = {
                #     'elapsed_seconds': elapsed_seconds,  # 使用经过的秒数
                #     'sensor_vehicle_id': sensor_vehicle.id,
                #     'parked_vehicle_id': parked_vehicle.id,
                #     'distance': distance,
                #     'sensor_location': {
                #         'x': sensor_vehicle.get_location().x,
                #         'y': sensor_vehicle.get_location().y,
                #         'z': sensor_vehicle.get_location().z
                #     },
                #     'parked_location': {
                #         'x': parked_location.x,
                #         'y': parked_location.y,
                #         'z': parked_location.z
                #     },
                #     'sensor_yaw': sensor_vehicle.get_transform().rotation.yaw,
                # }
                
                # metadata_path = os.path.join(save_dir, f"{filename}_meta.json")
                # with open(metadata_path, 'w') as f:
                #     json.dump(metadata, f)
                
                break

def spawn_parked_vehicles(world):
    """在固定位置生成停放车辆"""
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

    for i, spot in enumerate(PARKED_VEHICLE_POSITIONS[:NUM_PARKED_VEHICLES]):
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        
        # 设置角色名称以便识别
        blueprint.set_attribute('role_name', 'parked')

        # 尝试生成车辆
        vehicle = world.try_spawn_actor(blueprint, spot)
        if vehicle is not None:
            parked_vehicles.append(vehicle)
            vehicle.set_autopilot(False)
            vehicle.set_simulate_physics(False)  # 禁用物理模拟

            print(f"Spawned parked vehicle {vehicle.id} at position: "
                  f"({spot.location.x:.2f}, {spot.location.y:.2f}) "
                  f"orientation: {spot.rotation.yaw:.1f}°")
        else:
            print(f"Failed to spawn vehicle at position ({spot.location.x:.2f}, {spot.location.y:.2f})")

    print(f"Successfully spawned {len(parked_vehicles)} parked vehicles at fixed positions")
    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager, simulation_start_time):
    """在地图上生成运动车辆（分为感知车辆和普通车辆）"""
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

    # 首先生成感知车辆（安装摄像头）
    for i in range(min(NUM_SENSOR_VEHICLES, len(all_spawn_points))):
        spawn_point = all_spawn_points[i]

        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        
        # 设置角色名称以便识别
        blueprint.set_attribute('role_name', 'sensor')

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 设置固定速度10m/s
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, VEHICLE_SPEED)
            traffic_manager.ignore_lights_percentage(vehicle, 0)  # 完全遵守红绿灯
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle, simulation_start_time))
            print(f"Spawned sensor vehicle {vehicle.id} ({vehicle.type_id}) with camera")

    # 然后生成普通车辆（不安装摄像头）
    for i in range(NUM_SENSOR_VEHICLES, min(NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES, len(all_spawn_points))):
        spawn_point = all_spawn_points[i]

        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        
        # 设置角色名称以便识别
        blueprint.set_attribute('role_name', 'normal')

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 设置固定速度10m/s
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, VEHICLE_SPEED)
            traffic_manager.ignore_lights_percentage(vehicle, 0)  # 完全遵守红绿灯
            # print(f"Spawned normal vehicle {vehicle.id} ({vehicle.type_id})")

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

        # 记录模拟开始时间
        simulation_start_time = time.time()

        # 生成停放车辆
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（返回车辆列表和摄像头管理器列表）
        moving_vehicles, camera_managers = spawn_moving_vehicles(world, traffic_manager, simulation_start_time)

        # 主循环
        print(f"Simulation started with {NUM_SENSOR_VEHICLES} sensor and {NUM_NORMAL_VEHICLES} normal vehicles")
        while time.time() - simulation_start_time < SIMULATION_TIME:
            world.tick()  # 推进仿真

        print("Simulation finished")

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        # 清理阶段
        print("Cleaning up...")

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