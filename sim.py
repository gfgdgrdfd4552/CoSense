import os
import time
import random
import weakref
import math
import carla
import json

# 常量定义
NUM_PARKED_VEHICLES = 13  # 路边停放车辆数量
NUM_SENSOR_VEHICLES = 0   # 感知车辆数量（安装摄像头）
NUM_NORMAL_VEHICLES = 0  # 普通车辆数量（不安装摄像头） 
SIMULATION_TIME = 7200    # 模拟时长(秒)
IMAGE_WIDTH = 800         # 摄像头图像宽度
IMAGE_HEIGHT = 600        # 摄像头图像高度
LANE_WIDTH = 3.5          # 单车道宽度(米)

# 道路坐标(单向二车道)
STREET_START = carla.Location(x=80, y=15.2, z=0)  # 道路起点
STREET_END = carla.Location(x=-10, y=15.2, z=0)   # 道路终点
STREET_WIDTH = 7          # 道路总宽度
PARKING_OFFSET = 5.0      # 停车位距道路中心线偏移量
VEHICLE_LENGTH = 4.5      # 平均车长(米)

class StreetMonitor:
    """道路监控器"""

    @staticmethod
    def calculate_parking_positions():
        """在道路右侧计算连续的停车位位置"""
        parking_positions = []

        # 计算道路方向向量和长度
        direction = carla.Vector3D(
            x=STREET_END.x - STREET_START.x,
            y=STREET_END.y - STREET_START.y,
            z=0
        )
        road_length = math.sqrt(direction.x ** 2 + direction.y ** 2)
        direction = direction / road_length  # 单位化向量

        # 计算右侧垂直向量(道路右侧)
        right_vector = carla.Vector3D(
            x=-direction.y,
            y=direction.x,
            z=0
        )
        # 向右偏移(1.5倍车道宽度)
        right_offset = right_vector * (LANE_WIDTH * 1.5)

        # 计算停车位间距(考虑车长和间隙)
        spacing = VEHICLE_LENGTH * 1.3  # 车辆间保留30%间隙

        # 计算实际可停放车辆数量
        max_vehicles = min(NUM_PARKED_VEHICLES, int(road_length / spacing))

        # 从起点开始生成连续停车位
        for i in range(max_vehicles):
            # 沿道路方向的位移
            along_road = direction * (i * spacing)

            # 计算停车位中心位置
            center_pos = carla.Location(
                x=STREET_START.x + along_road.x,
                y=STREET_START.y + along_road.y,
                z=0.1  # 轻微抬高避免地面碰撞
            )

            # 偏移到右侧路边
            parking_pos = carla.Location(
                x=center_pos.x + right_offset.x,
                y=center_pos.y + right_offset.y,
                z=0.1
            )

            # 计算朝向(与道路方向相同)
            yaw = math.degrees(math.atan2(direction.y, direction.x))

            parking_positions.append(
                carla.Transform(parking_pos, carla.Rotation(yaw=yaw))
            )

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

class VehicleCameraManager:
    """车辆摄像头管理器（仅用于感知车辆）"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.sensor = None
        self.recorded_positions = set()  # 已记录的x位置集合
        self.world = vehicle.get_world()
        
        # 初始化摄像头
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        bp.set_attribute('fov', '100')  # 视野110度

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
        if not StreetMonitor.is_on_street(vehicle_location):
            return

        # 检查是否到达需要记录的x位置
        current_x = round(vehicle_location.x)
        target_positions = [80, 70, 60, 50, 40, 30, 20, 10, 0]
        for target_x in target_positions:
            # 如果接近目标位置(±0.5米范围内)且未记录过
            if (abs(current_x - target_x) <= 0.5 and 
                target_x not in self.recorded_positions):
                # 按车辆ID创建子目录
                vehicle_id = self.vehicle.id
                save_dir = f'output/camera_images/vehicle_{vehicle_id}'
                os.makedirs(save_dir, exist_ok=True)

                # 记录位置
                self.recorded_positions.add(target_x)
                
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
                print(f"Saved image at x={target_x} for sensor vehicle {vehicle_id}")

def spawn_parked_vehicles(world):
    """在道路右侧生成停放车辆"""
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

            print(f"Spawned parked vehicle {i + 1} at position: "
                  f"({spot.location.x:.2f}, {spot.location.y:.2f}) "
                  f"orientation: {spot.rotation.yaw:.1f}°")
        else:
            print(f"Failed to spawn vehicle at position ({spot.location.x:.2f}, {spot.location.y:.2f})")

    print(f"Successfully spawned {len(parked_vehicles)} parked vehicles on road right side")
    return parked_vehicles

def spawn_moving_vehicles(world, traffic_manager):
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

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 设置固定速度
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, 30)
            traffic_manager.ignore_lights_percentage(vehicle, 0)  # 完全遵守红绿灯
            
            # 安装摄像头
            camera_managers.append(VehicleCameraManager(vehicle))
            print(f"Spawned sensor vehicle {vehicle.id} ({vehicle.type_id}) with camera")

    # 然后生成普通车辆（不安装摄像头）
    for i in range(NUM_SENSOR_VEHICLES, min(NUM_SENSOR_VEHICLES + NUM_NORMAL_VEHICLES, len(all_spawn_points))):
        spawn_point = all_spawn_points[i]

        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)
            
            # 设置随机速度
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
            traffic_manager.set_desired_speed(vehicle, random.uniform(25, 30))
            traffic_manager.ignore_lights_percentage(vehicle, 0)  # 完全遵守红绿灯
            # traffic_manager.set_desired_speed(vehicle, 10)
            print(f"Spawned normal vehicle {vehicle.id} ({vehicle.type_id})")

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

        # 生成停放车辆
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        for _ in range(20):  # 20 ticks at 0.05s = 1 second
            world.tick()

        # 生成运动车辆（返回车辆列表和摄像头管理器列表）
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