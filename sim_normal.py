import os
import numpy
import time
import random
import weakref
import csv
import math
import carla

# 常量定义
NUM_PARKED_VEHICLES = 3  # 路边停放车辆数量
NUM_MOVING_VEHICLES = 50  # 运动车辆数量
SIMULATION_TIME = 3600  # 模拟时长(秒)
IMAGE_WIDTH = 800  # 摄像头图像宽度
IMAGE_HEIGHT = 600  # 摄像头图像高度
CAMERA_FREQ = 1.0  # 图像采集频率(秒)
THROUGHPUT_RECORD_INTERVAL = 1.0  # 吞吐量记录间隔(秒)
LANE_WIDTH = 3.5  # 单车道宽度(米)

# 道路坐标(单向二车道)
# STREET_START = carla.Location(x=-43.5, y=0, z=0)  # 道路起点
# STREET_END = carla.Location(x=-43.5, y=-40, z=0)  # 道路终点

STREET_START = carla.Location(x=-30, y=26.3, z=0)  # 道路起点
STREET_END = carla.Location(x=30, y=26.3, z=0)  # 道路终点

STREET_WIDTH = 7  # 道路总宽度
PARKING_OFFSET = 5.0  # 停车位距道路中心线偏移量
VEHICLE_LENGTH = 4.5  # 平均车长(米)

YAW_RATE_LIST = []  # 全局偏航角速度列表

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
        max_vehicles = min(NUM_PARKED_VEHICLES+3, int(road_length / spacing))

        # 从起点开始生成连续停车位
        for i in range(3, max_vehicles):
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
    """车辆摄像头管理器"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.sensor = None
        self.image_count = 0
        self.last_capture_time = 0
        self.recording = False
        self.last_yaw = 0  # 上一帧偏航角
        self.yaw_rate_data = []  # 偏航角速度数据存储
  
        # 初始化摄像头
        world = self.vehicle.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        bp.set_attribute('fov', '110')  # 视野110度

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))  # 摄像头安装位置

        self.sensor = world.spawn_actor(bp, transform, attach_to=self.vehicle)

        # 设置图像回调函数
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: self._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self or not self.recording or not self.vehicle.is_alive:
            return

        current_time = time.time()
        if current_time - self.last_capture_time >= CAMERA_FREQ:
            self.last_capture_time = current_time
            self.image_count += 1

            # 获取当前摄像头变换矩阵
            camera_transform = self.sensor.get_transform()
            current_yaw = camera_transform.rotation.yaw  # 当前偏航角

            # 按车辆ID创建子目录
            vehicle_id = self.vehicle.id
            save_dir = f'output/camera_images/vehicle_{vehicle_id}'
            os.makedirs(save_dir, exist_ok=True)

            # 保存图像
            image.save_to_disk(f'{save_dir}/{self.image_count:05d}.png')
         
            if self.image_count != 1:  # 从第二帧开始计算角速度
                time_diff = 1  # 固定1秒间隔
                yaw_diff = current_yaw - self.last_yaw
                yaw_rate = yaw_diff / time_diff  # 偏航角速度(度/秒)
                
                # 存储数据[帧编号, 角速度]
                self.yaw_rate_data.append([
                    self.image_count,  
                    yaw_rate  
                ])
                YAW_RATE_LIST.append(abs(yaw_rate))  # 记录绝对值到全局列表
                
            self.last_yaw = current_yaw  # 更新上一帧角度

    def save_angle_data(self, vehicle_id):
        """保存偏航角速度数据到CSV文件"""
        if len(self.yaw_rate_data) > 0:
            filename = f'output/camera_angles/vehicle_{vehicle_id}.csv'
            os.makedirs('output/camera_angles', exist_ok=True)
            
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Time(s)", "YawRate(deg/s)"])
                for data in self.yaw_rate_data:
                    writer.writerow(data)

            print(f"Saved camera angle data to {filename}")

class TrafficThroughputRecorder:
    """交通流量记录器"""

    def __init__(self):
        self.passed_vehicles = set()  # 已通过车辆集合
        self.throughput_data = []  # 流量数据
        self.start_time = time.time()  # 开始时间

    def vehicle_passed(self, vehicle_id, timestamp):
        """记录通过道路的车辆"""
        if vehicle_id not in self.passed_vehicles:
            self.passed_vehicles.add(vehicle_id)
            elapsed_time = int(timestamp - self.start_time)
            self.throughput_data.append((elapsed_time, len(self.passed_vehicles)))
            print(f"Vehicle {vehicle_id} passed at {elapsed_time}s - Total: {len(self.passed_vehicles)}")

    def save_to_csv(self, filename="throughput_data.csv"):
        """保存流量数据到CSV文件"""
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Cumulative Vehicle Count"])
            for data in self.throughput_data:
                writer.writerow(data)
        print(f"Throughput data saved to {filename}")


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
        'vehicle.volkswagen.t2'  # 如果需要也可以移除
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
    """在地图上随机生成运动车辆"""
    moving_vehicles = []
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

    for i in range(min(NUM_MOVING_VEHICLES, len(all_spawn_points))):
        spawn_point = all_spawn_points[i]

        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            moving_vehicles.append(vehicle)
            vehicle.set_autopilot(True)

            # traffic_manager.vehicle_percentage_speed_difference(vehicle, -50)  # 设置速度
            # 设置车辆速度为10m/s (36km/h)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)  # 0表示不减速
            traffic_manager.set_desired_speed(vehicle, 10)  # 设置目标速度为10m/s

    print(f"Spawned {len(moving_vehicles)} moving vehicles randomly on map")
    return moving_vehicles

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
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)  # 与前车保持2.5米距离
        traffic_manager.set_random_device_seed(42)  # 固定随机种子
        # traffic_manager.global_percentage_speed_difference(-50)  # 负值表示加速

        # 生成停放车辆
        parked_vehicles = spawn_parked_vehicles(world)

        # 等待1秒让车辆稳定
        time.sleep(1.0)

        # 生成运动车辆
        moving_vehicles = spawn_moving_vehicles(world, traffic_manager)

        # 为所有运动车辆安装摄像头
        for vehicle in moving_vehicles:
            camera_managers.append(VehicleCameraManager(vehicle))
            print(f"Camera deployed on vehicle: {vehicle.id} ({vehicle.type_id})")

        # 创建流量记录器
        throughput_recorder = TrafficThroughputRecorder()

        # 主循环
        start_time = time.time()
        print(f"Simulation started. Will run for {SIMULATION_TIME} seconds.")

        checked_vehicles = set()  # 已检查车辆集合

        while time.time() - start_time < SIMULATION_TIME:
            # 更新摄像头记录状态
            for manager in camera_managers:
                if manager.vehicle.is_alive:
                    current_location = manager.vehicle.get_location()
                    manager.recording = StreetMonitor.is_on_street(current_location)

            # 检查通过的车辆
            for vehicle in moving_vehicles:
                if vehicle.is_alive and vehicle.id not in checked_vehicles:
                    if StreetMonitor.is_on_street(vehicle.get_location()):
                        throughput_recorder.vehicle_passed(vehicle.id, time.time())
                        checked_vehicles.add(vehicle.id)

            world.tick()  # 推进仿真

        # 保存所有摄像头角度数据
        for manager in camera_managers:
            if manager.vehicle.is_alive:
                manager.save_angle_data(manager.vehicle.id)

        print(f"Simulation finished. Total vehicles passed: {len(throughput_recorder.passed_vehicles)}")
        yaw_rate_avg = numpy.mean(YAW_RATE_LIST)  # 计算平均偏航角速度
        print(f"Avg yaw rate: {yaw_rate_avg}")

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
                manager.recording = False
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