import os
import time
import random
import math
import carla

# ===== 配置参数 =====
NUM_PARKED_VEHICLES = 20          # 违停车辆数量
VEHICLE_SPACING = 8.0             # 车辆间距(米)
EDGE_OFFSET = 1.5                 # 基础道路边缘偏移(米)
LANE_OFFSET_MULTIPLIER = 1.0      # 额外偏移车道数 (1.0=一个标准车道宽度)
SIMULATION_TIME = 3600            # 模拟时长(秒)
DEBUG_MODE = False                 # 调试可视化开关

# 违停起始点（用户自定义坐标）
ILLEGAL_PARKING_POINT = carla.Location(x=-17.88, y= 214.17, z=3.21)

class ParkingGenerator:
    """动态计算道路边缘停车位"""

    @staticmethod
    def calculate_parking_positions(world):
        """沿道路边缘生成停车位，叠加车道偏移"""
        parking_positions = []
        carla_map = world.get_map()
        
        # 获取起始点道路信息
        start_waypoint = carla_map.get_waypoint(
            ILLEGAL_PARKING_POINT,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
       

        current_waypoint = start_waypoint
        
        for i in range(NUM_PARKED_VEHICLES):
            # 1. 计算道路方向向量
            yaw = current_waypoint.transform.rotation.yaw
            yaw_rad = math.radians(yaw)
            
            # 2. 计算基础边缘偏移（垂直于道路方向）
            edge_vector = carla.Vector3D(
                x = EDGE_OFFSET * math.sin(yaw_rad),
                y = -EDGE_OFFSET * math.cos(yaw_rad),
                z = 0
            )
            
            # 3. 计算车道宽度偏移（关键修改：叠加一个车道宽度）
            lane_width = current_waypoint.lane_width
            lane_offset = carla.Vector3D(
                x = lane_width * LANE_OFFSET_MULTIPLIER * math.sin(yaw_rad),
                y = -lane_width * LANE_OFFSET_MULTIPLIER * math.cos(yaw_rad),
                z = 0
            )
            
            # 4. 合并偏移量
            total_offset = edge_vector - lane_offset*1.25
            
            # 5. 计算生成位置
            spawn_location = current_waypoint.transform.location + total_offset
            spawn_location.z += 0.5  # Z轴缓冲防碰撞
            
            # 存储位置和朝向
            parking_positions.append(carla.Transform(
                spawn_location, 
                current_waypoint.transform.rotation  # 使用道路方向
            ))
            
            # 调试可视化
            if DEBUG_MODE:
                # 标记生成点（蓝色：基础边缘，红色：叠加车道偏移后）
                world.debug.draw_point(
                    current_waypoint.transform.location + edge_vector,
                    size=0.1, 
                    color=carla.Color(0, 0, 255),
                    life_time=SIMULATION_TIME
                )
                world.debug.draw_point(
                    spawn_location, 
                    size=0.1, 
                    color=carla.Color(255, 0, 0),
                    life_time=SIMULATION_TIME
                )
            
            # 获取下一个路径点（沿道路方向）
            next_waypoints = current_waypoint.next(VEHICLE_SPACING)
            current_waypoint = next_waypoints[0] if next_waypoints else current_waypoint
        
        return parking_positions

def spawn_parked_vehicles(world):
    """在计算位置生成违停车辆"""
    parked_vehicles = []
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    
    # 过滤可用车型（轿车类）
    allowed_types = [
        'vehicle.audi.a2', 'vehicle.audi.etron', 
        'vehicle.tesla.model3', 'vehicle.toyota.prius',
        'vehicle.chevrolet.impala', 'vehicle.ford.mustang'
    ]
    vehicle_blueprints = [bp for bp in vehicle_blueprints if bp.id in allowed_types]
    
    # 获取停车位
    parking_spots = ParkingGenerator.calculate_parking_positions(world)
    
    # 创建输出目录
    os.makedirs('output', exist_ok=True)
    with open('output/parking_coordinates.txt', 'w') as f:
        f.write("vehicle_id,x,y,z,yaw,lane_width\n")
    
    # 生成车辆
    for i, spot in enumerate(parking_spots):
        blueprint = random.choice(vehicle_blueprints)
        if blueprint.has_attribute('color'):
            colors = blueprint.get_attribute('color').recommended_values
            blueprint.set_attribute('color', random.choice(colors))
        
        # 三级高度防碰撞生成
        vehicle = None
        for z_offset in [0.0, 0.3, 0.6]:
            adjusted_spot = carla.Transform(
                carla.Location(spot.location.x, spot.location.y, spot.location.z + z_offset),
                spot.rotation
            )
            vehicle = world.try_spawn_actor(blueprint, adjusted_spot)
            if vehicle: 
                break
        
        if vehicle:
            # ==== 防漂移核心设置 ====
            vehicle.set_simulate_physics(True)      # 禁用物理引擎
            vehicle.apply_control(carla.VehicleControl(hand_brake=True))  # 强制手刹
             # ===== 关键修复：等待状态更新 =====
            for _ in range(5):  # 确保物理引擎完成计算
                world.tick()  # 必须调用tick更新状态
                time.sleep(0.05)  # 允许物理引擎计算
            # 记录坐标（含车道宽度）
            transform = vehicle.get_transform()
            waypoint = world.get_map().get_waypoint(transform.location)
            lane_width = waypoint.lane_width if waypoint else 0.0
            with open('output/parking_coordinates.txt', 'a') as f:
                f.write(f"{vehicle.id},"
                        f"{transform.location.x:.4f},"
                        f"{transform.location.y:.4f},"
                        f"{transform.location.z:.4f},"
                        f"{transform.rotation.yaw:.2f},"
                        f"{lane_width:.2f}\n")
            
            # 可视化车辆朝向
            if DEBUG_MODE:
                arrow_start = transform.location
                arrow_end = arrow_start + carla.Location(
                    x=3 * math.cos(math.radians(transform.rotation.yaw)),
                    y=3 * math.sin(math.radians(transform.rotation.yaw)),
                    z=0
                )
                world.debug.draw_arrow(
                    arrow_start, arrow_end,
                    thickness=0.05,
                    arrow_size=0.1,
                    color=carla.Color(0, 255, 0),
                    life_time=SIMULATION_TIME
                )
            
            parked_vehicles.append(vehicle)
            print(f"✅ 生成车辆 {i+1}/{len(parking_spots)} 位置: ({spot.location.x:.2f}, {spot.location.y:.2f})")
        else:
            print(f"❌ 生成失败: ({spot.location.x:.2f}, {spot.location.y:.2f})")
    
    return parked_vehicles

def main():
    parked_vehicles = []
    
    try:
        # 连接CARLA服务端
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        
        # 设置同步模式（确保精确帧控制）
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        # 标记起始点（红色大球）
        if DEBUG_MODE:
            world.debug.draw_point(
            ILLEGAL_PARKING_POINT, 
            size=0.3, 
            color=carla.Color(255, 0, 0), 
            life_time=SIMULATION_TIME
        )
        
        # 生成违停车辆
        parked_vehicles = spawn_parked_vehicles(world)
        world.tick()  # 触发状态更新
        
        print(f"\n🚗 成功生成 {len(parked_vehicles)} 辆违停车辆")
        print(f"⏱️ 仿真运行中... (持续 {SIMULATION_TIME} 秒)")
        
        # 主循环
        start_time = time.time()
        while time.time() - start_time < SIMULATION_TIME:
            world.tick()
            time.sleep(0.05)
            
    except Exception as e:
        print(f"❌ 运行时错误: {str(e)}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\n🧹 清理中...")
        destroyed_count = 0
        for vehicle in parked_vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
                destroyed_count += 1
        
        # 恢复异步模式
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except:
            pass
            
        print(f"销毁 {destroyed_count} 辆车")
        print("✅ 清理完成")

if __name__ == '__main__':
    main()