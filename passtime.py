import carla
import time
import math
import random
import sys
import atexit

# 连接Carla服务器
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # 获取世界对象（使用已加载的Town10HD地图）
    world = client.get_world()
    
    # 设置调试模式
    debug = world.debug

except Exception as e:
    print(f"连接Carla服务器失败: {e}")
    sys.exit(1)

# 设置起点和终点坐标
start_location = carla.Location(x=-64.644844, y=24.471010, z=0.600000)
end_location = carla.Location(x=-114.588699, y=70.059807, z=0.600000)

# 全局变量用于资源清理
vehicle = None
trajectory_points = []
actors_to_destroy = []

# 注册退出处理函数
def cleanup():
    print("\n执行资源清理...")
    if vehicle and vehicle.is_alive:
        vehicle.destroy()
    
    # 清理所有生成的actor
    for actor in actors_to_destroy:
        if actor and actor.is_alive:
            actor.destroy()
    
    print("资源清理完成")

atexit.register(cleanup)

try:
    # 获取蓝图库和地图
    blueprint_library = world.get_blueprint_library()
    map = world.get_map()

    # 生成车辆
    vehicle_bp = blueprint_library.find('vehicle.audi.a2')
    start_transform = carla.Transform(start_location, carla.Rotation(yaw=0))
    
    try:
        vehicle = world.spawn_actor(vehicle_bp, start_transform)
        actors_to_destroy.append(vehicle)
        print(f"车辆生成成功，ID: {vehicle.id}")
    except Exception as e:
        print(f"车辆生成失败: {e}")
        sys.exit(1)

    # 设置车辆基本参数
    vehicle.set_simulate_physics(True)

    # 获取起点和终点的路点
    start_waypoint = map.get_waypoint(start_location)
    end_waypoint = map.get_waypoint(end_location)

    # 可视化起点和终点
    debug.draw_string(start_location, "START", draw_shadow=False, 
                     color=carla.Color(r=0, g=255, b=0), life_time=100.0)
    debug.draw_string(end_location, "END", draw_shadow=False, 
                     color=carla.Color(r=255, g=0, b=0), life_time=100.0)

    # 路径跟随函数，增加变道概率
    def enhanced_path_following(vehicle, end_waypoint, lane_change_prob=0.1):
        try:
            start_time = time.time()
            current_waypoint = map.get_waypoint(vehicle.get_location())
            
            # 生成全局路径（确保经过相同街道）
            global_path = generate_global_path(current_waypoint, end_waypoint)
            
            # 可视化全局路径
            visualize_path(global_path, color=carla.Color(0, 255, 0))
            
            path_index = 0
            last_lane_change_time = 0
            
            while path_index < len(global_path):
                try:
                    current_location = vehicle.get_location()
                    current_waypoint = map.get_waypoint(current_location)
                    
                    # 存储轨迹点用于可视化
                    trajectory_points.append(current_location)
                    if len(trajectory_points) % 5 == 0:
                        visualize_trajectory(trajectory_points)
                    
                    # 检查是否到达当前目标点
                    target_waypoint = global_path[path_index]
                    if current_location.distance(target_waypoint.transform.location) < 3.0:
                        path_index += 1
                        if path_index >= len(global_path):
                            break
                        target_waypoint = global_path[path_index]
                    
                    # 随机变道逻辑
                    current_time = time.time()
                    if current_time - last_lane_change_time > 2.0:
                        if random.random() < lane_change_prob:
                            # 尝试变道
                            left_lane = current_waypoint.get_left_lane()
                            right_lane = current_waypoint.get_right_lane()
                            
                            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                                target_waypoint = left_lane
                                last_lane_change_time = current_time
                                print(f"变道到左侧车道 {time.strftime('%H:%M:%S')}")
                            elif right_lane and right_lane.lane_type == carla.LaneType.Driving:
                                target_waypoint = right_lane
                                last_lane_change_time = current_time
                                print(f"变道到右侧车道 {time.strftime('%H:%M:%S')}")
                    
                    # 计算控制指令
                    control = calculate_vehicle_control(vehicle, target_waypoint)
                    vehicle.apply_control(control)
                    
                    # 检查是否被中断
                    time.sleep(0.05)
                
                except KeyboardInterrupt:
                    print("\n检测到用户中断，准备退出...")
                    return None
                except Exception as e:
                    print(f"路径跟随过程中出错: {e}")
                    return None
            
            end_time = time.time()
            return end_time - start_time
        
        except Exception as e:
            print(f"路径跟随函数出错: {e}")
            return None

    # 生成全局路径（确保经过相同街道）
    def generate_global_path(start_waypoint, end_waypoint):
        try:
            # 这里使用简化的直线路径生成
            # 实际项目中应该使用Carla的全局路径规划器
            
            path = []
            direction = end_waypoint.transform.location - start_waypoint.transform.location
            distance = direction.length()
            direction = direction.make_unit_vector()
            
            steps = int(distance / 2.0)  # 每2米一个路点
            
            for i in range(steps):
                next_location = start_waypoint.transform.location + direction * (i * 2.0)
                next_waypoint = map.get_waypoint(next_location)
                if next_waypoint:
                    path.append(next_waypoint)
            
            path.append(end_waypoint)
            return path
        
        except Exception as e:
            print(f"路径生成失败: {e}")
            return []

    # 计算车辆控制指令
    def calculate_vehicle_control(vehicle, target_waypoint):
        try:
            current_transform = vehicle.get_transform()
            current_location = current_transform.location
            current_rotation = current_transform.rotation
            
            target_location = target_waypoint.transform.location
            
            # 计算方向和距离
            direction = target_location - current_location
            distance = direction.length()
            direction = direction.make_unit_vector()
            
            # 计算转向角度
            forward_vector = current_transform.get_forward_vector()
            angle = math.atan2(direction.y, direction.x) - math.atan2(forward_vector.y, forward_vector.x)
            angle = math.degrees(angle)
            
            # 归一化角度到[-180, 180]
            while angle > 180:
                angle -= 360
            while angle < -180:
                angle += 360
            
            # 创建控制指令
            control = carla.VehicleControl()
            
            # 根据距离调整油门
            if distance > 10:
                control.throttle = 0.75
            elif distance > 5:
                control.throttle = 0.5
            else:
                control.throttle = 0.3
            
            # 设置转向
            control.steer = angle / 100.0  # 归一化
            
            # 根据情况刹车
            if distance < 2.0:
                control.brake = 0.5
            else:
                control.brake = 0.0
            
            return control
        
        except Exception as e:
            print(f"控制计算失败: {e}")
            return carla.VehicleControl()  # 返回空控制

    # 可视化路径
    def visualize_path(path, color=carla.Color(255, 0, 0), life_time=100.0):
        try:
            for i in range(len(path)-1):
                debug.draw_line(
                    path[i].transform.location + carla.Location(z=0.5),
                    path[i+1].transform.location + carla.Location(z=0.5),
                    thickness=0.1, color=color, life_time=life_time)
        except Exception as e:
            print(f"路径可视化失败: {e}")

    # 可视化轨迹
    def visualize_trajectory(points, color=carla.Color(0, 0, 255), life_time=30.0):
        try:
            for i in range(len(points)-1):
                debug.draw_line(
                    points[i] + carla.Location(z=0.3),
                    points[i+1] + carla.Location(z=0.3),
                    thickness=0.05, color=color, life_time=life_time)
        except Exception as e:
            print(f"轨迹可视化失败: {e}")

    # 执行路径跟随
    print("开始路径跟随...")
    print(f"起点: {start_location}")
    print(f"终点: {end_location}")
    
    travel_time = enhanced_path_following(vehicle, end_waypoint, lane_change_prob=0.1)
    
    if travel_time is not None:
        print(f"车辆从起点行驶到终点耗时: {travel_time:.2f}秒")
    else:
        print("路径跟随被中断或出错")

    # 最终可视化完整轨迹
    visualize_trajectory(trajectory_points, color=carla.Color(255, 255, 0), life_time=300.0)

except KeyboardInterrupt:
    print("\n检测到用户中断，准备退出...")
except Exception as e:
    print(f"程序运行出错: {e}")
finally:
    cleanup()
    print("程序结束")