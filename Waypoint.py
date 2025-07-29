import carla

# 连接Carla服务器
client = carla.Client('localhost', 2000)
world = client.get_world()

# 获取所有推荐的车辆生成点
spawn_points = world.get_map().get_spawn_points()

# 打印前10个生成点作为起点候选
print("可用的起点位置:")
for i, spawn_point in enumerate(spawn_points[:10]):
    print(f"{i}: {spawn_point.location}")

# 选择起点和终点
start_point = spawn_points[0]  # 起点
end_point = spawn_points[9]   # 终点

print(f"\n选择的起点: {start_point.location}")
print(f"选择的终点: {end_point.location}")