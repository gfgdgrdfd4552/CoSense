import carla

# 连接到CARLA服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
# 步骤1：先运行这个脚本获取大致坐标范围
world = client.get_world()
map_name = world.get_map().name
spawn_points = world.get_map().get_spawn_points()

print(f"\n地图 {map_name} 信息:")
print(f"总生成点数量: {len(spawn_points)}")
print("X轴范围: {:.2f} 到 {:.2f}".format(
    min(p.location.x for p in spawn_points),
    max(p.location.x for p in spawn_points)))
print("Y轴范围: {:.2f} 到 {:.2f}".format(
    min(p.location.y for p in spawn_points),
    max(p.location.y for p in spawn_points)))

# 步骤2：根据输出调整下面这些值
STREET_START = carla.Location(x=-37.06, y=26, z=0)    # 街道起点
STREET_END = carla.Location(x=76.38, y=26, z=0)     # 街道终点
STREET_WIDTH = 30.0                              # 街道宽度

# 验证街道上的生成点
street_points = [p for p in spawn_points 
                if (STREET_START.x <= p.location.x <= STREET_END.x and 
                    abs(p.location.y) <= STREET_WIDTH/2)]
print(f"\n选择的街道上有 {len(street_points)} 个生成点")