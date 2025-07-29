import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
spectator = world.get_spectator()

client.load_world('Town04')
transform = spectator.get_transform()
location = transform.location
rotation = transform.rotation
print(f"Location: (x={location.x:.2f}, y={location.y:.2f}, z={location.z:.2f})")
print(f"Rotation: (pitch={rotation.pitch:.2f}, yaw={rotation.yaw:.2f}, roll={rotation.roll:.2f})")
world = client.reload_world()
        