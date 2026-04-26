import airsim

# 连接到AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# 获取所有对象的列表
print("正在列出环境中的所有对象...")
objects = client.simListSceneObjects()
print(f"找到 {len(objects)} 个对象:")
for obj in objects:
    print(f"  - {obj}")

# 尝试获取TargetPoint的位置
print("\n尝试获取TargetPoint的位置...")
try:
    target_pose = client.simGetObjectPose("Point_A")
    print(f"TargetPoint位置: x={target_pose.position.x_val}, y={target_pose.position.y_val}, z={target_pose.position.z_val}")
except Exception as e:
    print(f"获取TargetPoint位置失败: {e}")

# 尝试获取一些常见的目标点名称
common_target_names = ["Target", "Goal", "Destination", "Waypoint", "EndPoint"]
print("\n尝试获取常见目标点名称的位置...")
for name in common_target_names:
    try:
        target_pose = client.simGetObjectPose(name)
        print(f"{name}位置: x={target_pose.position.x_val}, y={target_pose.position.y_val}, z={target_pose.position.z_val}")
    except Exception as e:
        print(f"获取{name}位置失败: {e}")

# 关闭连接
client.reset()
print("\n测试完成。")