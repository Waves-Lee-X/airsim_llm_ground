import airsim
import numpy as np
from UAV_Autonomous_navigation import AirSimDroneEnv, SACAgent

# 创建环境和智能体
env = AirSimDroneEnv()
agent = SACAgent(state_dim=13, action_dim=3)

# 测试1: 使用默认目标位置
print("\n测试1: 使用默认目标位置")
print(f"当前目标位置: {env.target_position}")

# 测试2: 设置新的目标位置
print("\n测试2: 设置新的目标位置")
new_target = [30.0, 10.0, -2.0]
env.set_target_position(new_target)
print(f"更新后的目标位置: {env.target_position}")

# 测试3: 模拟目标位置在障碍物中的情况
print("\n测试3: 模拟目标位置在障碍物中的情况")
# 这里我们假设当前位置前方有障碍物
# 实际应用中，系统会自动检测并调整目标位置
print("注意: 当目标位置在障碍物中时，系统会自动寻找附近的可行位置")

# 关闭环境
env.client.reset()
env.client.enableApiControl(False)
print("\n测试完成。")