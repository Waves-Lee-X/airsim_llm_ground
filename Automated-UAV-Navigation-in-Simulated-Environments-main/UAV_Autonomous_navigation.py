import airsim
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import torch
import torch.nn as nn
import torch.optim as optim
from filterpy.kalman import KalmanFilter
import time
import torch.nn.functional as F
import random

# --- 第一步：定义AirSim Gym环境 ---
class AirSimDroneEnv(gym.Env):
    def __init__(self):
        super(AirSimDroneEnv, self).__init__()
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        self.avoid_direction = None

        self.fixed_altitude = -2.0  
        self.action_space = spaces.Box(low=-2, high=2, shape=(3,), dtype=np.float32)  
        self.observation_space = spaces.Box(low=-100, high=100, shape=(13,), dtype=np.float32)  

        # 用于障碍物预测的卡尔曼滤波器
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.kf.P *= 1000  # 初始不确定性
        self.kf.R *= 5  # 测量噪声
        self.obstacle_velocity = np.array([0, 0])
        self.target_position = self.get_target_position()

    def get_target_position(self):
        # 直接使用固定的默认目标位置，避免依赖环境中的TargetPoint对象
        # 这样可以确保系统能够正常运行，不受环境配置的影响
        default_target = np.array([71.7, 11.6, self.fixed_altitude])
        print(f"使用默认目标位置: {default_target}")
        return default_target
    
    def set_target_position(self, target_pos):
        """设置目标位置"""
        self.target_position = np.array(target_pos)
        print(f"目标位置已设置为: {self.target_position}")
        
        # 验证目标位置是否可行
        if self.is_target_in_obstacle():
            print("警告: 目标位置可能在障碍物中，正在寻找附近的可行位置...")
            self.target_position = self.find_nearest_valid_target()
            print(f"已调整目标位置为: {self.target_position}")
    
    def is_target_in_obstacle(self):
        """检查目标位置是否在障碍物中"""
        # 从目标位置获取激光雷达数据来检查是否有障碍物
        try:
            # 暂时使用当前位置的激光雷达数据进行简单判断
            # 实际应用中应该在目标位置附近进行检测
            front_distance, _, _, _, _ = self.get_lidar_data()
            # 如果前方距离过近，认为目标位置可能在障碍物中
            return front_distance < 1.0
        except Exception as e:
            print(f"检查目标位置时出错: {e}")
            return False
    
    def find_nearest_valid_target(self):
        """寻找最近的可行目标位置"""
        # 从当前位置向目标方向搜索，找到第一个可行的位置
        drone_state = self.client.getMultirotorState().kinematics_estimated.position
        current_pos = np.array([drone_state.x_val, drone_state.y_val, self.fixed_altitude])
        
        direction = self.target_position - current_pos
        if np.linalg.norm(direction) < 1e-6:
            return current_pos
        
        direction = direction / np.linalg.norm(direction)
        
        # 沿着方向搜索，步长为1米
        for i in range(1, 50):
            test_pos = current_pos + direction * i
            # 这里可以添加更复杂的障碍物检测逻辑
            # 暂时返回当前位置前方10米的位置
            return current_pos + direction * 10
        
        # 如果找不到可行位置，返回默认位置
        return np.array([50.0, 0.0, self.fixed_altitude])
    
    def land_at_target(self):
        """ 在目标位置降落无人机 """
        print("Landing at target location...")
        self.client.landAsync().join()
        print("Drone has landed successfully!")

    def get_lidar_data(self):
        lidar_data = self.client.getLidarData(lidar_name="LidarSensor1", vehicle_name="Drone1")
        
        if len(lidar_data.point_cloud) < 3:
            return 10.0, 0.5, -0.5, 0.0, 0.0  # 使用合理的默认值

        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        
        # 过滤无效点（距离过远或过近的点）
        valid_points = points[np.logical_and(points[:, 0] > 0.1, points[:, 0] < 100)]
        if len(valid_points) == 0:
            return 10.0, 0.5, -0.5, 0.0, 0.0

        # 定义区域（前方、左侧、右侧）
        front_obstacles = valid_points[(valid_points[:, 0] > 0) & (np.abs(valid_points[:, 1]) < 0.5)]  
        left_obstacles = valid_points[valid_points[:, 1] > 0.5]
        right_obstacles = valid_points[valid_points[:, 1] < -0.5]

        # 计算距离
        front_dist = np.min(front_obstacles[:, 0]) if len(front_obstacles) > 0 else 10.0
        left_dist = np.min(left_obstacles[:, 1]) if len(left_obstacles) > 0 else 0.5  # 左侧可用空间
        right_dist = np.max(right_obstacles[:, 1]) if len(right_obstacles) > 0 else -0.5  # 右侧可用空间，取最大值

        # 确保距离值在合理范围内
        front_dist = np.clip(front_dist, 0.1, 100)
        left_dist = np.clip(left_dist, 0.1, 10)
        right_dist = np.clip(right_dist, -10, -0.1)

        print(f"Front: {front_dist}, Left: {left_dist}, Right: {right_dist}")  
        if len(front_obstacles) > 0:
            left_edge = np.min(front_obstacles[:, 1])  # 最小Y值（最左侧点）
            right_edge = np.max(front_obstacles[:, 1])  # 最大Y值（最右侧点）
        else:
            left_edge = 0.0
            right_edge = 0.0

        return front_dist, left_dist, right_dist, left_edge, right_edge


    def predict_obstacle_position(self, new_position):
        # 增强的卡尔曼滤波器预测
        self.kf.predict()
        self.kf.update(new_position)
        
        # 预测多个时间步的位置
        predicted_positions = []
        for t in [0.5, 1.0, 2.0]:
            # 预测t秒后的位置
            predicted_x = self.kf.x[0] + self.kf.x[2] * t + 0.5 * self.kf.x[4] * t**2 if len(self.kf.x) > 4 else self.kf.x[0] + self.kf.x[2] * t
            predicted_y = self.kf.x[1] + self.kf.x[3] * t + 0.5 * self.kf.x[5] * t**2 if len(self.kf.x) > 5 else self.kf.x[1] + self.kf.x[3] * t
            predicted_positions.append(np.array([predicted_x, predicted_y]))
        
        # 使用2.0秒的预测作为主要预测
        predicted_position = predicted_positions[2]
        self.obstacle_velocity = self.kf.x[2:4]
        
        # 计算预测置信度
        # 基于滤波器的协方差矩阵计算不确定性
        position_covariance = np.sqrt(self.kf.P[0, 0] + self.kf.P[1, 1])
        confidence = max(0, 1.0 - position_covariance / 10.0)  # 归一化到0-1
        
        # 打印多时间步预测结果
        print(f"Predicted positions (0.5s, 1.0s, 2.0s): {[pos.round(2) for pos in predicted_positions]}")
        print(f"Prediction confidence: {confidence:.2f}")
        
        return predicted_position

    def step(self, action):
        drone_state = self.client.getMultirotorState().kinematics_estimated.position
        current_pos = np.array([drone_state.x_val, drone_state.y_val, self.fixed_altitude])
        target_x, target_y = self.target_position[:2]
        vx = float(action[0])  # 将NumPy值转换为Python浮点数
        vy = float(action[1])  # 将NumPy值转换为Python浮点数
        x_difference = abs(current_pos[0] - target_x)
    
        if x_difference < 2.0:  # 如果x接近，优先向目标移动
            print(x_difference)
            vy = np.sign(target_y - current_pos[1]) * abs(vy)  # 向目标y方向移动
            vx = np.sign(target_x - current_pos[0]) * abs(vx) 
        
        # 以给定速度移动无人机
        self.client.moveByVelocityAsync(vx, vy, 0, 1).join()
        front_distance, left_free_space, right_free_space,left_edgef,right_edgef = self.get_lidar_data()
        
        # 处理front_distance为NaN的情况
        if np.isnan(front_distance):
            front_distance = 10.0  # 使用较大的默认值
            print("警告: front_distance为NaN，使用默认值10.0")
        
        obstacle_position = np.array([current_pos[0] + front_distance, current_pos[1]])
        predicted_obstacle = self.predict_obstacle_position(obstacle_position)
        
        # 确保obstacle_velocity不包含NaN值
        if np.isnan(self.obstacle_velocity).any():
            self.obstacle_velocity = np.array([0.0, 0.0])
            print("警告: obstacle_velocity包含NaN值，重置为[0.0, 0.0]")
        
        # 处理left_edgef和right_edgef为NaN的情况
        if np.isnan(left_edgef):
            left_edgef = 0.0
        if np.isnan(right_edgef):
            right_edgef = 0.0
        
        new_state = np.concatenate([
            current_pos, 
            self.target_position, 
            [front_distance, left_free_space, right_free_space],
            self.obstacle_velocity.flatten(),
            [left_edgef,right_edgef]
        ])

        distance_to_target = np.linalg.norm(self.target_position - current_pos)
        reward = -distance_to_target
        ## **2. 向正确方向移动的奖励**
        movement_vector = np.array([vx, vy])
        target_direction = self.target_position[:2] - current_pos[:2]
        target_direction /= (np.linalg.norm(target_direction) + 1e-6)  
        alignment_score = np.dot(movement_vector, target_direction)  
        reward += alignment_score * 2  # 向正确方向移动的奖励

        ## **3. 在安全走廊内的额外奖励（避免不必要的偏离）**
        if abs(current_pos[1] - target_y) < 2.0:  
            reward += 3.0  # 保持对齐的奖励

        ## **4. 平滑避障的奖励**
        if front_distance > 5.0:  
            reward += 5.0  # 安全路径
        elif front_distance < 2.0:  
            reward -= 10.0  # 靠得太近的惩罚

        if left_free_space > 0.5 or right_free_space < -0.5:
            reward += 3.0  # 鼓励移动到开阔空间

        ## **5. 保持动量并避免不必要停止的奖励**
        if np.linalg.norm(movement_vector) > 0.5:
            reward += 2.0  # 不要过度减速的奖励
        terminated = distance_to_target < 1.0
        if terminated:
            self.land_at_target()
        truncated = False
        
        # 生成info字典，包含避障相关信息
        info = {
            "collision_count": 0,  # 可以根据实际碰撞检测逻辑更新
            "log": ""  # 用于存储避障相关的日志信息
        }
        
        # 检查是否执行了避障动作
        if front_distance < 3.0:
            if front_distance < 1.5:
                info["log"] = "CRITICAL: Very close to obstacle! Emergency backing up."
            elif front_distance < 3.0:
                info["log"] = "Warning: Close to obstacle! Preparing avoidance."
        
        return new_state, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        self.target_position = self.get_target_position()
        drone_state = self.client.getMultirotorState().kinematics_estimated.position
        front_distance, left_free_space, right_free_space, left_edgef, right_edgef = self.get_lidar_data()
        
        # 处理front_distance为NaN的情况
        if np.isnan(front_distance):
            front_distance = 10.0
        if np.isnan(left_free_space):
            left_free_space = 0.5
        if np.isnan(right_free_space):
            right_free_space = -0.5
        if np.isnan(left_edgef):
            left_edgef = 0.0
        if np.isnan(right_edgef):
            right_edgef = 0.0
            
        # 重置障碍物速度
        self.obstacle_velocity = np.array([0.0, 0.0])
        
        current_pos = np.array([drone_state.x_val, drone_state.y_val, self.fixed_altitude])
        new_state = np.concatenate([
            current_pos, 
            self.target_position, 
            [front_distance, left_free_space, right_free_space],
            self.obstacle_velocity.flatten(),
            [left_edgef, right_edgef]
        ])
        
        return new_state, {}
    
class ExpertPolicy:
    def __init__(self):
        self.moving_avoid_direction = None
        self.moving_avoid_counter = 0  # 移动障碍物避障的冷却时间
        self.static_avoid_direction = None
        self.static_avoid_counter = 0 
        self.avoid_attempts = 0  # 避障尝试次数
        self.last_avoid_direction = None  # 上一次避障方向
        self.obstacle_history = []  # 障碍物历史位置
        self.path_memory = []  # 路径记忆
        self.distance_history = []  # 距离历史，用于检测距离变化
        self.avoid_direction_history = []  # 避障方向历史

# --- 第二步：专家动作函数 ---
    def expert_action(self,state):
        current_pos = np.array(state[:3])
        target_pos = np.array(state[3:6])
        front_distance = state[6]
        left_free_space = state[7]
        right_free_space = state[8]
        obstacle_velocity = np.array(state[9:11])
        left_edge, right_edge = state[11], state[12]
        

        direction_to_target = target_pos - current_pos
        direction_to_target[2] = 0
        direction_to_target /= np.linalg.norm(direction_to_target) + 1e-6
        distance_to_target = np.linalg.norm(direction_to_target)
        direction_to_target /= (distance_to_target + 1e-6)  # 归一化

        future_obstacle_pos = current_pos[:2] + obstacle_velocity * 0.5
        print(f"Predicted Obstacle Future Position: {future_obstacle_pos}")
        # 🔹 检查移动障碍物是否在无人机的路径上
        obstacle_ahead = (future_obstacle_pos[0] > current_pos[0]) and (np.linalg.norm(future_obstacle_pos - current_pos[:2]) < 3.0)

        moving_towards_drone = np.linalg.norm(obstacle_velocity) > 0.2  
        action = np.array([direction_to_target[0], direction_to_target[1], 0.0]) 
        current_y = state[1]
        obstacle_center = (left_edge + right_edge) / 2 
        
        # 增加冷却时间计数器
        if self.moving_avoid_counter > 0:
            self.moving_avoid_counter -= 1
        if self.static_avoid_counter > 0:
            self.static_avoid_counter -= 1

        # 记录障碍物历史位置
        if front_distance < 5.0:
            self.obstacle_history.append((current_pos[:2], front_distance, obstacle_velocity))
            if len(self.obstacle_history) > 10:
                self.obstacle_history.pop(0)

        # 记录距离历史，用于检测距离变化
        self.distance_history.append(front_distance)
        if len(self.distance_history) > 5:
            self.distance_history.pop(0)

        # 检测距离是否连续减少（可能正在接近障碍物）
        if len(self.distance_history) == 5:
            distance_decreasing = all(self.distance_history[i] > self.distance_history[i+1] for i in range(4))
            if distance_decreasing and front_distance < 3.0:
                print("⚠️ Warning: Distance to obstacle is decreasing! Taking evasive action.")
                # 选择空间更大的一侧进行避障
                if abs(right_free_space) > left_free_space:
                    action = np.array([1.0, -1.5, 0])
                    print("Evasive action: Moving right to avoid obstacle.")
                else:
                    action = np.array([1.0, 1.5, 0])
                    print("Evasive action: Moving left to avoid obstacle.")
                self.avoid_attempts = 0

        # 🛑 **移动障碍物避障**
        if obstacle_ahead and moving_towards_drone:
            print("⚠️ Moving obstacle detected in path! Adjusting route.")
            if self.moving_avoid_direction is None or self.moving_avoid_counter == 0:  # 只选择一次方向
                
                # 预测障碍物的移动轨迹
                obstacle_future_pos = current_pos[:2] + obstacle_velocity * 2.0
                # 计算避开障碍物的最佳方向
                if abs(obstacle_future_pos[1] - current_pos[1]) < 1.0:
                    # 障碍物在正前方，选择空间更大的一侧
                    if abs(right_free_space) > left_free_space:
                        self.moving_avoid_direction = "right"
                        print("Moving Right to Avoid Moving Obstacle.")
                    else:
                        self.moving_avoid_direction = "left"
                        print("Moving Left to Avoid Moving Obstacle.")
                else:
                    # 障碍物有横向移动，选择相反方向
                    if obstacle_velocity[1] > 0:
                        self.moving_avoid_direction = "right"
                        print("Moving Right to Avoid Moving Obstacle (obstacle moving left).")
                    else:
                        self.moving_avoid_direction = "left"
                        print("Moving Left to Avoid Moving Obstacle (obstacle moving right).")

                # 增加冷却时间，确保有足够时间绕过障碍物
                self.moving_avoid_counter = 15
                self.last_avoid_direction = self.moving_avoid_direction
                # 记录避障方向
                self.avoid_direction_history.append(self.moving_avoid_direction)
                if len(self.avoid_direction_history) > 10:
                    self.avoid_direction_history.pop(0)

            # ✅ 执行移动障碍物避障
            if self.moving_avoid_direction == "right":
                print("movv")
                action = np.array([1.5, -2.0, 0])  # 向右前方快速移动
            elif self.moving_avoid_direction == "left":
                print("mov")
                action = np.array([1.5, 2.0, 0])  # 向左前方快速移动
            elif self.moving_avoid_direction == "stop":
                action = np.array([-1.5, 0, 0])  # 减速

        elif front_distance < 1.5:  # 前方距离过近，需要紧急后退
            print("⚠️ CRITICAL: Very close to obstacle! Emergency backing up.")
            action = np.array([-2.0, 0, 0])  # 紧急快速后退
            self.static_avoid_direction = None
            self.moving_avoid_direction = None
            self.avoid_attempts = 0
            self.avoid_direction_history = []

        elif front_distance < 3.0:  # 前方距离较近，需要准备避障
            print("⚠️ Warning: Close to obstacle! Preparing avoidance.")
            # 选择空间更大的一侧
            if abs(right_free_space) > left_free_space:
                action = np.array([0.5, -1.0, 0])  # 向右前方移动
                print("Preparing to avoid obstacle by moving right.")
            else:
                action = np.array([0.5, 1.0, 0])  # 向左前方移动
                print("Preparing to avoid obstacle by moving left.")
            self.avoid_attempts += 1
            # 记录避障方向
            avoid_dir = "right" if abs(right_free_space) > left_free_space else "left"
            self.avoid_direction_history.append(avoid_dir)
            if len(self.avoid_direction_history) > 10:
                self.avoid_direction_history.pop(0)

        elif front_distance < 5.5:  
            if self.static_avoid_direction is None or self.static_avoid_counter == 0:  # 每个障碍物只选择一次
                # 选择更大的空间
                if abs(right_free_space) > left_free_space:
                    self.static_avoid_direction = "right"
                elif left_free_space > abs(right_free_space):
                    self.static_avoid_direction = "left"
                else:
                    self.static_avoid_direction = "stop"

                # 增加冷却时间，确保有足够时间绕过障碍物
                self.static_avoid_counter = 15
                self.last_avoid_direction = self.static_avoid_direction
                # 记录避障方向
                if self.static_avoid_direction != "stop":
                    self.avoid_direction_history.append(self.static_avoid_direction)
                    if len(self.avoid_direction_history) > 10:
                        self.avoid_direction_history.pop(0)

            # ✅ 执行选择的动作
            if self.static_avoid_direction == "right":
                action = np.array([1.5, -2.0, 0])  # 向右前方快速移动
                print("Avoiding obstacle! Moving Right.")
            elif self.static_avoid_direction == "left":
                action = np.array([1.5, 2.0, 0])  # 向左前方快速移动
                print("Avoiding obstacle! Moving Left.")
            elif self.static_avoid_direction == "stop":
                action = np.array([-1.5, 0, 0])  # 减速
                print("Obstacle ahead & no free space! Slowing down.")

            # 增加避障尝试次数
            self.avoid_attempts += 1
            # 如果多次尝试避障仍未成功，尝试切换方向
            if self.avoid_attempts > 20:
                print("⚠️ Stuck in obstacle avoidance, switching direction.")
                if self.static_avoid_direction == "left":
                    self.static_avoid_direction = "right"
                elif self.static_avoid_direction == "right":
                    self.static_avoid_direction = "left"
                self.avoid_attempts = 0
                self.static_avoid_counter = 15
                # 记录新的避障方向
                self.avoid_direction_history.append(self.static_avoid_direction)
                if len(self.avoid_direction_history) > 10:
                    self.avoid_direction_history.pop(0)

        elif left_free_space < 0.5 :
            print("Obstacle on the left! Moving straight.")
            self.static_avoid_direction = None
            self.moving_avoid_direction = None
            self.avoid_attempts = 0
            action = np.array([2.0, 0, 0])  # 快速向前移动

        elif right_free_space > -0.5:
            print("Obstacle on the right! Moving straight")
            self.static_avoid_direction = None
            self.moving_avoid_direction = None
            self.avoid_attempts = 0
            action = np.array([2.0, 0, 0])  # 快速向前移动
        else:
            # 检查是否需要回归目标方向
            if self.last_avoid_direction is not None:
                # 如果已经避开了障碍物，回归目标方向
                print("Regressing to target direction.")
                self.last_avoid_direction = None
                # 清除避障方向历史
                self.avoid_direction_history = []
            
            # 检查避障方向是否重复
            if len(self.avoid_direction_history) >= 5:
                if all(d == self.avoid_direction_history[0] for d in self.avoid_direction_history):
                    print("⚠️ Warning: Repeating same avoidance direction! Forcing direction change.")
                    # 强制切换方向
                    if self.avoid_direction_history[0] == "right":
                        action = np.array([direction_to_target[0] * 1.5, direction_to_target[1] - 1.0, 0.0])
                        print("Forcing left direction to break loop.")
                    else:
                        action = np.array([direction_to_target[0] * 1.5, direction_to_target[1] + 1.0, 0.0])
                        print("Forcing right direction to break loop.")
                    # 清除避障方向历史
                    self.avoid_direction_history = []
                else:
                    action = np.array([direction_to_target[0] * 2.0, direction_to_target[1] * 2.0, 0.0]) 
            else:
                action = np.array([direction_to_target[0] * 2.0, direction_to_target[1] * 2.0, 0.0]) 
            
            self.static_avoid_direction = None
            self.moving_avoid_direction = None
            self.avoid_attempts = 0
            print("target") 

        # 记录当前路径
        self.path_memory.append((current_pos, action))
        if len(self.path_memory) > 30:  # 增加路径记忆长度
            self.path_memory.pop(0)

        return action




    

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU()
        )
        self.mean = nn.Linear(256, action_dim)
        self.log_std = nn.Linear(256, action_dim)
        
        # 初始化网络权重，避免NaN值
        self._initialize_weights()
    
    def _initialize_weights(self):
        # 使用 Xavier 初始化
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
    
    def forward(self, state):
        x = self.fc(state)
        
        # 检查中间结果是否包含NaN值
        if torch.isnan(x).any():
            print("网络中间结果包含NaN值")
            # 返回默认值
            mean = torch.zeros_like(self.mean(x))
            log_std = torch.zeros_like(self.log_std(x))
        else:
            mean = self.mean(x)
            log_std = self.log_std(x).clamp(-20, 2)
        
        std = torch.exp(log_std)
        return mean, std
    
    def sample(self, state):
        mean, std = self.forward(state)
        
        # 检查均值和标准差是否包含NaN值
        if torch.isnan(mean).any() or torch.isnan(std).any():
            print("均值或标准差包含NaN值")
            # 使用默认动作
            action = torch.zeros_like(mean)
            log_prob = torch.zeros_like(mean)
        else:
            try:
                normal = torch.distributions.Normal(mean, std)
                z = normal.rsample()
                action = torch.tanh(z)
                log_prob = normal.log_prob(z) - torch.log(1 - action.pow(2) + 1e-6)
            except Exception as e:
                print(f"采样时出错: {e}")
                # 使用默认动作
                action = torch.zeros_like(mean)
                log_prob = torch.zeros_like(mean)
        
        return action, log_prob.sum(dim=-1, keepdim=True)

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim + action_dim, 128), nn.ReLU(),
            nn.Linear(128, 128), nn.ReLU(),
            nn.Linear(128, 1)
        )
        
        # 初始化网络权重，避免NaN值
        self._initialize_weights()
    
    def _initialize_weights(self):
        # 使用 Xavier 初始化
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
    
    def forward(self, state, action):
        # 检查输入是否包含NaN值
        if torch.isnan(state).any() or torch.isnan(action).any():
            print("评论家网络输入包含NaN值")
            # 返回默认值
            return torch.zeros(state.shape[0], 1, device=state.device)
        
        x = torch.cat([state, action], dim=-1)
        q_value = self.fc(x)
        
        # 检查输出是否包含NaN值
        if torch.isnan(q_value).any():
            print("评论家网络输出包含NaN值")
            # 返回默认值
            return torch.zeros_like(q_value)
        
        return q_value

class ReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
    
    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) >= self.capacity:
            self.buffer.pop(0)
        self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            torch.tensor(states, dtype=torch.float32),
            torch.tensor(actions, dtype=torch.float32),
            torch.tensor(rewards, dtype=torch.float32).unsqueeze(1),
            torch.tensor(next_states, dtype=torch.float32),
            torch.tensor(dones, dtype=torch.float32).unsqueeze(1)
        )
    
    def __len__(self):
        return len(self.buffer)

class SACAgent:
    def __init__(self, state_dim=6, action_dim=3, gamma=0.99, tau=0.005, alpha=0.2, buffer_size=10000, batch_size=64):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.actor = Actor(state_dim, action_dim).to(self.device)
        self.critic1 = Critic(state_dim, action_dim).to(self.device)
        self.critic2 = Critic(state_dim, action_dim).to(self.device)
        self.critic_target1 = Critic(state_dim, action_dim).to(self.device)
        self.critic_target2 = Critic(state_dim, action_dim).to(self.device)
        self.critic_target1.load_state_dict(self.critic1.state_dict())
        self.critic_target2.load_state_dict(self.critic2.state_dict())
        
        self.replay_buffer = ReplayBuffer(capacity=buffer_size)
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha
        self.batch_size = batch_size

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=0.001)
        self.critic_optimizer = optim.Adam(list(self.critic1.parameters()) + list(self.critic2.parameters()), lr=0.001)
        self.log_alpha = torch.tensor(np.log(self.alpha), requires_grad=True, device=self.device)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=0.001)
        self.target_entropy = -action_dim
        self.expert_action = ExpertPolicy()

    def select_action(self, state):
        # 检查状态是否包含NaN值
        if np.isnan(state).any():
            print("状态包含NaN值:", state)
            # 使用默认动作
            return np.array([1.0, 0.0, 0.0])
        
        expert_act = np.array(self.expert_action.expert_action(state))  # 专家动作
        state_tensor = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        
        # 检查状态张量是否包含NaN值
        if torch.isnan(state_tensor).any():
            print("状态张量包含NaN值")
            # 使用默认动作
            return np.array([1.0, 0.0, 0.0])
        
        try:
            sac_action, _ = self.actor.sample(state_tensor)
            sac_action = sac_action.detach().cpu().numpy().flatten() 
            # 混合专家和SAC动作
            action = 0.2* expert_act + 0.8* sac_action
            action[2] = 0.0  # 确保高度保持固定
            return np.clip(action, -1, 1)
        except Exception as e:
            print(f"选择动作时出错: {e}")
            # 使用默认动作
            return np.array([1.0, 0.0, 0.0])

    def train(self, batch_size):
        if len(self.replay_buffer) < batch_size:
            return  # 如果缓冲区未满则跳过

        # 采样批次
        states, actions, rewards, next_states, dones = self.replay_buffer.sample(batch_size)
        states = states.to(self.device)
        actions = actions.to(self.device)
        rewards = rewards.to(self.device)
        next_states = next_states.to(self.device)
        dones = dones.to(self.device)

        # ----- 评论家更新 -----
        with torch.no_grad():
            next_action, next_log_prob = self.actor.sample(next_states)  # 下一个动作
            next_q1 = self.critic_target1(next_states, next_action)
            next_q2 = self.critic_target2(next_states, next_action)
            next_q = torch.min(next_q1, next_q2) - self.alpha * next_log_prob
            target_q = rewards + (1 - dones) * self.gamma * next_q  # 计算目标Q值

        # 计算当前Q值
        q1 = self.critic1(states, actions)
        q2 = self.critic2(states, actions)
        
        # 评论家损失
        critic_loss = F.mse_loss(q1, target_q) + F.mse_loss(q2, target_q)
        
        # 反向传播评论家
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # ----- 演员更新 -----
        new_action, log_prob = self.actor.sample(states)  # 采样新动作
        q1_new = self.critic1(states, new_action)
        q2_new = self.critic2(states, new_action)
        q_new = torch.min(q1_new, q2_new)

        actor_loss = (self.alpha * log_prob - q_new).mean()

        # 反向传播演员
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # ----- 熵温度更新 -----
        alpha_loss = -(self.log_alpha * (log_prob + self.target_entropy).detach()).mean()

        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()
        
        # 更新alpha
        self.alpha = self.log_alpha.exp()

        # ----- 目标网络更新 -----
        for param, target_param in zip(self.critic1.parameters(), self.critic_target1.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        
        for param, target_param in zip(self.critic2.parameters(), self.critic_target2.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        return actor_loss.item(), critic_loss.item(), alpha_loss.item()

    def save(self, filename):
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic1_state_dict': self.critic1.state_dict(),
            'critic2_state_dict': self.critic2.state_dict(),
            'critic_target1_state_dict': self.critic_target1.state_dict(),
            'critic_target2_state_dict': self.critic_target2.state_dict(),
            'actor_optimizer_state_dict': self.actor_optimizer.state_dict(),
            'critic_optimizer_state_dict': self.critic_optimizer.state_dict(),
            'alpha_optimizer_state_dict': self.alpha_optimizer.state_dict(),
            'log_alpha': self.log_alpha.item(),
        }, filename)

    def load(self, filename):
        checkpoint = torch.load(filename, map_location=self.device)
        
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic1.load_state_dict(checkpoint['critic1_state_dict'])
        self.critic2.load_state_dict(checkpoint['critic2_state_dict'])
        self.critic_target1.load_state_dict(checkpoint['critic_target1_state_dict'])
        self.critic_target2.load_state_dict(checkpoint['critic_target2_state_dict'])
        
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer_state_dict'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer_state_dict'])
        self.alpha_optimizer.load_state_dict(checkpoint['alpha_optimizer_state_dict'])
        
        self.log_alpha = torch.tensor(checkpoint['log_alpha'], requires_grad=True, device=self.device)
        self.alpha = self.log_alpha.exp()
        
        print(f"Model loaded from {filename}")