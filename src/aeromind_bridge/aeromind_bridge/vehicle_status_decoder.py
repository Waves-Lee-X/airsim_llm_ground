#!/usr/bin/env python3
"""
vehicle_status_decoder.py — PX4 VehicleStatus 消息中文解析器

来源：整合自 hw_insight 的 msg_px4_fmu_out_vehicle_status.py
用途：将 VehicleStatus 的数值字段解码为可读的中文描述，
      用于调试、日志输出、DroneState 消息填充。

用法：
    from aeromind_bridge.vehicle_status_decoder import VehicleStatusDecoder
    decoder = VehicleStatusDecoder()
    print(decoder.decode_nav_state(14))  # "NAVIGATION_STATE_OFFBOARD:(外部控制模式)"
"""


class VehicleStatusDecoder:
    """PX4 VehicleStatus 所有枚举字段的中文解码器"""

    # ---- 解锁状态 ----
    ARMING_STATE_MAP = {
        1: "DISARMED(未解锁,不可起飞)",
        2: "ARMED(已解锁,可以起飞)",
    }

    # ---- 解锁/加锁原因 ----
    ARM_DISARM_REASON_MAP = {
        0:  "TRANSITION_TO_STANDBY(切换为待命状态)",
        1:  "RC_STICK(遥控器摇杆操作)",
        2:  "RC_SWITCH(遥控器开关操作)",
        3:  "COMMAND_INTERNAL(飞控内部命令)",
        4:  "COMMAND_EXTERNAL(外部命令,如地面站/ROS 2)",
        5:  "MISSION_START(任务开始)",
        6:  "SAFETY_BUTTON(机身物理安全按钮)",
        7:  "AUTO_DISARM_LAND(降落后自动锁定)",
        8:  "AUTO_DISARM_PREFLIGHT(起飞前自动锁定)",
        9:  "KILL_SWITCH(急停开关)",
        10: "LOCKDOWN(锁定逻辑触发)",
        11: "FAILURE_DETECTOR(故障检测)",
        12: "SHUTDOWN(关机操作)",
        13: "UNIT_TEST(单元测试)",
    }

    # ---- 导航/飞行模式 ----
    NAV_STATE_MAP = {
        0:  "MANUAL(手动模式: 摇杆全控制,仅基础姿态稳定)",
        1:  "ALTCTL(高度控制: 摇杆水平,飞控维持高度)",
        2:  "POSCTL(位置控制: 自动保持位置和高度)",
        3:  "AUTO_MISSION(自动任务: 按飞行计划执行)",
        4:  "AUTO_LOITER(自动悬停: 在目标点盘旋)",
        5:  "AUTO_RTL(自动返航,如失联/低电量)",
        6:  "POSITION_SLOW(慢速位置控制)",
        10: "ACRO(特技模式: 无姿态限制)",
        12: "DESCEND(垂直下降: 仅控制高度)",
        13: "TERMINATION(终止模式: 关闭所有电机)",
        14: "OFFBOARD(外部控制: 由 ROS 2/地面站控制)",
        15: "STAB(自稳模式: 手动姿态,飞控补偿风力)",
        17: "AUTO_TAKEOFF(自动起飞)",
        18: "AUTO_LAND(自动降落)",
        19: "AUTO_FOLLOW_TARGET(自动跟随目标)",
        20: "AUTO_PRECLAND(精确定点着陆)",
        21: "ORBIT(环绕模式)",
        22: "AUTO_VTOL_TAKEOFF(VTOL自动起飞)",
    }

    # ---- 故障检测位掩码 ----
    FAILURE_MASK_MAP = {
        1:   "ROLL(横滚控制故障)",
        2:   "PITCH(俯仰控制故障)",
        4:   "ALT(高度控制故障)",
        8:   "EXT(外部故障)",
        16:  "ARM_ESC(电调解锁失败)",
        32:  "BATTERY(电池故障)",
        64:  "IMBALANCED_PROP(螺旋桨不平衡)",
        128: "MOTOR(电机故障)",
    }

    # ---- 飞行器类型 ----
    VEHICLE_TYPE_MAP = {
        0: "UNKNOWN(未知)",
        1: "ROTARY_WING(旋翼/多旋翼)",
        2: "FIXED_WING(固定翼)",
        3: "ROVER(地面车)",
        4: "AIRSHIP(飞艇)",
    }

    # ---- 故障保护延迟状态 ----
    FAILSAFE_DEFER_MAP = {
        0: "DISABLED(严格遵循保护规则,立即触发)",
        1: "ENABLED(延迟触发,给予处理时间)",
        2: "WOULD_FAILSAFE(已延迟,记录故障但暂不触发)",
    }

    # ---- HIL 状态 ----
    HIL_STATE_MAP = {
        0: "OFF(硬件在环关闭)",
        1: "ON(硬件在环开启)",
    }

    # ==================== 解码方法 ====================

    def decode_arming_state(self, value: int) -> str:
        return self.ARMING_STATE_MAP.get(value, f"未知({value})")

    def decode_arm_reason(self, value: int) -> str:
        return self.ARM_DISARM_REASON_MAP.get(value, f"未知({value})")

    def decode_nav_state(self, value: int) -> str:
        return self.NAV_STATE_MAP.get(value, f"未知({value})")

    def decode_nav_state_short(self, value: int) -> str:
        """返回简短的英文模式名（用于 DroneState.mode 字段）"""
        full = self.NAV_STATE_MAP.get(value, f"UNKNOWN_{value}")
        return full.split("(")[0].strip()

    def decode_failures(self, value: int) -> list:
        """解析故障位掩码,返回故障名列表"""
        if value == 0:
            return []
        failures = []
        for mask, name in self.FAILURE_MASK_MAP.items():
            if value & mask:
                failures.append(name)
        return failures

    def decode_vehicle_type(self, value: int) -> str:
        return self.VEHICLE_TYPE_MAP.get(value, f"未知({value})")

    def decode_failsafe_defer(self, value: int) -> str:
        return self.FAILSAFE_DEFER_MAP.get(value, f"未知({value})")

    def decode_hil_state(self, value: int) -> str:
        return self.HIL_STATE_MAP.get(value, f"未知({value})")

    def is_armed(self, status_msg) -> bool:
        """消息中 arming_state==2 表示已解锁"""
        return status_msg.arming_state == 2

    def is_offboard(self, status_msg) -> bool:
        """消息中 nav_state==14 表示 Offboard 模式"""
        return status_msg.nav_state == 14

    def has_failure(self, status_msg) -> bool:
        """检查是否有任何故障"""
        return status_msg.failure_detector_status != 0

    def summary(self, status_msg) -> str:
        """单行摘要: 解锁状态 + 飞行模式 + 故障"""
        arm = self.decode_arming_state(status_msg.arming_state)
        nav = self.decode_nav_state(status_msg.nav_state)
        fails = self.decode_failures(status_msg.failure_detector_status)
        fail_str = " | ".join(fails) if fails else "无故障"
        return f"[{arm}] [{nav}] 故障: {fail_str}"

    def detailed(self, status_msg) -> str:
        """多行详细状态报告"""
        lines = [
            "=" * 60,
            f"时间戳:          {status_msg.timestamp / 1e6:.2f}s",
            f"解锁状态:        {self.decode_arming_state(status_msg.arming_state)}",
            f"飞行模式:        {self.decode_nav_state(status_msg.nav_state)}",
            f"最近解锁原因:    {self.decode_arm_reason(status_msg.latest_arming_reason)}",
            f"最近加锁原因:    {self.decode_arm_reason(status_msg.latest_disarming_reason)}",
            f"故障:            {', '.join(self.decode_failures(status_msg.failure_detector_status)) or '无'}",
            f"故障保护状态:    {status_msg.failsafe}",
            f"保护延迟:        {self.decode_failsafe_defer(status_msg.failsafe_defer_state)}",
            f"预检通过:        {status_msg.pre_flight_checks_pass}",
            f"飞行器类型:      {self.decode_vehicle_type(status_msg.vehicle_type)}",
            f"GCS 连接:        {'断开' if status_msg.gcs_connection_lost else '正常'}",
            f"电源有效:        {status_msg.power_input_valid}",
            f"安全开关解除:    {status_msg.safety_off}",
            f"VTOL:           {status_msg.is_vtol}",
            f"HIL:            {self.decode_hil_state(status_msg.hil_state)}",
            "=" * 60,
        ]
        return "\n".join(lines)
