#!/usr/bin/env python3
import time
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import UInt16, Empty, UInt8,Float32MultiArray  
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.srv import BusServoCtrl, GetArmCoords, GetArmIK, GetArmFK, GetArmFullState
from ros_robot_controller_msgs.msg import (
    ButtonState, BuzzerState, OLEDState, MotorState, MotorsState, MecanumState, TankState,
    ConveyorState, StepperRun, ArmCoords, ArmMoveInc, ArmServoSingle, EspnowState, ArmFullState,ServosPosition
)


class RosRobotController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.board = Board()
        self.board.enable_reception(True)
        self.running = True
        self.cb_group = ReentrantCallbackGroup()

        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.arm_full_state_pub = self.create_publisher(ArmFullState, '~/arm/full_state', 1)
        self.arm_joint_pub = self.create_publisher(Float32MultiArray, '~/arm/joint_states', 1)
        self.last_arm_state = None

        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_callback, 5)
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_callback, 5)
        self.create_subscription(MotorState, '~/chassis/set_single_motor', self.set_single_motor_callback, 5)
        self.create_subscription(MotorsState, '~/chassis/set_motors', self.set_motors_callback, 5)
        self.create_subscription(Empty, '~/chassis/stop_motors', self.stop_motors_callback, 5)
        # self.create_subscription(Twist, '~/chassis/set_mecanum', self.set_mecanum_callback, 5)
        self.create_subscription(Twist, '~/cmd_vel', self.set_mecanum_callback, 5)
        self.create_subscription(TankState, '~/chassis/set_tank', self.set_tank_callback, 5)
        self.create_subscription(UInt8, '~/conveyor/set', self.set_conveyor_callback, 5)
        self.create_subscription(Empty, '~/stepper/reset', self.stepper_reset_callback, 5)
        self.create_subscription(UInt8, '~/stepper/set_div', self.stepper_div_callback, 5)
        self.create_subscription(StepperRun, '~/stepper/run', self.stepper_run_callback, 5)
        self.create_subscription(ArmCoords, '~/arm/set_coords', self.set_arm_coords_callback, 5)
        self.create_subscription(ArmMoveInc, '~/arm/move_inc', self.arm_move_inc_callback, 5)
        self.create_subscription(ArmServoSingle, '~/arm/servo_single', self.arm_servo_single_callback, 5)
        self.create_subscription(EspnowState, '~/espnow/set', self.espnow_set_callback, 5)
        self.create_subscription(UInt8, '~/arm/set_move_acc', self.set_move_acc_callback, 5)
        self.create_service(GetArmIK, '~/arm/get_ik', self.get_arm_ik_callback, callback_group=self.cb_group)
        self.create_service(GetArmFK, '~/arm/get_fk', self.get_arm_fk_callback, callback_group=self.cb_group)
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)

        self.create_subscription(UInt8, '~/set_oled_icon', self.set_oled_icon_callback, 5)

        self.create_service(BusServoCtrl, '~/bus_servo/ctrl', self.bus_servo_ctrl_callback, callback_group=self.cb_group)
        self.create_service(GetArmCoords, '~/arm/get_coords', self.get_arm_coords_callback, callback_group=self.cb_group)
        self.create_service(GetArmFullState, '~/arm/get_full_state', self.get_arm_full_state_callback, callback_group=self.cb_group)

        threading.Thread(target=self.pub_callback, daemon=True).start()

        # 兼容旧代码: 提供 /controller_manager/init_finish 和 /kinematics/init_finish 服务
        self.create_service(Trigger, '/controller_manager/init_finish', self.init_finish_callback)
        self.create_service(Trigger, '/kinematics/init_finish', self.init_finish_callback)

    def init_finish_callback(self, request, response):
        response.success = True
        response.message = "init finish"
        return response

    @staticmethod
    def _jetarm_roll_to_driver_roll(angle_deg):
        return -float(angle_deg)

    @staticmethod
    def _driver_roll_to_jetarm_roll(angle_deg):
        return -float(angle_deg)

    def set_bus_servo_position(self, msg):
        for i in msg.position:
            self.board.bus_servo_set_position(i.id, position=i.position, acc=0, speed=0) 

    def _driver_joint_angles_to_public(self, joint_angles):
        public_angles = [float(v) for v in joint_angles]
        if len(public_angles) >= 5:
            public_angles[4] = self._driver_roll_to_jetarm_roll(public_angles[4])
        return public_angles

    def set_buzzer_callback(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def set_oled_callback(self, msg):
        self.board.set_oled_text(msg.index, msg.text)

    def set_single_motor_callback(self, msg):
        self.board.set_single_motor(msg.id, msg.speed)
        
    def get_arm_ik_callback(self, request, response):
        res = self.board.set_arm_coords(
            request.x, request.y, request.z, request.pitch,
            self._jetarm_roll_to_driver_roll(request.roll), request.claw, time_ms=0, calc_only=True
        )
        if res is not None:
            response.success = True
            response.servos = res['servos']
            self.get_logger().info(f"IK逆解计算成功, 舵机脉冲: {response.servos}")
        else:
            response.success = False
            self.get_logger().error("IK逆解计算失败或超时")
        return response

    def get_arm_fk_callback(self, request, response):
        res = self.board.get_fk_coords(
            request.j1, request.j2, request.j3, request.j4,
            self._jetarm_roll_to_driver_roll(request.roll), request.claw
        )
        if res is not None:
            response.success = True
            response.x = float(res['x'])
            response.y = float(res['y'])
            response.z = float(res['z'])
            response.pitch = float(res['pitch'])
            response.roll = self._driver_roll_to_jetarm_roll(res['roll'])
            response.claw = float(res['claw'])
            self.get_logger().info(f"FK正解计算成功, 坐标: X={res['x']}, Y={res['y']}, Z={res['z']}")
        else:
            response.success = False
            self.get_logger().error("FK正解计算失败或超时")
        return response
        
    def set_motors_callback(self, msg):
        self.board.set_motor_speed(msg.speed1, msg.speed2, msg.speed3, msg.speed4)

    def stop_motors_callback(self, msg):
        self.board.stop_all_motors()

    @staticmethod
    def _cmd_vel_to_chassis_speed(value):
        value = float(value)
        if -1.0 <= value <= 1.0:
            value *= 100.0
        return max(-100, min(100, int(round(value))))

    def set_mecanum_callback(self, msg):
        # self.board.set_mecanum(msg.vx, msg.vy, msg.vz)
        vx = self._cmd_vel_to_chassis_speed(msg.linear.x)
        vy = self._cmd_vel_to_chassis_speed(msg.linear.y)
        vz = self._cmd_vel_to_chassis_speed(msg.angular.z)
        self.board.set_mecanum(vx, vy, vz)

    def set_tank_callback(self, msg):
        self.board.set_tank(msg.speed, msg.turn)
    
    def set_oled_icon_callback(self, msg):
        self.board.set_oled_icon(msg.data)
        
    def set_conveyor_callback(self, msg):
        self.board.set_conveyor(msg.data)

    def stepper_reset_callback(self, msg):
        self.board.stepper_reset()

    def stepper_div_callback(self, msg):
        self.board.stepper_set_div(msg.data)

    def stepper_run_callback(self, msg):
        self.board.stepper_run(msg.steps)

    # def set_arm_coords_callback(self, msg):
    #     self.board.set_arm_coords(msg.x, msg.y, msg.z, msg.pitch, msg.roll, msg.time_ms)
    def set_arm_coords_callback(self, msg):
        self.board.set_arm_coords(
            msg.x,
            msg.y,
            msg.z,
            msg.pitch,
            self._jetarm_roll_to_driver_roll(msg.roll),
            msg.claw,
            msg.time_ms,
        )
    # def set_arm_coords_callback(self, msg):
    #     self.board.set_arm_coords(msg.x, msg.y, msg.z, msg.pitch, msg.roll, msg.time_ms)
    def arm_move_inc_callback(self, msg):
        self.board.arm_move_inc(
            msg.dx,
            msg.dy,
            msg.dz,
            msg.dpitch,
            self._jetarm_roll_to_driver_roll(msg.droll),
            msg.dclaw,
            msg.time_ms,
        )
        
    # def arm_move_inc_callback(self, msg):
    #     self.board.arm_move_inc(msg.dx, msg.dy, msg.dz, msg.dpitch, msg.droll, msg.time_ms)

    def arm_servo_single_callback(self, msg):
        self.board.arm_move_servo_single(msg.id, msg.pos, msg.time_ms)

    def espnow_set_callback(self, msg):
        self.board.espnow_set_channel(msg.channel)
        time.sleep(0.05)
        self.board.espnow_set_global_acc(msg.global_acc)
        time.sleep(0.05)
        self.board.espnow_sync_ctrl(msg.sync_enable)
        
    def set_move_acc_callback(self, msg):
        # msg.data 的范围是 0~254
        self.board.set_move_acc(msg.data)
        self.get_logger().info(f"已下发底层插补加速度: {msg.data}")
        
    def bus_servo_ctrl_callback(self, request, response):
        try:
            if request.set_torque:
                self.board.bus_servo_enable_torque(request.id, request.torque_enable)
                time.sleep(0.05)
            if request.set_mode:
                self.board.bus_servo_set_mode(request.id, request.mode)
                time.sleep(0.05)
            if request.set_position:
                self.board.bus_servo_set_position(request.id, request.position, request.acc, request.speed)

            response.current_position = self.board.bus_servo_read_position(request.id) or 0
            response.current_voltage = self.board.bus_servo_read_voltage(request.id) or 0
            response.current_temperature = self.board.bus_servo_read_temperature(request.id) or 0
            response.current_mode = self.board.bus_servo_read_mode(request.id) or 0
            response.success = True
        except Exception:
            response.success = False
        return response

    def get_arm_coords_callback(self, request, response):
        coords = self.board.get_arm_coords()
        if coords:
            response.success = True
            response.x = float(coords[0])
            response.y = float(coords[1])
            response.z = float(coords[2])
        else:
            response.success = False
        return response

    def _read_real_arm_state(self):
        tcp_pose = self.board.get_real_tcp_pose()
        if tcp_pose is None:
            return None

        joints_res = self.board.get_real_joint_angles()
        joint_angles = [0.0] * 6
        if joints_res and 'joints' in joints_res:
            for i, joint in enumerate(joints_res['joints'][:6]):
                joint_angles[i] = float(joint.get('angle', 0.0))

        servos = None
        full_state = self.board.get_full_state()
        if full_state is not None:
            servos = [int(v) for v in full_state.get('servos', [])[:6]]

        if servos is None or len(servos) < 6:
            if joints_res and 'joints' in joints_res:
                servos = [int(joint.get('pulse', 0)) for joint in joints_res['joints'][:6]]
            else:
                servos = [0] * 6

        if len(servos) < 6:
            servos = servos + [0] * (6 - len(servos))

        state = {
            'x': float(tcp_pose['x']),
            'y': float(tcp_pose['y']),
            'z': float(tcp_pose['z']),
            'yaw': float(tcp_pose['yaw']),
            'pitch': float(tcp_pose['pitch']),
            'roll': self._driver_roll_to_jetarm_roll(tcp_pose['roll']),
            'claw': float(tcp_pose['claw']),
            'servos': servos[:6],
            'joint_angles': self._driver_joint_angles_to_public(joint_angles[:6]),
        }
        self.last_arm_state = state
        return state

    def get_arm_full_state_callback(self, request, response):
        state = self._read_real_arm_state() or self.last_arm_state
        if state is None:
            response.success = False
            return response

        response.success = True
        response.x = float(state['x'])
        response.y = float(state['y'])
        response.z = float(state['z'])
        response.pitch = float(state['pitch'])
        response.roll = float(state['roll'])
        response.claw = float(state['claw'])
        response.yaw = float(state['yaw'])
        response.servos = [int(v) for v in state['servos']]
        response.joint_angles = [float(v) for v in state['joint_angles']]
        return response

    # def pub_callback(self):
    #     count = 0
    #     while self.running:
    #         if count % 20 == 0:
    #             battery_data = self.board.get_battery()
    #             if battery_data is not None:
    #                 self.battery_pub.publish(UInt16(data=battery_data))
            
    #         button_data = self.board.get_button()
    #         if button_data is not None:
    #             msg = ButtonState()
    #             msg.id = int(button_data[0])
    #             msg.state = int(button_data[1])
    #             self.button_pub.publish(msg)
            
    #         count += 1
    #         time.sleep(0.05)
            
    #     if rclpy.ok(): rclpy.shutdown()
    def pub_callback(self):
        count = 0
        self.get_logger().info("状态发布线程已启动")

        while self.running and rclpy.ok():
            try:
                state = self._read_real_arm_state() or self.last_arm_state

                if state:
                    full_msg = ArmFullState()
                    full_msg.x = float(state['x'])
                    full_msg.y = float(state['y'])
                    full_msg.z = float(state['z'])
                    full_msg.pitch = float(state['pitch'])
                    full_msg.roll = float(state['roll'])
                    full_msg.claw = float(state['claw'])
                    full_msg.yaw = float(state['yaw'])
                    full_msg.servos = [int(s) for s in state['servos']]
                    full_msg.joint_angles = [float(v) for v in state['joint_angles']]
                    self.arm_full_state_pub.publish(full_msg)

                    array_msg = Float32MultiArray()
                    array_msg.data = [
                        full_msg.x, full_msg.y, full_msg.z,
                        full_msg.yaw, full_msg.pitch, full_msg.roll, full_msg.claw,
                        *[float(v) for v in full_msg.joint_angles],
                        *[float(v) for v in full_msg.servos],
                    ]
                    self.arm_joint_pub.publish(array_msg)

                button_data = self.board.get_button()
                if button_data is not None:
                    btn_msg = ButtonState()
                    btn_msg.id = int(button_data[0])
                    btn_msg.state = int(button_data[1])
                    self.button_pub.publish(btn_msg)

                if count % 20 == 0:
                    battery_data = self.board.get_battery()
                    if battery_data is not None:
                        self.battery_pub.publish(UInt16(data=int(battery_data)))

                count += 1
                if count >= 1000:
                    count = 0

            except Exception as e:
                self.get_logger().error(f"pub_callback 发生异常: {str(e)}")

            time.sleep(0.1)

        self.get_logger().info("状态发布线程已停止")

def main():
    rclpy.init()
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
