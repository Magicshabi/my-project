#!/usr/bin/env python3
import time
import queue
import struct
import serial
import threading

class ESP32Cmd:
    FIRMWARE_VERSION_CHECK = 1
    CHECK_BAT_LEVEL = 2
    ACTION_GROUP_RUN = 3
    ACTION_GROUP_STOP = 4
    ACTION_GROUP_DOWNLOAD = 5
    FKINE_RESULT_GET = 6
    IKINE_RESULT_GET = 7
    COORDINATE_SET = 8
    BUZZER_SET = 9
    OLED_SET = 10
    GET_CUR_COORDS = 11
    OLED_ICON = 12 
    SET_SINGLE_MOTOR = 13
    STOP_ALL_MOTOR = 14
    SET_MOTOR_SPEED = 15
    CONVEYOR_SET = 16
    STEPPER_RESET = 17
    STEPPER_DIV = 18
    STEPPER_RUN = 19
    BUTTON_EVENT = 22
    ACTION_GROUP_ERASE = 23
    SET_ESPNOW_CHANNEL = 30
    SET_GLOBAL_ACC = 31
    ESPNOW_SYNC_CTRL = 33
    MECANUM_CONTROL = 34
    TANK_CONTROL = 35
    SET_PEER_MAC = 36
    ARM_MOVE_INC = 50
    ARM_SERVO_SINGLE = 51
    
    GET_CUR_COORDS = 11      # 返回 24 字节 (6轴坐标 + 6个舵机值)
    ARM_MOVE_INC = 50        # 6轴增量控制
    ARM_ALL_RESET = 54    
    SET_MOVE_ACC = 56
    GET_REAL_JOINT_ANGLES = 65  # 返回 24 字节: 6关节 × (pulse + angle_x10)
    GET_REAL_TCP_POSE = 66      # 返回 14 字节: X,Y,Z,Yaw,Pitch,Roll,Claw (角度×10)   

class ServoCmd:
    READ = 2
    WRITE = 3

class ServoReg:
    ID = 5
    MODE = 33
    TORQUE_ENABLE = 40
    ACC = 41
    GOAL_POSITION_L = 42
    PWM_SPEED_L = 44
    GOAL_SPEED_L = 46
    PRESENT_POSITION_L = 56
    PRESENT_VOLTAGE = 62
    PRESENT_TEMPERATURE = 63

def checksum_inv8(data):
    return (~sum(data)) & 0xFF

class PacketState:
    START1 = 0
    START2 = 1
    ID = 2
    LENGTH = 3
    CMD = 4
    DATA = 5
    CHECKSUM = 6

class Board:
    def __init__(self, device="/dev/rrc", baudrate=1000000, timeout=0.1):
        self.enable_recv = False
        self.frame =[]
        self.port = serial.Serial(None, baudrate, timeout=timeout)
        self.port.rts = False
        self.port.dtr = False
        self.port.setPort(device)
        self.port.open()
        time.sleep(3.0) 
        self.port.reset_input_buffer() 
        
        # self.state = PacketState.START1
        self.state = 0 
        self.data_len = 0
        self.recv_count = 0
        
        self.sys_queue = queue.Queue(maxsize=10)
        self.arm_queue = queue.Queue(maxsize=10)
        self.key_queue = queue.Queue(maxsize=10)
        self.bus_servo_queue = queue.Queue(maxsize=10)
        self.real_joint_queue = queue.Queue(maxsize=10)
        self.real_tcp_queue = queue.Queue(maxsize=10)
        self.servo_read_lock = threading.Lock()
        
        time.sleep(0.1)
        threading.Thread(target=self.recv_task, daemon=True).start()
        
    def set_oled_icon(self, icon_id):
        self.buf_write(0xFF, ESP32Cmd.OLED_ICON, struct.pack('<B', int(icon_id)))
        
    def buf_write(self, target_id, cmd, data=[]):
        length = 2 + len(data)
        buf = [0xFF, 0xFF, target_id, length, cmd] + list(data)
        check_data = buf[2:]
        buf.append(checksum_inv8(check_data))
        self.port.write(buf)

    def dispatch_packet(self, pid, cmd, data):
        if pid == 0xFF or pid == 0x5A:
            if cmd == ESP32Cmd.CHECK_BAT_LEVEL:
                try: self.sys_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd in (ESP32Cmd.GET_CUR_COORDS, ESP32Cmd.FKINE_RESULT_GET, ESP32Cmd.IKINE_RESULT_GET):
                try: self.arm_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_REAL_JOINT_ANGLES:
                try: self.real_joint_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.GET_REAL_TCP_POSE:
                try: self.real_tcp_queue.put_nowait(data)
                except queue.Full: pass
            elif cmd == ESP32Cmd.BUTTON_EVENT:
                try: self.key_queue.put_nowait(data)
                except queue.Full: pass
        else:
            try: self.bus_servo_queue.put_nowait([cmd] + list(data))
            except queue.Full: pass

    def enable_reception(self, enable=True):
        self.enable_recv = enable

    def recv_task(self):
        while True:
            if self.enable_recv and self.port.in_waiting > 0:
                try:
                    data_bytes = self.port.read(self.port.in_waiting)
                    for dat in data_bytes:
                        if self.state == PacketState.START1:
                            if dat == 0xFF: self.state = PacketState.START2
                        elif self.state == PacketState.START2:
                            if dat == 0xFF: self.state = PacketState.ID
                            else: self.state = PacketState.START1
                        elif self.state == PacketState.ID:
                            self.frame = [dat]
                            self.state = PacketState.LENGTH
                        elif self.state == PacketState.LENGTH:
                            self.frame.append(dat)
                            self.data_len = dat - 2
                            if self.data_len < 0 or self.data_len > 250:
                                self.state = PacketState.START1
                            else:
                                self.state = PacketState.CMD
                        elif self.state == PacketState.CMD:
                            self.frame.append(dat)
                            self.recv_count = 0
                            self.state = PacketState.DATA if self.data_len > 0 else PacketState.CHECKSUM
                        elif self.state == PacketState.DATA:
                            self.frame.append(dat)
                            self.recv_count += 1
                            if self.recv_count >= self.data_len:
                                self.state = PacketState.CHECKSUM
                        elif self.state == PacketState.CHECKSUM:
                            if checksum_inv8(self.frame) == dat:
                                self.dispatch_packet(self.frame[0], self.frame[2], self.frame[3:])
                            self.state = PacketState.START1
                except Exception:
                    pass
            else:
                time.sleep(0.005)
    
    def set_move_acc(self, acc):
        """
        【新增】：设置底层 10ms 插补算法的起步加速度 
        :param acc: 加速度值 (0~254，0为最快/无缓冲，数值越大缓冲越平滑)
        """
        acc_val = int(acc)
        if acc_val > 254: acc_val = 254
        if acc_val < 0: acc_val = 0
        
        self.buf_write(0xFF, ESP32Cmd.SET_MOVE_ACC, struct.pack('<B', acc_val))
        
    def get_battery(self):
        if not self.enable_recv: return None
        while not self.sys_queue.empty(): self.sys_queue.get()
        self.buf_write(0xFF, ESP32Cmd.CHECK_BAT_LEVEL)
        try:
            data = self.sys_queue.get(timeout=0.5)
            if len(data) >= 2:
                return struct.unpack('<H', bytes(data[:2]))[0]
        except queue.Empty: return None

    def get_button(self):
        if not self.enable_recv: return None
        try:
            data = self.key_queue.get_nowait()
            if len(data) >= 2:
                return data[0], data[1]
        except queue.Empty: return None

    def set_buzzer(self, freq, on_time_s, off_time_s, repeat=1):
        on_ms = int(on_time_s * 1000)
        off_ms = int(off_time_s * 1000)
        data = struct.pack("<IIHH", on_ms, off_ms, int(repeat), int(freq))
        self.buf_write(0xFF, ESP32Cmd.BUZZER_SET, data)

    def set_oled_text(self, line, text):
        text_bytes = text.encode('utf-8')[:20]
        data =[int(line), len(text_bytes)] + list(text_bytes)
        self.buf_write(0xFF, ESP32Cmd.OLED_SET, data)

    def get_arm_coords(self):
        if not self.enable_recv: return None
        while not self.arm_queue.empty(): self.arm_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_CUR_COORDS)
        time.sleep(0.1) 
        last_coords = None
        while not self.arm_queue.empty():
            data = self.arm_queue.get()
            if len(data) >= 6:
                last_coords = struct.unpack('<hhh', bytes(data[:6]))
        return last_coords

    # def set_arm_coords(self, x, y, z, pitch, roll=0, time_ms=1000):
    #     """设置5轴绝对坐标: X, Y, Z, Pitch(俯仰), Roll(横滚)"""
    #     pitch_val = int(pitch * 10)  # 协议要求放大10倍
    #     # 格式: pitch(h), x(h), y(h), z(h), roll(h), time(h) -> 共12字节
    #     data = struct.pack("<hhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(time_ms))
    #     self.buf_write(0xFF, ESP32Cmd.COORDINATE_SET, data)
        
    # def arm_move_inc(self, dx, dy, dz, dpitch, droll=0, time_ms=1000):
    #     """5轴坐标增量控制"""
    #     dpitch_val = int(dpitch * 10)
    #     # 格式: dx, dy, dz, dpitch, droll, time -> 共12字节
    #     data = struct.pack("<hhhhhh", int(dx), int(dy), int(dz), dpitch_val, int(droll), int(time_ms))
    #     self.buf_write(0xFF, ESP32Cmd.ARM_MOVE_INC, data)
    def set_arm_coords(self, x, y, z, pitch, roll=0, claw=0, time_ms=1000):
        pitch_val = int(pitch * 10)
        # 格式: pitch(h), x(h), y(h), z(h), roll(h), claw(h), time(h) -> 14字节
        data = struct.pack("<hhhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(claw), int(time_ms))
        self.buf_write(0xFF, ESP32Cmd.COORDINATE_SET, data)
        
    def set_arm_coords(self, x, y, z, pitch, roll=0, claw=0, time_ms=1000, calc_only=False):
        pitch_val = int(pitch * 10)
        
        if calc_only:
            if not self.enable_recv: return None
            while not self.arm_queue.empty(): self.arm_queue.get()
            
            data = struct.pack("<hhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(claw))
            self.buf_write(0xFF, ESP32Cmd.IKINE_RESULT_GET, data)
            
            try:
                res_data = self.arm_queue.get(timeout=0.2)
                if len(res_data) >= 24:
                    pose = struct.unpack('<hhhhhh', bytes(res_data[:12]))
                    servos = struct.unpack('<hhhhhh', bytes(res_data[12:24]))
                    return {
                        "x": pose[1], "y": pose[2], "z": pose[3], 
                        "pitch": pose[0]/10.0, "roll": pose[4], "claw": pose[5],
                        "servos": list(servos)
                    }
            except queue.Empty:
                return None
            return None
            
        else:
            # === 2. 正常运动 ===
            data = struct.pack("<hhhhhhh", pitch_val, int(x), int(y), int(z), int(roll), int(claw), int(time_ms))
            self.buf_write(0xFF, ESP32Cmd.COORDINATE_SET, data)
            return True
    
    def get_fk_coords(self, j1, j2, j3, j4, roll=0, claw=0):
        """正运动学计算：传入4个关节角度(度)及roll/claw，计算目标坐标，不运动"""
        if not self.enable_recv: return None
        # 清空队列防止读到旧数据
        while not self.arm_queue.empty(): self.arm_queue.get()
        
        # AT32 期望 12 字节: j1, j2, j3, j4, roll, claw (关节角为了精度放大10倍)
        p1, p2, p3, p4 = int(j1*10), int(j2*10), int(j3*10), int(j4*10)
        data = struct.pack("<hhhhhh", p1, p2, p3, p4, int(roll), int(claw))
        self.buf_write(0xFF, ESP32Cmd.FKINE_RESULT_GET, data)
        
        try:
            # 等待底层返回结果
            res_data = self.arm_queue.get(timeout=0.2)
            if len(res_data) >= 24:
                pose = struct.unpack('<hhhhhh', bytes(res_data[:12]))
                servos = struct.unpack('<hhhhhh', bytes(res_data[12:24]))
                return {
                    "x": pose[1], "y": pose[2], "z": pose[3], 
                    "pitch": pose[0]/10.0, "roll": pose[4], "claw": pose[5],
                    "servos": list(servos)
                }
        except queue.Empty:
            return None
        return None
    
    def arm_move_inc(self, dx, dy, dz, dpitch, droll=0, dclaw=0, time_ms=1000):
        dpitch_val = int(dpitch * 10)
        # 格式: dx, dy, dz, dpitch, droll, dclaw, time -> 14字节
        data = struct.pack("<hhhhhhh", int(dx), int(dy), int(dz), dpitch_val, int(droll), int(dclaw), int(time_ms))
        self.buf_write(0xFF, ESP32Cmd.ARM_MOVE_INC, data)
    def arm_all_reset(self, time_ms=2000):
        """机械臂所有关节回中/复位"""
        # 对应中位机指令 54
        data = struct.pack("<H", int(time_ms))
        self.buf_write(0xFF, 54, data)
    
    # def get_full_state(self):
    #     """
    #     通过运动学接口读取当前位姿及5个舵机的实时脉冲值
    #     返回: (dict) {坐标信息, 舵机列表}
    #     """
    #     if not self.enable_recv: return None
    #     # 清空队列旧数据
    #     while not self.arm_queue.empty(): self.arm_queue.get()
        
    #     # 发送读取指令 (CMD 11)
    #     self.buf_write(0xFF, ESP32Cmd.GET_CUR_COORDS)
        
    #     try:
    #         # 等待底层返回 20 字节数据
    #         data = self.arm_queue.get(timeout=0.5)
    #         if len(data) >= 20:
    #             # 解析前10字节：位姿
    #             pose = struct.unpack('<hhhhh', bytes(data[:10]))
    #             # 解析后10字节：5个舵机脉冲 (ID 1-5)
    #             servos = struct.unpack('<hhhhh', bytes(data[10:20]))
                
    #             return {
    #                 "x": pose[0], "y": pose[1], "z": pose[2], 
    #                 "pitch": pose[3]/10.0, "roll": pose[4],
    #                 "servos": list(servos) # [s1, s2, s3, s4, s5]
    #             }
    #     except queue.Empty:
    #         return None
    #     return None
    def get_full_state(self):
        if not self.enable_recv:
            return None
        while not self.arm_queue.empty():
            self.arm_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_CUR_COORDS)
        try:
            data = self.arm_queue.get(timeout=0.2)
            if len(data) >= 24:
                # 当前底层 GET_CUR_COORDS 返回顺序为:
                # X, Y, Z, Pitch(x10), Roll, Claw
                pose = struct.unpack('<hhhhhh', bytes(data[:12]))
                servos = struct.unpack('<hhhhhh', bytes(data[12:24]))

                return {
                    "x": pose[0],
                    "y": pose[1],
                    "z": pose[2],
                    "pitch": pose[3] / 10.0,
                    "roll": pose[4],
                    "claw": pose[5],
                    "servos": list(servos)
                }
        except queue.Empty:
            return None
        return None
    
    def get_arm_servos(self):
        """专门用于快速获取6个舵机当前脉冲位置的简易函数"""
        res = self.get_full_state()
        return res["servos"] if res else None

    def get_real_joint_angles(self):
        """
        查询6个关节的真实角度和脉冲值 (基于舵机实际读回位置, 非目标值)
        返回: dict { "joints": [ {"pulse": int, "angle": float}, ... ] }  (6个关节)
              或 None (超时)
        """
        if not self.enable_recv:
            return None
        while not self.real_joint_queue.empty():
            self.real_joint_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_REAL_JOINT_ANGLES)
        try:
            data = self.real_joint_queue.get(timeout=0.3)
            if len(data) >= 24:
                joints = []
                for i in range(6):
                    pulse = struct.unpack('<h', bytes(data[i*4:i*4+2]))[0]
                    angle_x10 = struct.unpack('<h', bytes(data[i*4+2:i*4+4]))[0]
                    joints.append({"pulse": pulse, "angle": angle_x10 / 10.0})
                return {"joints": joints}
        except queue.Empty:
            return None
        return None

    def get_real_tcp_pose(self):
        """
        查询真实TCP位姿 (基于舵机实际读回位置做FK, 含yaw)
        返回: dict { "x", "y", "z", "yaw", "pitch", "roll", "claw" }
              角度单位: 度, 坐标单位: mm
              或 None (超时)
        """
        if not self.enable_recv:
            return None
        while not self.real_tcp_queue.empty():
            self.real_tcp_queue.get()
        self.buf_write(0xFF, ESP32Cmd.GET_REAL_TCP_POSE)
        try:
            data = self.real_tcp_queue.get(timeout=0.3)
            if len(data) >= 14:
                vals = struct.unpack('<hhhhhhh', bytes(data[:14]))
                return {
                    "x": vals[0],
                    "y": vals[1],
                    "z": vals[2],
                    "yaw": vals[3] / 10.0,
                    "pitch": vals[4] / 10.0,
                    "roll": vals[5] / 10.0,
                    "claw": vals[6] / 10.0
                }
        except queue.Empty:
            return None
        return None
    
    def arm_move_servo_single(self, servo_id, pos, time_ms=1000):
        data = struct.pack("<BhH", int(servo_id), int(pos), int(time_ms))
        self.buf_write(0xFF, ESP32Cmd.ARM_SERVO_SINGLE, data)

    def bus_servo_set_position(self, servo_id, position, acc=0, speed=0):
        data = [ServoReg.ACC, acc] + list(struct.pack("<hHh", int(position), 0, int(speed)))
        self.buf_write(servo_id, ServoCmd.WRITE, data)

    def _bus_servo_read(self, servo_id, reg, length, unpack_format):
        with self.servo_read_lock:
            while not self.bus_servo_queue.empty(): self.bus_servo_queue.get()
            self.buf_write(servo_id, ServoCmd.READ,[reg, length])
            try:
                res = self.bus_servo_queue.get(block=True, timeout=0.2)
                if len(res) >= 1 + length:
                    values = struct.unpack(unpack_format, bytes(res[1:1+length]))
                    return values[0] if len(values) == 1 else values
            except queue.Empty: return None
        return None

    def bus_servo_read_position(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.PRESENT_POSITION_L, 2, '<h')

    def bus_servo_read_voltage(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.PRESENT_VOLTAGE, 1, '<B')

    def bus_servo_read_temperature(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.PRESENT_TEMPERATURE, 1, '<B')

    def bus_servo_read_mode(self, servo_id):
        return self._bus_servo_read(servo_id, ServoReg.MODE, 1, '<B')

    def set_single_motor(self, motor_idx, speed):
        self.buf_write(0xFF, ESP32Cmd.SET_SINGLE_MOTOR, struct.pack("<Bb", int(motor_idx), int(speed)))

    def set_motor_speed(self, s1, s2, s3, s4):
        self.buf_write(0xFF, ESP32Cmd.SET_MOTOR_SPEED, struct.pack("<bbbb", s1, s2, s3, s4))

    def stop_all_motors(self):
        self.buf_write(0xFF, ESP32Cmd.STOP_ALL_MOTOR)

    def set_mecanum(self, vx, vy, vz):
        self.buf_write(0xFF, ESP32Cmd.MECANUM_CONTROL, struct.pack("<bbb", vx, vy, vz))

    def set_tank(self, speed, turn):
        self.buf_write(0xFF, ESP32Cmd.TANK_CONTROL, struct.pack("<bb", speed, turn))

    def set_conveyor(self, speed):
        self.buf_write(0xFF, ESP32Cmd.CONVEYOR_SET, struct.pack("<b", speed))

    def stepper_reset(self):
        self.buf_write(0xFF, ESP32Cmd.STEPPER_RESET)

    def stepper_set_div(self, code):
        self.buf_write(0xFF, ESP32Cmd.STEPPER_DIV, struct.pack('<B', int(code)))

    def stepper_run(self, steps):
        self.buf_write(0xFF, ESP32Cmd.STEPPER_RUN, struct.pack("<i", int(steps)))

    def espnow_set_channel(self, channel):
        self.buf_write(0xFF, ESP32Cmd.SET_ESPNOW_CHANNEL, struct.pack('<B', channel))
        
    def espnow_set_global_acc(self, acc):
        self.buf_write(0xFF, ESP32Cmd.SET_GLOBAL_ACC, struct.pack('<B', acc))
        
    def espnow_sync_ctrl(self, enable):
        self.buf_write(0xFF, ESP32Cmd.ESPNOW_SYNC_CTRL, struct.pack('<B', 1 if enable else 0))


# ================== 综合功能测试套件 ==================
def run_robot_comprehensive_test(board, servo_id=1):
    print("\n================== 机械臂硬件与运动学综合测试启动 ==================")
    
    # --- 1. 读取基础信息 ---
    print("\n[1] 系统与状态读取测试")
    vol_mv = board.get_battery()
    if vol_mv is not None:
        print(f" -> 读取电压成功: {vol_mv/1000.0:.2f} V")
    else:
        print(" -> [警告] 电压读取超时，请检查固件通讯。")

    coords = board.get_arm_coords()
    if coords is not None:
        print(f" -> 初始全局坐标读取成功: X={coords[0]}, Y={coords[1]}, Z={coords[2]}")
    else:
        print(" -> [警告] 初始全局坐标读取超时。")

    # --- 2. 运动学坐标控制测试 ---
    print("\n[2] 运动学坐标控制 (逆解) 测试")
    print(" -> 绝对坐标指令: 移动到 (0, 150, 150), Pitch=0度，耗时 2000ms ...")
    board.set_arm_coords(x=150, y=0, z=150, pitch=0, time_ms=1000)
    time.sleep(2.5)

    print(" -> 增量坐标指令: Z轴上升 30mm, 俯仰角增加 5度，耗时 1500ms ...")
    board.arm_move_inc(dx=0, dy=0, dz=30, dpitch=5, time_ms=1500)
    time.sleep(2.0)
    
    # 获取移动后的新坐标
    coords_new = board.get_arm_coords()
    if coords_new:
        print(f" -> 移动后坐标为: X={coords_new[0]}, Y={coords_new[1]}, Z={coords_new[2]}")

    # --- 3.  ---
    print("\n[3] 舵机高级运动控制与角度读取")
    print(f" -> 控制舵机 {servo_id} 运动到位置 800 (加加速度=50, 速度=1200)...")
    board.bus_servo_set_position(servo_id, position=800, acc=50, speed=1200)
    time.sleep(2.0)
    
    print(f" -> 请求读取舵机 {servo_id} 的实时返回角度...")
    pos = board.bus_servo_read_position(servo_id)
    if pos is not None:
        print(f" -> [成功] 返回的舵机角度: {pos}")
    else:
        print(" -> [警告] 获取舵机角度超时。（注：目前的ESP32固件在C++层面可能未将舵机透传数据推送到USB，SDK端已完美适配并开放该函数）")
        
    # 读取温度和电压
    temp = board.bus_servo_read_temperature(servo_id)
    if temp is not None:
        print(f" -> [成功] 返回的舵机温度: {temp} °C")
        
    # 舵机回中
    board.bus_servo_set_position(servo_id, position=0, acc=100, speed=2500)
    time.sleep(1.0)

    # --- 4. 传送带测试 ---
    print("\n[4] 传送带测试")
    print(" -> 正转 (速度50)...")
    board.set_conveyor(50)
    time.sleep(1.5)
    print(" -> 停止传送带...")
    board.set_conveyor(0)
    time.sleep(0.5)

    # --- 5. 步进电机(滑杆)测试 ---
    print("\n[5] 步进电机(滑杆)测试")
    print(" -> 步进电机复位...")
    board.stepper_reset()
    time.sleep(0.5)
    print(" -> 运行正向 2000 步...")
    board.stepper_run(2000)
    time.sleep(2.0)

    # --- 6. 多路/单路电机控制测试 ---
    print("\n[6] 驱动电机控制测试")
    print(" -> 单路驱动 1 号电机 (速度 60)...")
    board.set_single_motor(1, 60)
    time.sleep(1.5)
    print(" -> 一键停止所有电机...")
    board.stop_all_motors()
    time.sleep(0.5)

    # --- 7. 底盘综合测试 (履带与麦轮) ---
    print("\n[7] 底盘协议层级测试")
    print(" -> 麦克纳姆轮底盘: 侧向平移 (vx=50, vy=0, vz=0)...")
    board.set_mecanum(50, 0, 0)
    time.sleep(1.5)
    print(" -> 停止麦轮...")
    board.set_mecanum(0, 0, 0)
    time.sleep(0.5)
    
    print(" -> 履带底盘: 原地转向 (转速 40)...")
    board.set_tank(0, 40)
    time.sleep(1.5)
    print(" -> 停止履带...")
    board.set_tank(0, 0)
    time.sleep(0.5)

    print("\n================== 测试完毕 ==================")


if __name__ == "__main__":
    board = Board(device="/dev/rrc") 
    board.enable_reception(True)
    
    print("程序已启动，正在等待串口初始化...")
    time.sleep(1)
    # board.set_move_acc(20)  #
    
    # 用一声蜂鸣声提示准备就绪
    board.set_buzzer(freq=2500, on_time_s=0.2, off_time_s=0.0, repeat=1)
    board.set_arm_coords(x=200, y=0, z=200, pitch=0, time_ms=500)
    time.sleep(0.5)
    board.set_arm_coords(x=250, y=0, z=200, pitch=0, time_ms=500)
    time.sleep(0.5)
    # 运行全面的测试流程
    # run_robot_comprehensive_test(board, servo_id=1)

    print("\n进入挂起等待状态，可用 Ctrl+C 安全退出程序...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n程序退出。")
