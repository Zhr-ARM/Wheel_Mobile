import serial
import time
import struct
import numpy as np
from numpy.linalg import inv # 需要求逆运算

class JY61P:
    def __init__(self, port, baudrate=9600):
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self.last_angle_time = time.time()
        self.acc = [0.0, 0.0, 0.0]   # [ax, ay, az]
        self.gyro = [0.0, 0.0, 0.0]  # [wx, wy, wz]
        self.angle = [0.0, 0.0, 0.0] # [roll, pitch, yaw]
        self.temp = 0.0
        self.dt = 0.0 
        
        self.data_ready = False
        
        self.buffer = b""

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    def get_acc(self):
        return self.acc

    def get_gyro(self):
        return self.gyro

    def get_angle(self):
        return self.angle

    def get_dt(self):
        return self.dt

    def zero_z_axis(self):
        print("开始 Z 轴置零...")
        unlock_cmd = b'\xFF\xAA\x69\x88\xB5'
        self.ser.write(unlock_cmd)
        time.sleep(0.2)
        zero_cmd = b'\xFF\xAA\x01\x04\x00'
        self.ser.write(zero_cmd)
        print("正在校准，请保持静止 3 秒...")
        time.sleep(3.0)
        save_cmd = b'\xFF\xAA\x00\x00\x00'
        self.ser.write(save_cmd)
        print("Z 轴置零完成并保存。")

    def update(self):
        try:
            if self.ser.in_waiting:
                self.buffer += self.ser.read(self.ser.in_waiting)
        except Exception as e:
            print(f"串口读取错误: {e}")
            return

        while len(self.buffer) >= 11:
            idx = self.buffer.find(b'\x55')
            if idx == -1:
                self.buffer = b""
                break
            
            if idx > 0:
                self.buffer = self.buffer[idx:]
            
            if len(self.buffer) < 11:
                break

            packet_type = self.buffer[1]
            if packet_type not in [0x51, 0x52, 0x53]:
                self.buffer = self.buffer[1:]
                continue
                
            data_packet = self.buffer[:11]
            checksum = sum(data_packet[:10]) & 0xFF
            if checksum != data_packet[10]:
                print(f"校验和错误: 计算值 {checksum:02X} != 接收值 {data_packet[10]:02X}")
                self.buffer = self.buffer[1:]
                continue
            
            if packet_type == 0x51: 
                raw = struct.unpack('<hhhh', data_packet[2:10])
                ax = raw[0] / 32768.0 * 16.0 * 9.8 
                ay = raw[1] / 32768.0 * 16.0 * 9.8
                az = raw[2] / 32768.0 * 16.0 * 9.8
                temp = raw[3] / 100.0
                
                self.acc = [ax, ay, az]
                self.temp = temp
                
            elif packet_type == 0x52: 
                raw = struct.unpack('<hhhh', data_packet[2:10])
                wx = raw[0] / 32768.0 * 2000.0
                wy = raw[1] / 32768.0 * 2000.0
                wz = raw[2] / 32768.0 * 2000.0
                
                self.gyro = [wx, wy, wz]

            elif packet_type == 0x53: 
                raw = struct.unpack('<hhhh', data_packet[2:10])
                roll = raw[0] / 32768.0 * 180.0
                pitch = raw[1] / 32768.0 * 180.0
                yaw = raw[2] / 32768.0 * 180.0
                
                current_time = time.time()
                self.dt = current_time - self.last_angle_time
                self.last_angle_time = current_time
                
                self.angle = [roll, pitch, yaw]
                self.data_ready = True
            
            self.buffer = self.buffer[11:]

def quaternion_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def to_euler(q):
    q0, q1, q2, q3 = q
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch_arg = 2 * (q0 * q2 - q1 * q3)
    pitch_arg = np.clip(pitch_arg, -1.0, 1.0)
    pitch = np.arcsin(pitch_arg)

    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)

class BaseAHRS:
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
    
    def update(self, acc_g, gyro_dps, dt):
        raise NotImplementedError("Must be implemented in subclass")
    
    def get_euler(self):
        return to_euler(self.q)
    
class MahonyAHRS(BaseAHRS):
    def __init__(self, Kp=10.0, Ki=0.01):
        super().__init__()
        self.Kp = Kp
        self.Ki = Ki
        self.e_int = np.array([0.0, 0.0, 0.0])

    def change_gains(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki

    def set_initial_bias(self, initial_bias_dps):
        self.e_int = np.radians(np.array(initial_bias_dps))
        print(f"Mahony 积分误差 (Bias) 初始化为: {self.e_int} rad/s")

    def update(self, acc_g, gyro_dps, dt):
        gyro = np.radians(gyro_dps)
        acc = np.array(acc_g)
        norm_acc = np.linalg.norm(acc)
        if norm_acc == 0:
            return self.get_euler()
        a_norm = acc / norm_acc

        q0, q1, q2, q3 = self.q
        vx = 2 * (q1 * q3 - q0 * q2)
        vy = 2 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        v = np.array([vx, vy, vz])

        e = np.cross(a_norm, v)

        if self.Ki > 0:
            self.e_int += e * self.Ki * dt
        
        gyro_corr = gyro + self.Kp * e + self.e_int

        omega_quat = np.array([0.0, *gyro_corr])
        q_dot = 0.5 * quaternion_mult(self.q, omega_quat)
        self.q += q_dot * dt
        self.q /= np.linalg.norm(self.q)
        return self.get_euler()

def collect_initial_bias(imu_device, seconds=5, rate=200):
    print(f"请保持 IMU 静止 {seconds} 秒，开始采集初始 Bias...")
    
    time.sleep(0.5) 
    imu_device.buffer = b"" 
    
    bias_data = []
    
    end_time = time.time() + seconds
    
    while time.time() < end_time or len(bias_data) < 100:
        imu_device.update()
        if imu_device.data_ready:
            bias_data.append(imu_device.get_gyro()) 
            imu_device.data_ready = False
        time.sleep(0.001)

    if not bias_data:
        print("警告: 未采集到有效数据，Bias 初始化失败。")
        return [0.0, 0.0, 0.0]

    avg_bias = np.mean(bias_data, axis=0)
    print(f"采集到 {len(bias_data)} 个点。")
    print(f"初始陀螺仪 Bias (deg/s): X={avg_bias[0]:.4f}, Y={avg_bias[1]:.4f}, Z={avg_bias[2]:.4f}")
    return avg_bias

class MadgwickAHRS(BaseAHRS):
    def __init__(self, beta=0.4):
        super().__init__()
        self.beta = beta 
        
    def update(self, acc_g, gyro_dps, dt):
        gyro = np.radians(gyro_dps)
        acc = np.array(acc_g)
        omega_quat = np.array([0.0, *gyro])
        q_dot_gyro = 0.5 * quaternion_mult(self.q, omega_quat)

        norm_acc = np.linalg.norm(acc)
        if norm_acc == 0: 
             return self.get_euler()
        a_norm = acc / norm_acc

        q0, q1, q2, q3 = self.q

        vx = 2 * (q1 * q3 - q0 * q2)
        vy = 2 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        m = np.array([
            -2 * (q2 * a_norm[1] - q1 * a_norm[2]),
            2 * (q3 * a_norm[1] - q0 * a_norm[2] + q1 * a_norm[0]),
            2 * (q0 * a_norm[1] + q3 * a_norm[2] - q2 * a_norm[0]),
            2 * (q1 * a_norm[1] + q2 * a_norm[2])
        ])
        norm_m = np.linalg.norm(m)
        if norm_m > 0:
            m /= norm_m

        self.q += (q_dot_gyro - self.beta * m) * dt

        self.q /= np.linalg.norm(self.q)
        
        return self.get_euler()
    
def EKF_Omega(w):
    wx, wy, wz = w
    return np.array([
        [0, -wx, -wy, -wz],
        [wx, 0, wz, -wy],
        [wy, -wz, 0, wx],
        [wz, wy, -wx, 0]
    ])

def EKF_F_jacobian(q, gyro_corr, dt):
    F = np.eye(7)
    q_mat = np.eye(4) + 0.5 * EKF_Omega(gyro_corr) * dt
    F[:4, :4] = q_mat
    
    q0, q1, q2, q3 = q
    Jq = np.array([
        [q1, q2, q3],
        [-q0, q3, -q2],
        [-q3, -q0, q1],
        [q2, -q1, -q0]
    ])
    F[:4, 4:] = -0.5 * Jq * dt
    return F

def EKF_H_jacobian(q):
    H = np.zeros((3, 7))
    q0, q1, q2, q3 = q
    
    H[0, 0] = -2*q2
    H[0, 1] = 2*q3
    H[0, 2] = -2*q0
    H[0, 3] = 2*q1
    
    H[1, 0] = 2*q1
    H[1, 1] = 2*q0
    H[1, 2] = 2*q3
    H[1, 3] = 2*q2
    
    H[2, 0] = 2*q0
    H[2, 1] = -2*q1
    H[2, 2] = -2*q2
    H[2, 3] = 2*q3
    
    return H

class EKFAHRS(BaseAHRS):
    def __init__(self, R_base=None, Q_q=1e-5, Q_b=1e-6, P_init=0.1, R_mult_factor=100.0):
        super().__init__()
        
        # 状态向量 x = [q0, q1, q2, q3, bx, by, bz] (7个状态)
        self.x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(7) * P_init 
        
        # 过程噪声 Q
        Q_diag = np.concatenate(([Q_q]*4, [Q_b]*3))
        self.Q = np.diag(Q_diag) 
        
        # 基础测量噪声 R_base
        if R_base is None:
             R_base = [0.1, 0.1, 0.1] 
        self.R_base = np.diag(R_base)
        
        # 自适应 R 调整参数
        self.R_mult_factor = R_mult_factor 

    def _h_observation(self, q):
        q0, q1, q2, q3 = q
        h = np.array([
             2 * (q1 * q3 - q0 * q2),
             2 * (q0 * q1 + q2 * q3),
             q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        ])
        return h

    def _adjust_R(self, acc_g):
        acc_g_norm = np.linalg.norm(acc_g) / 9.8 
        acc_err = abs(acc_g_norm - 1.0)
        
        R_mult = 1.0 + self.R_mult_factor * acc_err**2 
        
        R_adjusted = self.R_base * R_mult
        return R_adjusted

    def update(self, acc_g, gyro_dps, dt):
        
        # 0. 数据准备
        gyro = np.radians(gyro_dps)
        acc = np.array(acc_g) / 9.8 # 转换为 g
        
        # ------------------- 预测阶段 (Time Update) -------------------
        q_prev = self.x[:4]
        b_prev = self.x[4:]
        
        # 1. 修正角速度：使用上一时刻的 Bias 估计
        gyro_corr = gyro - b_prev
        
        # 2. 状态预测 (x_k^-) - 姿态预测
        omega_quat = np.array([0.0, *gyro_corr])
        q_dot = 0.5 * quaternion_mult(q_prev, omega_quat)
        q_pred = q_prev + q_dot * dt
        q_pred /= np.linalg.norm(q_pred)
        
        # Bias 预测不变
        b_pred = b_prev
        x_pred = np.concatenate((q_pred, b_pred))
        
        # 3. 协方差矩阵预测 (P_k^-)
        F = EKF_F_jacobian(q_prev, gyro_corr, dt) 
        self.P = F @ self.P @ F.T + self.Q * dt**2

        # ------------------- 更新阶段 (Measurement Update) -------------------
        
        # 4. 自适应 R 调整
        R_k = self._adjust_R(acc_g)
        
        # 5. 观测雅可比矩阵 H (3x7)
        H = EKF_H_jacobian(q_pred)

        # 6. 计算卡尔曼增益 K_k
        P_HT = self.P @ H.T
        S = H @ P_HT + R_k
        try:
            K = P_HT @ inv(S) # 需要求逆
        except np.linalg.LinAlgError:
            # 矩阵奇异，跳过更新步骤
            # print("警告: 协方差矩阵奇异，跳过 EKF 更新。")
            self.x = x_pred
            return to_euler(self.x[:4])

        # 7. 计算残差 y_k
        z_k = acc
        h_k = self._h_observation(q_pred)
        y = z_k - h_k
        
        # 8. 更新状态和协方差矩阵
        self.x = x_pred + K @ y
        self.P = (np.eye(7) - K @ H) @ self.P
        
        # 9. 四元数重新归一化 
        self.x[:4] /= np.linalg.norm(self.x[:4])
        
        return to_euler(self.x[:4])


if __name__ == "__main__":
    try:
        imu = JY61P('COM11', 115200)
        # imu.zero_z_axis()

        ahrs_esti = MahonyAHRS(Kp=15.0, Ki=0.0)
        initial_bias_dps = collect_initial_bias(imu, seconds=10)
        ahrs_esti.set_initial_bias(initial_bias_dps)

        # ahrs_esti = MadgwickAHRS(beta=0.1)

        # ahrs_esti = EKFAHRS(Q_q=1e-5, Q_b=1e-7, P_init=0.5, R_mult_factor=200.0)

        print("开始读取 IMU 数据... (按 Ctrl+C 停止)")
        count = 0
        while True:
            imu.update() 
            
            if imu.data_ready:
                ax, ay, az = imu.get_acc()
                gx, gy, gz = imu.get_gyro()
                roll, pitch, yaw = imu.get_angle()
                dt = imu.get_dt()
                
                esti_yaw, esti_pitch, esti_roll = ahrs_esti.update([ax, ay, az], [gx, gy, gz], dt)
                 
                count += 1
                if count % 10 == 0:
                    # print(f"DT: {dt*1000:.2f}ms")
                    # print(f"加速度: ax={ax:.2f}m/s², ay={ay:.2f}m/s², az={az:.2f}m/s²")
                    # print(f"角速度: gx={gx:.2f}°/s, gy={gy:.2f}°/s, gz={gz:.2f}°/s")
                    # print(f"角度: roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
                    print(f"估算: roll={esti_roll:.2f}°, pitch={esti_pitch:.2f}°, yaw={esti_yaw:.2f}°")
                    count = 0
                    
                imu.data_ready = False
            
            time.sleep(0.001) # 防止 CPU 占用过高
            
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
    except KeyboardInterrupt:
        print("\n停止读取")
        if 'imu' in locals():
            imu.close()
