import math
import numpy
import control
import matplotlib.pyplot as plt

# 출력축(조인트) 기준 BLDC 액추에이터 모델
# 외부 부하는 출력축 기준으로 입력, 내부에서 모터축으로 반사
class BLDCActuator():
    
    def __init__(self):
        self.params()
        self.reset()
    
    def reset(self):
        self.i_u = 0.0
        self.i_v = 0.0
        self.i_w = 0.0
        self.w_output = 0.0
        self.theta_e = 0.0
    
    def params(self):
        
        self.R = 0.35          # 모터 상저항 Ω
        self.L = 0.00017       # 상인덕턴스 0.17mH
        self.Ke = 6 / 1000 * (60 / (2 * math.pi))  # 역기전력 상수 6 Vdc/Krpm → V/(rad/s)
        self.Kt = 1.9          # 토크 상수 Nm/A 
        self.J = 0.3 * 1e-4    # 관성 모멘트 (로터)
        self.b = 0             # 점성 마찰 계수 일단 0
        self.M = 0             # 상호 인덕턴스 0
        self.tau_load = 5      # 외부 부하 Nm (출력축 기준)
        self.dt = 0.0001       # MCU로부터의 제어 10kHz
        self.Pole = 26         # 극수
        self.P = 13            # 극쌍수
        self.G = 36            # 감속비 (기어비)
        self.J_load = 0.0      # 외부 링크 관성 kg·m² (출력축 기준)
        self.J_eq = self.J * (self.G ** 2) + self.J_load  # 출력축 기준 등가 관성
        self.w_max = 111 * self.G
        self.I_max = 21.5      # 최대 A
        self.V_max = 24        # 입력 V = 최대 V
    
    # V_desired를 받아서 V_desired = R * I + L(d_I_actual/dt) + e 
    # 근데 실제로는 e를 따로 더해준다는데 같이 더해줘야하나? 아니다 차라리 e를 계산하는 함수를 만들자
    
    # 역기전력 계산 함수
    def calc_Back_BMF(self, Ke, w_motor):
        
        E = Ke * w_motor
        
        return E
    
    # 듀얼 엔코더, 해상도로 각도를 리턴
    def dual_encoder(self, theta_elec, theta_mech):
        
        # 전기각 theta_elec - 엔코더 2^17
        res_elec = 2*math.pi / 131072
        theta_elec = round(theta_elec / res_elec) * res_elec
        
        # 기계각? theta_mech - 엔코더 2^18
        res_mech = 2*math.pi / 262144
        theta_mech = round(theta_mech / res_mech) * res_mech
        
        return theta_elec, theta_mech
    
    # 전기각 theta_elec의 변화에 따라서 홀센서의 1, 0 신호들
    def whole_sensor_switching_process(self, direction):
        
        # 시계방향 
        if direction == "forward":
            step1 = [0, 0, 1]
            step2 = [0, 1, 1]
            step3 = [0, 1, 0]
            step4 = [0, 0, 1]
            step5 = [1, 0, 0]
            step6 = [1, 0, 1]
            
        # 반시계방향
        elif direction == "backward":
            step1 = [1, 1, 0]
            step2 = [1, 0, 0]
            step3 = [1, 0, 1]
            step4 = [0, 0, 1]
            step5 = [0, 1, 1]
            step6 = [0, 1, 0]
        
        step_list = [step1, step2, step3, step4, step5, step6]    
        
        return step_list
            
    # 사다리꼴 함수
    def trapezoid_function(self, theta_elec):
        pi = math.pi
        
        if 0 <= theta_elec < pi/6:
            return 6*theta_elec/pi
        elif pi/6 <= theta_elec < 5*pi/6:
            return 1
        elif 5*pi/6 <= theta_elec < 7*pi/6:
            return -6*theta_elec/pi + 6
        elif 7*pi/6 <= theta_elec < 11*pi/6:
            return -1
        elif 11*pi/6 <= theta_elec < 2*pi:
            return 6*theta_elec/pi - 12
        else:
            return 0
    
    # 센서리스 버전이 깔끔하려나?
    # def calc_velocity():
    
    # 출력축 각속도 계산, 이때 tau는 전부 출력단 기준, J_eq 역시
    def mechanical_equation(self, tau_elec, tau_load, J_eq, B, w_output, dt):
        
        dw_output = ((tau_elec - tau_load) - B * w_output) / J_eq
        w_output += dw_output * dt
        
        return w_output
    
    # V_desired가 들어오고 그게 3상으로 나누어졌을 때의 각 상에 걸리는 전류
    def electrical_equation(self, V_u, V_v, V_w, M, e_u, e_v, e_w, i_u, i_v, i_w, dt, R, L):
        
        di_u = (V_u - R * i_u - e_u) / (L - M) * dt
        di_v = (V_v - R * i_v - e_v) / (L - M) * dt
        di_w = (V_w - R * i_w - e_w) / (L - M) * dt
        
        I = di_u, di_v, di_w
        
        return I
    
    # Kt가 출력축 기준 토크 상수
    def calc_torque_elec(self, Kt, I):
        
        tau_e = Kt * I
        
        return tau_e
    
    
    def step(self, voltage, tau_load, theta_e, w_output, P, dt, Ke, Kt, M, R, L, J_eq, b, G, i_u, i_v, i_w):
        pi2 = 2 * math.pi
        
        # 모터축 속도 역산 (전기적 계산에 필요)
        w_motor = w_output * G
        
        # 1. 전기각 업데이트 (theta_e += w_motor * 극쌍수 * dt, 0~2π 범위 유지)
        theta_e = (theta_e + w_motor * P * dt) % pi2
        
        # 2. 사다리꼴 파형 계수 계산 (U, V, W 각 상 120도 위상차)
        trap_u = self.trapezoid_function(theta_e % pi2)
        trap_v = self.trapezoid_function((theta_e - pi2 / 3) % pi2)
        trap_w = self.trapezoid_function((theta_e - 2 * pi2 / 3) % pi2)
        
        # 3. 역기전력 계산 (e = Ke * w_motor * trap) — 모터축 기준
        E = self.calc_Back_BMF(Ke, w_motor)
        e_u = E * trap_u
        e_v = E * trap_v
        e_w = E * trap_w
        
        # 4. 인버터 전압 분배 (입력 전압을 사다리꼴 위상에 맞춰 각 상에 인가)
        V_u = voltage * trap_u
        V_v = voltage * trap_v
        V_w = voltage * trap_w
        
        # 5. 중성점 전압 계산 (Y결선: V_n = (ΣV - Σe) / 3)
        V_n = ((V_u + V_v + V_w) - (e_u + e_v + e_w)) / 3
        
        # 6. 전기자 방정식 (di = (V상 - V_n - e - R*i) / L * dt) — 모터축 기준
        di_u, di_v, di_w = self.electrical_equation(
            V_u - V_n, V_v - V_n, V_w - V_n,
            M, e_u, e_v, e_w,
            i_u, i_v, i_w,
            dt, R, L
        )
        i_u += di_u
        i_v += di_v
        i_w += di_w
        
        # 7. 전자기 토크 계산 (tau_e = Kt * Σ(i * trap))
        tau_e = Kt * (i_u * trap_u + i_v * trap_v + i_w * trap_w)
        
        # 8. 기계 방정식 (tau_e, tau_load, J_eq 모두 출력축)
        w_output = self.mechanical_equation(tau_e, tau_load, J_eq, b, w_output, dt)
        
        # 9. 출력 리턴
        current = (abs(i_u) + abs(i_v) + abs(i_w)) / 2.0
        theta_mech = theta_e / P
        theta_output = theta_mech / G
        theta_elec_q, theta_mech_q = self.dual_encoder(theta_e, theta_mech)
        return tau_e, current, w_output, theta_e, theta_output, theta_elec_q, theta_mech_q, i_u, i_v, i_w
    
    
# BLDC Plant로부터 받은 값들로 폐루프 PI 제어
# theta_desired, w_desired, i_desired 를 MainController로부터 받고 
# theta_current, w_current, i_current 를 Plant로부터 받음
class InnerPIController():
    
    def __init__(self):
        self.params()
    
    def params(self):
        self.dt     = 0.0001 # 10kHz
        self.Kp_pos = 1
        self.Kp_vel = 1
        self.Kp_cur = 2
        self.Ki_pos = 0.1
        self.Ki_vel = 0.1
        self.Ki_cur = 0.5
    
    def position_PI_control(self, theta_desired, theta_current, q_error_sum, Kp_pos, Ki_pos, dt, w_max):
        
        error = theta_desired - theta_current
        q_error_sum += error * dt
        
        w_desired = Kp_pos * error + Ki_pos * q_error_sum
        
        if w_desired > w_max:
            w_desired = w_max
            q_error_sum -= error * dt
        elif w_desired < -w_max:
            w_desired = -w_max
            q_error_sum -= error * dt
        
        return w_desired, q_error_sum

    def velocity_PI_control(self, w_desired, w_current, w_error_sum, Kp_vel, Ki_vel, dt, I_max):
        
        error = w_desired - w_current
        w_error_sum += error * dt
        
        I_desired = Kp_vel * error + Ki_vel * w_error_sum
        
        if I_desired > I_max:
            I_desired = I_max
            w_error_sum -= error * dt
        elif I_desired < -I_max:
            I_desired = -I_max
            w_error_sum -= error * dt
        
        return I_desired, w_error_sum
    
    def current_PI_control(self, I_desired, I_current, i_error_sum, Kp_cur, Ki_cur, dt, V_max, V_ff=0.0):
        
        error = I_desired - I_current
        i_error_sum += error * dt
        
        V_desired = Kp_cur * error + Ki_cur * i_error_sum + V_ff  # PI + 피드포워드
        
        if V_desired > V_max:
            V_desired = V_max
            i_error_sum -= error * dt
        elif V_desired < -V_max:
            V_desired = -V_max
            i_error_sum -= error * dt
        
        return V_desired, i_error_sum
    
    
# desired command 주는 곳 나중에 ModernRobotics의 RNEA나 DLS IK의 값을 받도록 설정해보자 
# 지금 여기는 개루프임
class MainController():
    
    def desired_command(self, command_mode, value):
        
        if command_mode == 'torque_mode':
            
            return value
            
        elif command_mode == 'velocity_mode':
            
            return value
        
        elif command_mode == 'position_mode':
            
            return value
    
    def compute_voltage(self, mode, target, pi, theta_current, w_current, I_current,
                        q_err_sum, w_err_sum, i_err_sum,
                        Kp_pos, Ki_pos, Kp_vel, Ki_vel, Kp_cur, Ki_cur,
                        dt, w_max, I_max, V_max, Kt, Ke, G):
        
        # 역기전력 피드포워드: V_ff = Ke * w_motor
        w_motor = w_current * G
        V_ff = Ke * w_motor
        
        if mode == 'position_mode':
            w_des, q_err_sum = pi.position_PI_control(target, theta_current, q_err_sum, Kp_pos, Ki_pos, dt, w_max)
            I_des, w_err_sum = pi.velocity_PI_control(w_des, w_current, w_err_sum, Kp_vel, Ki_vel, dt, I_max)
            V_des, i_err_sum = pi.current_PI_control(I_des, I_current, i_err_sum, Kp_cur, Ki_cur, dt, V_max, V_ff)
        
        elif mode == 'velocity_mode':
            I_des, w_err_sum = pi.velocity_PI_control(target, w_current, w_err_sum, Kp_vel, Ki_vel, dt, I_max)
            V_des, i_err_sum = pi.current_PI_control(I_des, I_current, i_err_sum, Kp_cur, Ki_cur, dt, V_max, V_ff)
        
        elif mode == 'torque_mode':
            I_des = target / Kt
            V_des, i_err_sum = pi.current_PI_control(I_des, I_current, i_err_sum, Kp_cur, Ki_cur, dt, V_max, V_ff)
        
        return V_des, q_err_sum, w_err_sum, i_err_sum
    

# 시뮬레이션 루프
plant = BLDCActuator()
pi_ctrl = InnerPIController()
mc = MainController()

# 초기값
theta_e = 0.0
theta_output = 0.0
w_output = 0.0
i_u = i_v = i_w = 0.0
q_err = w_err = i_err = 0.0
current = 0.0

mode = 'torque_mode'
target = 7.0
duration = 0.2
steps = int(duration / plant.dt)

# 로깅
t_log = []
torque_log = []
current_log = []
omega_log = []
voltage_log = []
emf_u_log = []
emf_v_log = []
emf_w_log = []

for i in range(steps): 
    t = i * plant.dt
    
    # 1. MainController: 목표값 → 전압
    voltage, q_err, w_err, i_err = mc.compute_voltage(
        mode, target, pi_ctrl, theta_output, w_output, current,
        q_err, w_err, i_err,
        pi_ctrl.Kp_pos, pi_ctrl.Ki_pos, pi_ctrl.Kp_vel, pi_ctrl.Ki_vel, pi_ctrl.Kp_cur, pi_ctrl.Ki_cur,
        plant.dt, plant.w_max, plant.I_max, plant.V_max, plant.Kt, plant.Ke, plant.G
    )
    
    # 2. Plant: 전압 → 토크, 전류, 각속도
    tau_e, current, w_output, theta_e, theta_output, theta_elec_q, theta_mech_q, i_u, i_v, i_w = plant.step(
        voltage, plant.tau_load, theta_e, w_output, plant.P, plant.dt,
        plant.Ke, plant.Kt, plant.M, plant.R, plant.L, plant.J_eq, plant.b, plant.G, i_u, i_v, i_w
    )
    
    # Back-EMF 로깅 (모터축 속도로 계산)
    pi2 = 2 * math.pi
    w_motor_log = w_output * plant.G
    e_u = plant.Ke * w_motor_log * plant.trapezoid_function(theta_e % pi2)
    e_v = plant.Ke * w_motor_log * plant.trapezoid_function((theta_e - pi2 / 3) % pi2)
    e_w = plant.Ke * w_motor_log * plant.trapezoid_function((theta_e - 2 * pi2 / 3) % pi2)
    
    t_log.append(t)
    torque_log.append(tau_e)
    current_log.append(current)
    omega_log.append(w_output)
    voltage_log.append(voltage)
    emf_u_log.append(e_u)
    emf_v_log.append(e_v)
    emf_w_log.append(e_w)

# 플롯
fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)

axes[0].plot(t_log, torque_log, 'b', linewidth=0.5)
axes[0].axhline(y=target, color='r', linestyle='--', label=f'target={target}')
axes[0].set_ylabel('Torque [Nm]')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t_log, current_log, 'g', linewidth=0.5)
axes[1].set_ylabel('Current [A]')
axes[1].grid(True)

axes[2].plot(t_log, omega_log, 'm', linewidth=0.5)
axes[2].set_ylabel('Output w [rad/s]')
axes[2].grid(True)

axes[3].plot(t_log, voltage_log, 'orange', linewidth=0.5)
axes[3].set_ylabel('Voltage [V]')
axes[3].grid(True)

axes[4].plot(t_log, emf_u_log, 'r', linewidth=0.5, label='U')
axes[4].plot(t_log, emf_v_log, 'g', linewidth=0.5, label='V')
axes[4].plot(t_log, emf_w_log, 'b', linewidth=0.5, label='W')
axes[4].set_ylabel('Back-EMF [V]')
axes[4].set_xlabel('Time [s]')
axes[4].legend()
axes[4].grid(True)

fig.suptitle(f'BLDC Output-Shaft Simulation ({mode}, target={target})')
plt.tight_layout()
plt.show()

