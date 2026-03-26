import math
import numpy
import control
import matplotlib.pyplot as plt

'''
3상 BLDC 모터의 모델링

def base - 전기자 모델링 

def torque_equation - 토크 방정식 (여기서 마찰토크 뺄 수 있게 만들자)

def Back EMF - 역기전력 방정식
'''

# 전압을 입력으로 받아서 각속도와 전류, 실제 토크를 출력으로 내놓는 플랜트

class MyActuatorPlant():
    
    def __init__(self):
        self.configuration()
        self.reset()
        
    # MyActuator RMD-X4-P36-36-E 파라미터
    def configuration(self):
        # === RMD-X4-P36-36-E (Without Brake, EtherCAT & CAN BUS) ===
        # Mechanical Specs
        self.Weight = 0.36            # 무게 kg
        self.gRatio = 36              # 기어비
        self.Pole_Pair = 13           # 극쌍수
        self.Phase_Connection = "Y"   # 3상 Y결선
        self.Backlash = 10            # 백래시 Arcmin
        self.BackDriveTorque = 1.14   # 역구동 토크 Nm
        self.Output_Bearing = "Crossed Roller Bearings"
        self.Axial_Load_Suffer = 1.3  # 축방향 하중(Suffer) KN
        self.Axial_Load_Stress = 1.3  # 축방향 하중(Stress) KN
        self.Radial_Load = 1.5        # 반경방향 하중 KN
        self.Inertia = 0.3            # 관성 Kg·cm²
        self.J = self.Inertia * 1e-4  # 관성 모멘트 kg·m² (0.3 Kg·cm² → 0.00003 kg·m²)
        self.Encoder = "Dual Encoder ABS 17BIT (Input) / 18BIT (Output)"
        self.Control_Accuracy = 0.01  # 제어 정밀도 Degree
        self.Communication = "EtherCAT & CAN BUS"
        self.Insulation_Grade = "F"

        # Electrical Specs
        self.max_V = 24               # 입력 전압 V
        self.No_Load_Speed = 111      # 무부하 속도 RPM
        self.No_Load_Current = 0.9    # 무부하 전류 A
        self.Rated_Speed = 83         # 정격 속도 RPM
        self.Rated_Torque = 10.5      # 정격 토크 Nm
        self.Rated_Output_Power = 100 # 정격 출력 W
        self.Rated_Phase_Current = 6.1  # 정격 상전류 Arms
        self.Peak_Torque = 34.0       # 피크 토크 Nm
        self.Peak_Phase_Current = 21.5  # 피크 상전류 Arms
        self.Efficiency = 63.1        # 효율 %

        self.Back_EMF = 6             # 역기전력 Vdc/Krpm
        self.Kt = 1.9                 # 모듈 토크 상수 Nm/A
        self.R = 0.35                 # 모터 상저항 Ω
        self.L = 0.00017              # 모터 상인덕턴스 0.17mH

        # Derived / Simulation Parameters
        self.dt = 0.0001              # 샘플링 타임 10kHz
        self.Ke = self.Back_EMF / 1000 * (60 / (2 * 3.141592))  # Vdc/Krpm → V/(rad/s)
        self.Kt_motor = self.Kt / self.gRatio  # 모터 토크 상수 Nm/A
        self.LoadTorque = 0.0         # 부하 토크 Nm

        # Stall Torque Data (Torque → Temp Rise, Stall Time, Phase Current)
        self.stall_torque_data = {
            # torque(Nm): (temp_rise(°C), stall_time(s), phase_current(Arms))
            17.25: (30, 15, 9.2),
            23:    (58, 10, 12.7),
            28.75: (41,  5, 16.3),
            34.5:  (50,  3, 21.2),
        }

    '''
    출력 베어링은 Crossed Roller Bearing 이라고 하네
    정격 출력은 100W
    입력된 전기 에너지 중 실제 기계적 회전에너지로 전환되는 비율은 63.1%
    '''

    # HC (6시), HB (10시), HA (2시)
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
        
        step_sequence = [step1, step2, step3, step4, step5, step6]    
        return step_sequence


    def torque_equation(self, e_u, i_u, e_v, i_v, e_w, i_w, w_motor):
        tau_e = (e_u * i_u + e_v * i_v + e_w * i_w) / w_motor
        return tau_e


    def trapezoid_function(self, theta):
        pi = math.pi
        
        if 0 <= theta < pi/6:
            theta = 6*theta/pi
        elif pi/6 <= theta < 5*pi/6:
            theta = 1
        elif 5*pi/6 <= theta < 7*pi/6:
            theta = -6*theta/pi + 6
        elif 7*pi/6 <= theta < 11*pi/6:
            theta = -1
        elif 11*pi/6 <= theta < 2*pi:
            theta = 6*theta/pi-12
        
        return theta


    def Back_EMF_function(self, w_motor):
        E = self.Ke * w_motor
        return E


    def mechanical_equation(self, tau_e, tau_l):
        dw_motor = (tau_e - tau_l) / self.J
        return dw_motor


    def electrical_equation(self, V, e_u, e_v, e_w, i_u, i_v, i_w):
        di_u = (V - e_u - self.R * i_u) / self.L * self.dt
        di_v = (V - e_v - self.R * i_v) / self.L * self.dt
        di_w = (V - e_w - self.R * i_w) / self.L * self.dt
        return di_u, di_v, di_w

    '''
    def friction_torque_function(self, w_motor, friction_coefficient):
        tau_friction = friction_coefficient * w_motor
        return tau_friction
    '''
    
    def reset(self):
        self.i_u = 0.0
        self.i_v = 0.0
        self.i_w = 0.0
        self.omega_motor = 0.0
        self.theta_e = 0.0

    def step(self, voltage, load_torque=0.0):
        pi2 = 2 * math.pi
        
        # 1. 전기각 업데이트
        self.theta_e = (self.omega_motor * self.Pole_Pair * self.dt + self.theta_e) % pi2
        
        # 2. 사다리꼴 파형 계수 계산 (역기전력 및 이상적 정류에 사용)
        trap_u = self.trapezoid_function(self.theta_e)
        trap_v = self.trapezoid_function((self.theta_e - pi2 / 3) % pi2)
        trap_w = self.trapezoid_function((self.theta_e - 2 * pi2 / 3) % pi2)
        
        # 3. 역기전력 계산
        e_u = self.Ke * self.omega_motor * trap_u
        e_v = self.Ke * self.omega_motor * trap_v
        e_w = self.Ke * self.omega_motor * trap_w
        
        # 4. 이상적인 인버터 전압 분배 (입력 전압을 각 상의 역기전력 위상에 맞춰 인가)
        V_u = voltage * trap_u
        V_v = voltage * trap_v
        V_w = voltage * trap_w
        V_n = ((V_u + V_v + V_w) - (e_u + e_v + e_w)) / 3
        
        # 5. 전기자 방정식 (각 상별로 다른 전압 인가)
        di_u = (V_u - V_n - e_u - self.R * self.i_u) / self.L * self.dt
        di_v = (V_v - V_n - e_v - self.R * self.i_v) / self.L * self.dt
        di_w = (V_w - V_n - e_w - self.R * self.i_w) / self.L * self.dt
        
        self.i_u += di_u
        self.i_v += di_v
        self.i_w += di_w
        
        # 6. 전자기 토크 계산 (올바른 BLDC 토크 방정식 적용)
        # Kt_motor(0.0527)와 Ke(0.0573)가 물리적으로 유사한 값을 가지므로 논리적으로 맞습니다.
        tau_e = self.Kt_motor * (self.i_u * trap_u + self.i_v * trap_v + self.i_w * trap_w)
        
        # 7. 기계 방정식
        tau_load_motor = load_torque / self.gRatio
        dw = (tau_e - tau_load_motor) / self.J * self.dt
        self.omega_motor += dw
        
        # 8. 출력 (전류는 RMS나 유효 전류 크기로 추정하기 위해 절대값 평균 사용)
        output_torque = tau_e * self.gRatio
        current_magnitude = (abs(self.i_u) + abs(self.i_v) + abs(self.i_w)) / 2.0
        
        return output_torque, current_magnitude, self.omega_motor


# 목표 토크를 받아서 플랜트에 줄 전압 계산, 입력: desired torque, 출력: voltage

class MyActuatorInnerController():
    
    def __init__(self, kp=0.45, ki=0.00, max_voltage=24.0):
        self.Kp = kp
        self.Ki = ki
        self.max_voltage = max_voltage
        self.integral_error = 0.0
        
    def compute_voltage(self, tau_desired, tau_actual, dt=0.0001):
        tau_error = tau_desired - tau_actual
        self.integral_error += tau_error * dt
        
        voltage = self.Kp * tau_error + self.Ki * self.integral_error
        
        # Anti-windup (적분기 포화 방지)
        if voltage > self.max_voltage:
            voltage = self.max_voltage
            self.integral_error -= tau_error * dt 
        elif voltage < -self.max_voltage:
            voltage = -self.max_voltage
            self.integral_error -= tau_error * dt
            
        return voltage
    
# 플랜트와 이너 컨트롤러 기반으로 출력된 토크로 에러 정의해서 PD 제어로 tau_ff + tau_fb로 제어하는 매니퓰레이터 컨트롤러

class SingleActuatorController():
    
    def __init__(self):
        self.plant = MyActuatorPlant()
        self.controller = MyActuatorInnerController()
    
    def run(self, tau_desired=5.0, duration=2.0, load_torque=5.0):

        dt = self.plant.dt
        steps = int(duration / dt)
        
        t_log = []
        torque_log = []
        current_log = []
        omega_log = []
        voltage_log = []
        emf_u_log = []
        emf_v_log = []
        emf_w_log = []
        
        self.plant.reset()
        self.controller.prev_tau_error = 0.0
        
        for i in range(steps):
            t = i * dt
            tau_actual = torque_log[-1] if torque_log else 0.0
            
            # 이너 컨트롤러: 목표 토크 → 전압
            voltage = self.controller.compute_voltage(tau_desired, tau_actual, dt)
            
            # 플랜트: 전압 → 토크, 전류, 각속도
            torque, current, omega = self.plant.step(voltage, load_torque)
            
            # Back-EMF 사다리꼴 파형 로깅
            pi2 = 2 * math.pi
            theta_e = self.plant.theta_e
            e_u = self.plant.Ke * self.plant.omega_motor * self.plant.trapezoid_function(theta_e % pi2)
            e_v = self.plant.Ke * self.plant.omega_motor * self.plant.trapezoid_function((theta_e - pi2 / 3) % pi2)
            e_w = self.plant.Ke * self.plant.omega_motor * self.plant.trapezoid_function((theta_e - 2 * pi2 / 3) % pi2)
            
            t_log.append(t)
            torque_log.append(torque)
            current_log.append(current)
            omega_log.append(omega)
            voltage_log.append(voltage)
            emf_u_log.append(e_u)
            emf_v_log.append(e_v)
            emf_w_log.append(e_w)
        
        # 플롯
        fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
        
        axes[0].plot(t_log, torque_log, 'b', linewidth=0.5)
        axes[0].axhline(y=tau_desired, color='r', linestyle='--', label=f'desired={tau_desired} Nm')
        axes[0].set_ylabel('Torque [Nm]')
        axes[0].legend()
        axes[0].grid(True)
        
        axes[1].plot(t_log, current_log, 'g', linewidth=0.5)
        axes[1].set_ylabel('Current [A]')
        axes[1].grid(True)
        
        axes[2].plot(t_log, omega_log, 'm', linewidth=0.5)
        axes[2].set_ylabel('Motor ω [rad/s]')
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
        
        fig.suptitle(f'MyActuator Plant + InnerController (τ_des={tau_desired} Nm, load={load_torque} Nm)')
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    ctrl = SingleActuatorController()
    ctrl.run(tau_desired=5.0, duration=0.5, load_torque=0.0)
