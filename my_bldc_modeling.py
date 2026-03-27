import mass
import numpy
import control
import matplotlib.pyplot as plt

# InnerController로부터 V_desired를 받아서 w_motor, E, I를 내보냄
class BLDCActuator():
    
    # 엑추에이터의 파라미터 정의
    def params():
    
    # V_desired를 받아서 V_desired = R * I + L(d_I_actual/dt) + e 
    # 근데 실제로는 e를 따로 더해준다는데 같이 더해줘야하나? 아니다 차라리 e를 계산하는 함수를 만들자
    
    # 역기전력 계산 함수
    def calc_Back_BMF():
        
        back_bmf = ke * w_motor
        
        return back_bmf
    
    # 듀얼 엔코더 하나는 2^17, 2^18 해상도로 각도를 리턴
    def dual_encoder():
        
        return theta
    
    # 센서리스 버전이 깔끔하려나?
    def calc_velocity():
        
        w_rawer = 
        
         
    
    
    
# BLDC Plant로부터 받은 값들로 폐루프 PI 제어

# 위치제어 구현할까?
# 속도제어 구현해야함
# 토크제어 구현해야함

# MIT 모드? 같은 것으로 해서 position, velocity, current (아니면 Kt 곱해서 torque) 반환하도록 해보자
class InnerPIController():
    
    def params
    
    def position_PI_control
    
    def velocity_PI_control
    
    def current_PI_control
    
# desired command 주는 곳 나중에 ModernRobotics의 RNEA나 DLS IK의 값을 받도록 설정해보자 
# 지금 여기는 개루프임
class MainController():
    
    
     