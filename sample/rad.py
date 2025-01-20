import matplotlib.pyplot as plt
import datetime
import pandas as pd
import os

log_file = '/home/twins/ros2_ws/src/data/arduino_log_01_20_16_02_41.csv'

data = pd.read_csv(log_file)

def plot_and_save_graph(data, save_path):
    w_ref = data['w_ref'].values
    w_curr = data['w_call'].values
    time_steps = range(len(data))

    if not os.path.exists(save_path):
        os.makedirs(save_path)

    
    timestamps = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    save_path = os.path.join(save_path, f"speed_plot_{timestamps}")

    plt.figure(figsize = (10,6))
    plt.plot(time_steps, w_ref, 'r--', label='Set Goal', linewidth=2)  # 목표속도 (빨간 점선)
    plt.plot(time_steps, w_curr, 'g-', label='Current Speed', linewidth=2)  # 현재속도 (초록 실선)
    plt.xlabel('Time Step')
    plt.ylabel('Speed')
    plt.title('control')
    plt.legend()
    plt.grid()

    plt.savefig(save_path)
    plt.show()

    print(f"그래프가 {save_path}에 저장되었습니다. ")

def plot_and_pwm(data, save_path):
    pwm = data['ard_steer_pwm'].values
    time_steps = range(len(data))

    if not os.path.exists(save_path):
        os.makedirs(save_path)

    timestamps = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    save_path = os.path.join(save_path, f"pwm_plot_{timestamps}")

    plt.figure(figsize = (10,6))
    plt.plot(time_steps, pwm, 'b-', label='PWM', linewidth=2)  

    plt.xlabel('Time Step')
    plt.ylabel('PWM')
    plt.title('PWM')
    plt.legend()
    plt.grid()

    plt.savefig(save_path)
    plt.show()

    print(f"그래프가 {save_path}에 저장되었습니다. ")


save_path = "/home/twins/ros2_ws/src/data/rad_graph"
save_path1 = "/home/twins/ros2_ws/src/data/rad_pwm"
plot_and_save_graph(data, save_path)
plot_and_pwm(data, save_path1)