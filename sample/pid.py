import matplotlib.pyplot as plt
import datetime
import pandas as pd
import os
import numpy as np

log_file = '/home/twins/ros2_ws/src/data/last/arduino_log_01_19_23_48_31_0.5ms.csv'

data = pd.read_csv(log_file)

def plot_and_save_graph(data, save_path):
    v_ref = data['v_ref'].values
    v_curr = data['v_call'].values
    time_steps = range(len(data))

    if not os.path.exists(save_path):
        os.makedirs(save_path)

    
    timestamps = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    save_path = os.path.join(save_path, f"speed_plot_{timestamps}")

    plt.figure(figsize = (10,6))
    plt.plot(time_steps, v_ref, 'r--', label='Set Goal', linewidth=2)  # 목표속도 (빨간 점선)
    plt.plot(time_steps, np.round((v_curr),3), 'g-', label='Current Speed', linewidth=2)  # 현재속도 (초록 실선)
    plt.xlabel('Time Step')
    plt.ylabel('Speed')
    plt.title('control')
    plt.legend()
    plt.grid()

    plt.savefig(save_path)
    plt.show()

    print(f"그래프가 {save_path}에 저장되었습니다. ")

def plot_and_pwm(data, save_path):
    pwm = data['ard_throttle_pwm'].values
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


save_path = "/home/twins/ros2_ws/src/data/last"
save_path1 = "/home/twins/ros2_ws/src/data/last"
plot_and_save_graph(data, save_path)
plot_and_pwm(data, save_path1)