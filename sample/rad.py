import matplotlib.pyplot as plt
import datetime
import pandas as pd
import os
from scipy.signal import medfilt

log_file = '/home/twins/ros2_ws/src/data/arduino_log_01_25_22_50_28.csv'

data = pd.read_csv(log_file)

def plot_and_save_graph_filtered(data, save_path):
    w_ref = data['w_ref'].values
    w_curr = data['w_call'].values

    # Apply median filter
    
    w_curr= medfilt(w_curr, kernel_size=11)  # Adjust kernel size as needed

    time_steps = range(len(data))

    if not os.path.exists(save_path):
        os.makedirs(save_path)

    timestamps = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    save_path = os.path.join(save_path, f"speed_plot_filtered_{timestamps}")

    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, w_ref, 'r--', label='Set Goal (Filtered)', linewidth=2)  # Filtered 목표속도
    plt.plot(time_steps, w_curr, 'g-', label='Current Speed (Filtered)', linewidth=2)  # Filtered 현재속도
    plt.xlabel('Time Step')
    plt.ylabel('Speed')
    plt.title('Control with Median Filter')
    plt.legend()
    plt.grid()

    plt.savefig(save_path)
    plt.show()

    print(f"Filtered 그래프가 {save_path}에 저장되었습니다.")

save_path = "/home/twins/ros2_ws/src/data/rad_graph_filtered"
plot_and_save_graph_filtered(data, save_path)
