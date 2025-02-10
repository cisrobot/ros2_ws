import matplotlib.pyplot as plt
import datetime
import pandas as pd
import os
import numpy as np
from scipy.signal import medfilt

# 4개의 CSV 파일 경로를 리스트로 정리
log_files = [
    '/home/twins/ros2_ws/src/data/last/rad_1ms.csv',
    '/home/twins/ros2_ws/src/data/last/rad_-0.5.csv',
    '/home/twins/ros2_ws/src/data/last/rad_-1ms.csv',
    '/home/twins/ros2_ws/src/data/last/rad 0.5.csv',
]

# 그래프 저장 경로
save_path = "/home/twins/ros2_ws/src/data/last"

# 메디안 필터 생성 함수
def apply_median_filter(data, kernel_size):  # 기본적으로 커널 크기를 5로 설정
    return medfilt(data, kernel_size=kernel_size)

# 통합 그래프 생성 및 저장 함수
def plot_combined_graphs(log_files, save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    # 타임스탬프 및 파일 이름 설정
    timestamps = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    combined_speed_path = os.path.join(save_path, f"combined_angular_plot_{timestamps}.png")

    # 속도 그래프
    plt.figure(figsize=(12, 8))
    for log_file in log_files:
        file_name = os.path.basename(log_file).replace('.csv', '')
        try:
            data = pd.read_csv(log_file)

            if 'w_call' in data.columns:
                start_index = data[data['w_call'] >0.01].index[0]
                data = data.iloc[start_index:start_index + 100]  # 해당 시점부터 100개 데이터만 사용

            time_steps = range(len(data))
            w_ref = data['w_ref'].values
            w_curr = data['w_call'].values

            # 메디안 필터 적용
            w_curr_filtered = apply_median_filter(w_curr, kernel_size=11)  # 커널 크기 5 사용

            plt.plot(time_steps, w_ref, '--', linewidth=1.5)
            plt.plot(time_steps, w_curr_filtered, '-', linewidth=1.5)
        except Exception as e:
            print(f"Failed to process {log_file}: {e}")

    plt.ylim(-1.5, 1.5)  # y축 범위를 -1.5~1.5로 제한
    plt.xlabel('Time Step')
    plt.ylabel('Angular Velocity')
    plt.title('Combined Angular Control with Median Filtering')
    plt.legend()
    plt.grid()
    plt.savefig(combined_speed_path)
    plt.show()
    print(f"Combined speed graph saved at {combined_speed_path}")

# 통합 그래프 생성
plot_combined_graphs(log_files, save_path)
