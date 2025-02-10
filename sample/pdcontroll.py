import matplotlib.pyplot as plt
import datetime
import pandas as pd
import os
import numpy as np

# 4개의 CSV 파일 경로를 리스트로 정리
log_files = [
    '/home/twins/ros2_ws/src/data/last/arduino_log_01_19_23_48_31_0.5ms.csv',
    '/home/twins/ros2_ws/src/data/last/arduino_log_01_19_23_59_10_1ms.csv',
    '/home/twins/ros2_ws/src/data/last/last2ms.csv',
    '/home/twins/ros2_ws/src/data/last/arduino_log_01_20_15_43_21_1.5ms.csv',
]

# 그래프 저장 경로
save_path = "/home/twins/ros2_ws/src/data/last"

# 통합 그래프 생성 및 저장 함수
def plot_combined_graphs(log_files, save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    # 타임스탬프 및 파일 이름 설정
    timestamps = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    combined_speed_path = os.path.join(save_path, f"combined_speed_plot_{timestamps}.png")
    combined_pwm_path = os.path.join(save_path, f"combined_pwm_plot_{timestamps}.png")

    # 속도 그래프
    plt.figure(figsize=(10, 6))
    for log_file in log_files:
        file_name = os.path.basename(log_file).replace('.csv', '')
        try:
            data = pd.read_csv(log_file)

            # `v_call`이 0.2보다 큰 값이 처음 발생한 시점 찾기
            start_index = data[data['v_call'] > 0.2].index[0]
            data = data.iloc[start_index:start_index + 100]  # 해당 시점부터 100개 데이터만 사용

            time_steps = range(len(data))
            v_ref = data['v_ref'].values
            v_curr = data['v_call'].values

            plt.plot(time_steps, v_ref, '--', linewidth=1.5)
            plt.plot(time_steps, np.round(v_curr, 3), '-', linewidth=1.5)
        except Exception as e:
            print(f"Failed to process {log_file}: {e}")

    plt.ylim(0, 3)  # y축 범위를 0~3으로 제한
    plt.xlabel('Time Step')
    plt.ylabel('Speed')
    plt.title('Combined Speed Control')
    plt.legend()
    plt.grid()
    plt.savefig(combined_speed_path)
    plt.show()
    print(f"Combined speed graph saved at {combined_speed_path}")

    # PWM 그래프
    plt.figure(figsize=(10, 6))
    for log_file in log_files:
        file_name = os.path.basename(log_file).replace('.csv', '')
        try:
            data = pd.read_csv(log_file)

            # `v_call`이 0보다 큰 값이 처음 발생한 시점 찾기
            start_index = data[data['v_call'] > 0].index[0]
            data = data.iloc[start_index:start_index + 100]  # 해당 시점부터 100개 데이터만 사용

            time_steps = range(len(data))
            pwm = data['ard_throttle_pwm'].values

            plt.plot(time_steps, pwm, '-', label=f'{file_name} - PWM', linewidth=1.5)
        except Exception as e:
            print(f"Failed to process {log_file}: {e}")

    plt.xlabel('Time Step')
    plt.ylabel('PWM')
    plt.title('Combined PWM Control')
    plt.legend()
    plt.grid()
    plt.savefig(combined_pwm_path)
    plt.show()
    print(f"Combined PWM graph saved at {combined_pwm_path}")

# 통합 그래프 생성
plot_combined_graphs(log_files, save_path)
