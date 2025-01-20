import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter
import csv
from datetime import datetime

# csv 파일 읽기
csv_file = "/home/kon/Desktop/THE/pwm_t1780.csv"  # csv가 저장된 경로
data = pd.read_csv(csv_file)  # pd 라이브러리를 사용하여 csv 파일을 읽어 저장할 변수

# 데이터를 CSV 파일로 저장하는 함수
def save_data_to_csv(time_steps, position_x, linear_x, linear_y, throttle_pwm, angular_z, steer_pwm):
    # 현재 시간을 기반으로 파일 이름 생성
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = f"/home/kon/ros2_ws/src/sample/new_data/velocity/zed_filter_data_{current_time}.csv"

    # 새로운 CSV 파일 열기 및 데이터 쓰기
    with open(file_name, "w", newline='') as file:
        writer = csv.writer(file)
        
        # 헤더 작성
        writer.writerow([
            'Time_Step', 'Position_X', 'Twist_Linear_X', 'Twist_Linear_Y', 
            'Throttle_PWM', 'Twist_Angular_Z', 'Steer_PWM'
        ])
        
          # 각 그래프에서 사용한 데이터를 순서대로 저장
        for i in range(len(time_steps)):
            writer.writerow([
                time_steps[i],
                position_x[i],
                linear_x[i],
                linear_y[i],
                throttle_pwm[i],
                angular_z[i],
                steer_pwm[i]
            ])
    
    print(f"Data saved to {file_name}")

# 그래프를 그리는 함수
def make_plot():
    # dataframe의 인덱스를 넘파이 배열로 변환하여, 그래프의 x축으로 사용할 시간 스텝 데이터를 준비
    time_steps = data.index.to_numpy()

    fig, axs = plt.subplots(5, 1, figsize=(10, 10))

    axs[0].plot(time_steps, data['Position_X'].to_numpy(), label='Position X')
    axs[0].set_title('Position')  # 제목
    axs[0].legend()  # 범례
    axs[0].set_xlabel('Time Step')  # x축 이름
    axs[0].set_ylabel('Values')  # y축 이름

    filter_Linear_X = median_filter(data['Twist_Linear_X'].to_numpy(),size = 5)
    filter_Linear_Y = median_filter(data['Twist_Linear_Y'].to_numpy(),size = 5)
    axs[1].plot(time_steps, filter_Linear_X, label='Linear Velocity X')
    axs[1].plot(time_steps,filter_Linear_Y, label='Linear Velocity Y')
    axs[1].set_title('Linear Velocity (m/s)')
    axs[1].legend()
    axs[1].set_xlabel('Time Step')
    axs[1].set_ylabel('Values')

    axs[2].plot(time_steps, data['Throttle_PWM'].to_numpy(), label='Throttle_PWM')
    axs[2].set_title('PWM')
    axs[2].legend()
    axs[2].set_xlabel('Time Step')
    axs[2].set_ylabel('Values')

    filter_Angular_Z = median_filter(data['Twist_Angular_Z'].to_numpy(),size = 7)
    axs[3].plot(time_steps, filter_Angular_Z, label='Angular Velocity Z')
    axs[3].set_title('Angular Velocity (rad/s)')
    axs[3].legend()
    axs[3].set_xlabel('Time Step')
    axs[3].set_ylabel('Values')

    axs[4].plot(time_steps, data['Steer_PWM'].to_numpy(), label='Steer_PWM')
    axs[4].set_title('Steer_PWM')
    axs[4].legend()
    axs[4].set_xlabel('Time Step')
    axs[4].set_ylabel('Values')

    plt.tight_layout()  # 플롯의 레이아웃을 최적화하여 요소들이 겹치지 않도록 합니다.
    plt.show()  # 그래프를 화면에 표시합니다.

    # 그래프에 사용된 데이터를 CSV 파일로 저장
    #ave_data_to_csv(time_steps,data['Position_X'].to_numpy(), filter_Linear_X, data['Twist_Linear_Y'].to_numpy() ,data['Throttle_PWM'].to_numpy(), filter_Angular_Z , data['Steer_PWM'].to_numpy())

# 그래프 함수 호출
make_plot()
