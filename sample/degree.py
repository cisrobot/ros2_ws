import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_mean_points_with_normalized_pwm_and_radians(file_1180_path, file_1780_path):
    # CSV 파일을 불러옵니다.
    file_pwm1180 = pd.read_csv(file_1180_path)
    file_pwm1780 = pd.read_csv(file_1780_path)

    # 'deg' 값이 NaN이 아닌, 즉 유효한 각도 값이 있는 행만 필터링합니다.
    valid_angle_1180pwm = file_pwm1180[file_pwm1180['deg'].notna()]
    valid_angle_1780pwm = file_pwm1780[file_pwm1780['deg'].notna()]

    # 유효한 각도 값의 평균을 계산합니다.
    mean_deg_1180 = valid_angle_1180pwm['deg'].mean()
    mean_deg_1780 = valid_angle_1780pwm['deg'].mean()

    # degree 값을 radian으로 변환합니다.
    mean_rad_1180 = np.deg2rad(mean_deg_1180)
    mean_rad_1780 = np.deg2rad(mean_deg_1780)

    # PWM 값을 정규화합니다 (normalized_pwm = (pwm - 1480) / (1780 - 1480))
    normalized_pwm_1180 = (1180 - 1480) / (1780 - 1480)
    normalized_pwm_1780 = (1780 - 1480) / (1780 - 1480)

    # 두 점을 잇는 1차 함수의 기울기와 절편을 구합니다.
    slope = (normalized_pwm_1780 - normalized_pwm_1180) / (mean_rad_1780 - mean_rad_1180)
    intercept = normalized_pwm_1180 - slope * mean_rad_1180

    # 시각화
    plt.figure(figsize=(8, 6))
    
    # 1180 PWM에 대한 시각화
    plt.scatter(mean_rad_1180, normalized_pwm_1180, color='blue', label=f'1180 PWM (mean rad: {mean_rad_1180:.2f})')
    
    # 1780 PWM에 대한 시각화
    plt.scatter(mean_rad_1780, normalized_pwm_1780, color='red', label=f'1780 PWM (mean rad: {mean_rad_1780:.2f})')

    # 두 점을 잇는 직선을 플롯합니다.
    rad_values = np.linspace(mean_rad_1180, mean_rad_1780, 100)
    pwm_values = slope * rad_values + intercept
    plt.plot(rad_values, pwm_values, color='green', label='Linear Function')

    # 1차 함수 식을 그래프에 추가
    equation_text = f'y = {slope:.2f}x + {intercept:.2f}'
    plt.text(mean_rad_1180, normalized_pwm_1780, equation_text, fontsize=12, color='green')
    print(equation_text)

    # 플롯 설정
    plt.title('Mean Points with Normalized PWM and Linear Function')
    plt.xlabel('Radians')
    plt.ylabel('Normalized PWM (-1 to 1)')
    plt.legend()
    plt.grid(True)
    plt.show()

# 함수 호출 예시
plot_mean_points_with_normalized_pwm_and_radians('/home/kon/Desktop/THE/1180pwm.csv', '/home/kon/Desktop/THE/1780pwm.csv')

#pwm = 2.49@+ -0.01
