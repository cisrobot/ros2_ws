import pandas as pd
import numpy as np
import csv
from datetime import datetime

# CSV 파일을 읽어옵니다
df = pd.read_csv('/home/kon/Desktop/THE/pwm_t1180.csv')

# 필요한 열을 정의
V_x = df['Twist_Linear_X']
V_y = df['Twist_Linear_Y']
W = df['Twist_Angular_Z']

# 차량의 축간 거리 L 정의
L = 0.38
beta = np.arctan(V_y/V_x)
cos_beta = np.cos(beta)
# 속도 계산
V = abs(np.sqrt(V_x**2 + V_y**2))

# 반지름 계산
df['radius'] = V /W.where(W != 0, float('nan'))  # meter


# 조향각(arc tangent)을 계산하여 열로 추가
df['deg'] = np.degrees(np.arctan(L / (cos_beta * df['radius'])))  # 조향각

def save_data_to_csv(time_steps, linear_x, linear_y, angular_z, radius, deg):
    # 현재 시간을 기반으로 파일 이름 생성
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = f"/home/kon/ros2_ws/src/sample/new_data/theta/zed_filter_data_{current_time}.csv"

    # 새로운 CSV 파일 열기 및 데이터 쓰기
    with open(file_name, "w", newline='') as file:
        writer = csv.writer(file)
    
        # 헤더 작성
        writer.writerow([
            'Time_Step','Twist_Linear_X', 'Twist_Linear_Y', 
           'Twist_Angular_Z', 'radius','deg'
        ])

        for i in range(len(time_steps)):
            writer.writerow([
                time_steps[i],
                linear_x[i],
                linear_y[i],
                angular_z[i],
                radius[i],
                deg[i]
            ])
    print(f"Data saved to {file_name}")


# 원하는 열을 출력
save_data_to_csv(df.index.to_numpy(),V_x, V_y, W, df['radius'],df['deg'])
print(df[['Twist_Linear_X', 'Twist_Linear_Y','Twist_Angular_Z', 'radius', 'deg']])