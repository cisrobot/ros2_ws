import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# CSV 파일 경로
csv_file = "/home/kon/ros2_ws/src/zed_data_logger/zed_data.csv"  # 업로드된 파일 경로로 변경
data = pd.read_csv(csv_file)

# 시간 스텝 데이터 준비
time_steps = data.index.to_numpy()

# X, Y, Z 축의 최대 및 최소값을 계산하여 스케일 설정
min_range = min(data[['Position_X', 'Position_Y', 'Position_Z']].min())
max_range = max(data[['Position_X', 'Position_Y', 'Position_Z']].max())

# 3D 그래프를 그리기 위해 3D 축 생성
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Position 데이터를 빨간색으로 3D 플로팅
ax.plot(data['Position_X'].to_numpy(), 
        data['Position_Y'].to_numpy(), 
        data['Position_Z'].to_numpy(), 
        color='r', label='Position Path')

# 축의 스케일을 동일하게 설정
ax.set_xlim([min_range, max_range])
ax.set_ylim([min_range, max_range])
ax.set_zlim([min_range, max_range])

# 그래프 제목과 라벨 설정
ax.set_xlabel('Position X')
ax.set_ylabel('Position Y')
ax.set_zlabel('Position Z')

# 범례 추가
ax.legend()

# 그래프 표시
plt.show()