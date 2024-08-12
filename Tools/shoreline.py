import pandas as pd
import matplotlib.pyplot as plt


# data = pd.read_csv('D:/AA船舶避碰/AvoidingCollision.ChartElements3.csv')
data = pd.read_csv('D:/OneDrive/OneDrive - whut.edu.cn/015-Code/yfproject_2022/PathPlanning-Highlander/AvoidingCollision.ChartElements3.csv')
Lat = data['lat_d'].tolist()
Lon = data['lon_d'].tolist()
island_startpoint = []
island_endpoint = []
island_lat = []
island_lon = []
# 判断岛屿的起始点
for i in range(1, len(Lat)):
    for j in range(0, i):
        if Lat[i] == Lat[j] and Lon[i] == Lon[j]:
            island_startpoint.append(j)
            island_endpoint.append(i)
# 筛选海岸线的点
# 先把岛屿的点筛选出来
# 将所有点和岛屿的点经纬度合并，判断不属于岛屿的经纬度坐标即是海岸线
# 将海岸线坐标拆分
for i in range(len(island_startpoint)):
    for j in range(len(Lat)):
        if island_startpoint[i] <= j <= island_endpoint[i]:
            island_lat.append(Lat[j])
            island_lon.append(Lon[j])
all_position = list(zip(Lat, Lon))
island_position = list(zip(island_lat, island_lon))
shoreline_position = []
for i in range(len(all_position)):
    if all_position[i] in island_position:
        continue
    else:
        shoreline_position.append(all_position[i])
shoreline_position = list(zip(*shoreline_position))
shoreline_lon = shoreline_position[1]
shoreline_lat = shoreline_position[0]

print(len(Lat))
print(island_startpoint, island_endpoint)
print(len(island_lat))
print(len(shoreline_lon))

# 画图
for i in range(len(island_startpoint)):
    plt.plot(Lon[island_startpoint[i]:island_endpoint[i]+1], Lat[island_startpoint[i]:island_endpoint[i]+1],
             c='r', alpha=1)
plt.plot(shoreline_lon, shoreline_lat, c='r', alpha=1)
plt.title('shoreline')
plt.xlabel('Lon')
plt.ylabel('Lat')
plt.show()


