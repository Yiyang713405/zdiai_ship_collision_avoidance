import random
import matplotlib.pyplot as plt

# 随机生成 10 个障碍物，每个障碍物的宽高范围为 5~10
obstacles = []
for i in range(10):
    w, h = random.randint(5, 10), random.randint(5, 10)
    x, y = random.uniform(-50, 50), random.uniform(-50, 50)
    obstacles.append((x, y, w, h))

# 绘制障碍物
for obstacle in obstacles:
    x, y, w, h = obstacle
    rect = plt.Rectangle((x - w / 2, y - h / 2), w, h, color='gray')
    plt.gca().add_patch(rect)

# 绘制起点和终点
start = (-40, -40)
goal = (40, 40)
plt.plot(start[0], start[1], 'go', markersize=10)
plt.plot(goal[0], goal[1], 'ro', markersize=10)

# 设置坐标轴范围
plt.xlim([-60, 60])
plt.ylim([-60, 60])
plt.show()