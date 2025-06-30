import matplotlib.pyplot as plt
import csv

# 初始化数据列表
time1 = []
pos_now1 = []
time2 = []
pos_now2 = []
time3 = []
pos_now3 = []
time4 = []
pos_now4 = []
time5 = []
pos_now5 = []
# 读取 CSV 文件
with open('PID_p_log_2.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # 跳过表头
    for row in csvreader:
        time1.append(float(row[0]))
        pos_now1.append(-float(row[2]))
with open('PID_pid_log.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # 跳过表头
    for row in csvreader:
        time2.append(float(row[0]))
        pos_now2.append(-float(row[2]))
with open('scurve_pid_v_f_4.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # 跳过表头
    for row in csvreader:
        time3.append(float(row[0]))
        pos_now3.append(-float(row[2]))
with open('PID_fuzzy_4.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # 跳过表头
    for row in csvreader:
        time4.append(float(row[0]))
        pos_now4.append(-float(row[2]))
with open('scurve_pid_v_p.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # 跳过表头
    for row in csvreader:
        time5.append(float(row[0]))
        pos_now5.append(-float(row[2]))

# 绘制图表
plt.plot(time1, pos_now1, marker='o', markersize=1, label='p')
plt.plot(time2, pos_now2, marker='o', markersize=1, label='pid')
plt.plot(time3, pos_now3, marker='o', markersize=1, label='scurve_pid_f')
plt.plot(time4, pos_now4, marker='o', markersize=1, label='fuzzy_pid')
plt.plot(time5, pos_now5, marker='o', markersize=1, label='scurve_pid')

# 添加图例
plt.legend()

# 添加标题和标签
plt.title('Pos_Now vs Time')
plt.xlabel('Time')
plt.ylabel('Pos_Now')

# 显示图表
plt.grid(True)
plt.savefig('PID_i.png')
plt.show()