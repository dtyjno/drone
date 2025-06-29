import tkinter as tk 
import os
import yaml

filename = '/home/limile/new_ardupilot_ws/src/px4_ros_com/config/OffboardControl.yaml'

# ---------- 读取 YAML ----------
def load_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)

# ---------- 保存 YAML ----------
def save_yaml(filename, data):
    with open(filename, 'w') as f:
        yaml.dump(data, f)

# ---------- 启动主函数 ----------
def set_goals(yaml_file):
    data = load_yaml(yaml_file)

    class Goal:
        def __init__(self,x,y,z):
            self.x = x
            self.y = y
            self.z = z
        
        # ---------- 按下按钮的事件函数 ----------
        def change_goal(self):
            data["dx_shot"] = self.x
            data["dy_shot"] = self.y
            data["shot_halt"] = self.z
            save_yaml(yaml_file, data)

    goal1 = Goal(5,5,5)
    goal2 = Goal(5,10,10)
    goal3 = Goal(10,10,15)
    goal4 = Goal(10,5,20)

    root = tk.Tk()
    root.title('目标位置修改')
    root.geometry('230x100')

    # ---------- 生成四个按钮 ----------
    button1 = tk.Button(root, text="(5,5,5)", command = goal1.change_goal())
    button2 = tk.Button(root, text="(5,10,10)", command = goal2.change_goal())
    button3 = tk.Button(root, text="(10,10,15)", command = goal3.change_goal())
    button4 = tk.Button(root, text ="(10,5,20)", command = goal4.change_goal())
    
    button1.grid(row=1, column=1, padx=10, pady=5)
    button2.grid(row=1, column=10, padx=10, pady=5)
    button3.grid(row=10, column=10, padx=10, pady=5)
    button4.grid(row=10, column=1, padx=10, pady=5)


    root.mainloop()

# ---------- 启动 ----------
set_goals(filename)