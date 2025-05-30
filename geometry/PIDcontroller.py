import tkinter as tk 
import os
import yaml

filename = '/home/limile/dxy_apm_ws/src/px4_ros_com/config/pos_config.yaml'
filename_goal = '/home/limile/dxy_apm_ws/src/px4_ros_com/config/OffboardControl.yaml'

# ---------- 读取 YAML ----------
def load_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)

# ---------- 保存 YAML ----------
def save_yaml(filename, data):
    with open(filename, 'w') as f:
        yaml.dump(data, f)


# ---------- 主函数 ----------
def edit_mypid_fields(yaml_file,yaml_file_g):
    data = load_yaml(yaml_file)

    # 确保结构存在
    if "mypid" not in data or not isinstance(data["mypid"], dict):
        print("错误：YAML 文件中未找到 'mypid' 字段或格式错误。")
        return

    pid_data = data["mypid"]
    editable_keys = ["p", "i", "d","outputlimit","integrallimit"]
    
    root = tk.Tk()
    root.title("PID 参数调整器")
    root.geometry("620x300")

    def newwind():
        winNew = tk.Toplevel(root)
        winNew.geometry('230x100')
        winNew.title('修改目标位置')

        data_g = load_yaml(yaml_file_g)

        class Goal:
            
            def __init__(self,x,y,z):
                self.x = x
                self.y = y
                self.z = z
        
        # ---------- 按下按钮的事件函数 ----------
            def change_goal(self):
                data_g["dx_shot"] = self.x
                data_g["dy_shot"] = self.y
                data_g["shot_halt"] = self.z
                save_yaml(yaml_file_g, data_g)

        goal1 = Goal(5,5,5)
        goal2 = Goal(5,10,10)
        goal3 = Goal(10,10,15)
        goal4 = Goal(10,5,20)

    # ---------- 生成四个按钮 ----------
        button1 = tk.Button(winNew, text="(5,5,5)", command=goal1.change_goal)
        button2 = tk.Button(winNew, text="(5,10,10)", command=goal2.change_goal)
        button3 = tk.Button(winNew, text="(10,10,15)", command=goal3.change_goal)
        button4 = tk.Button(winNew, text="(10,5,20)", command=goal4.change_goal)
    
        button1.grid(row=1, column=1, padx=10, pady=5)
        button2.grid(row=1, column=10, padx=10, pady=5)
        button3.grid(row=10, column=10, padx=10, pady=5)
        button4.grid(row=10, column=1, padx=10, pady=5)

    mainmenu = tk.Menu(root)
    menuFile = tk.Menu(mainmenu)
    mainmenu.add_cascade(label='菜单',menu=menuFile)
    menuFile.add_command(label='修改目标位置',command=newwind)
    menuFile.add_separator()
    menuFile.add_command(label='退出',command=root.destroy)
 
    root.config(menu=mainmenu)

    slider_vars = {}
    for i, key in enumerate(editable_keys):
        if key in pid_data:
            tk.Label(root, text=key.upper()).grid(row=i, column=0, padx=10, pady=5)
            if key == "outputlimit" or key == "integrallimit":
                var = tk.DoubleVar(value=pid_data[key])
                slider = tk.Scale(
                    root,
                    from_=0.0,
                    to=3.0,
                    resolution=0.00001,
                    orient='horizontal',
                    variable=var,
                    length=400
                    )
                slider.grid(row=i, column=1, padx=10, pady=5)
                slider_vars[key] = var
            else:
                var = tk.DoubleVar(value=pid_data[key])
                slider = tk.Scale(
                    root,
                    from_=0.0,
                    to=1.0,
                    resolution=0.00001,
                    orient='horizontal',
                    variable=var,
                    length=400
                )
                slider.grid(row=i, column=1, padx=10, pady=5)
                slider_vars[key] = var
            
        else:
            print(f"警告：mypid 中缺少 '{key}' 字段")

    def save_changes():
        for key, var in slider_vars.items():
            pid_data[key] = round(var.get(), 4)
        save_yaml(yaml_file, data)
        print("已保存修改：", pid_data)

    tk.Button(root, text="保存修改", command=save_changes).grid(row=8, column=0, columnspan=2, pady=10)
    root.mainloop()

# ---------- 启动 ----------
edit_mypid_fields(filename,filename_goal)

