import os
import yaml

config_file_path = "/home/linhao/ardupilot_ws/src/ros_gz_sim_ardupilot/config/iris_runway_bridge.yaml"

with open(config_file_path, 'r') as file:
    try:
        config = yaml.safe_load(file)
        print(config)
    except yaml.YAMLError as exc:
        print(f"Error parsing YAML file: {exc}")