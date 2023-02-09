import yaml
from yaml.loader import SafeLoader
def reform_args(args):
    list_obstacle_1 = []
    list_obstacle_2 = []
    list_pattern_1 = []
    list_pattern_2 = []
    with open(args.config[0]) as f:
        data = yaml.load(f, Loader=SafeLoader)

        list_obstacle_1.append(data['obstacle']['position']['x'])
        list_obstacle_1.append(data['obstacle']['position']['y'])
        list_obstacle_1.append(data['obstacle']['position']['z'])
        list_obstacle_1.append(data['obstacle']['position']['a'])
        list_obstacle_1.append(data['obstacle']['position']['b'])
        list_obstacle_1.append(data['obstacle']['position']['c'])
        list_obstacle_1.append(data['obstacle']['position']['d'])

        list_obstacle_2.append(data['obstacle2']['position']['x'])
        list_obstacle_2.append(data['obstacle2']['position']['y'])
        list_obstacle_2.append(data['obstacle2']['position']['z'])
        list_obstacle_2.append(data['obstacle2']['position']['a'])
        list_obstacle_2.append(data['obstacle2']['position']['b'])
        list_obstacle_2.append(data['obstacle2']['position']['c'])
        list_obstacle_2.append(data['obstacle2']['position']['d'])

        list_pattern_1.append(str(data['pattern']).lower())
        list_pattern_2.append(str(data['pattern2']).lower())

        args.obstacle = list_obstacle_1
        args.obstacle2 = list_obstacle_2
        args.pattern = list_pattern_1
        args.pattern2 = list_pattern_2
        covnert_config(args)
    return args

def covnert_config(args):
    with open(args.config[0]) as f:
        data = yaml.load(f, Loader=SafeLoader)
    with open("/home/prasun/catkin_ws/src/avoidance/image_proc/src/nodelets/profile.txt", 'w') as f:
        if(data['noise']['type'] == "Gaussian"):
            f.write("gaussianNoise:" + str(data['noise']['active']) + "\n")
            f.write("gaussianMean:" + str(data["noise"]["noise_mean"]) + "\n")
            f.write("gaussianStdDeviation:" + str(data["noise"]["noise_std_deviation"]) + "\n")
        # gaussianNoise: True
        # gaussianMean: 20
        # gaussianStdDeviation: 20
