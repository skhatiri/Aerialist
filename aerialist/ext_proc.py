import subprocess


def run_proc():
    command = "roslaunch inter_images test.launch"
    sub_proc = subprocess.run(command, shell=True)
    return sub_proc
