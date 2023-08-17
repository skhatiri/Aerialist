import subprocess

with open("/src/aerialist/aerialist/command.txt", 'r') as file:
    cmd = file.read()
    print(cmd)
    command1 = "source /root/.bashrc && cd /src/aerialist" + " && " + cmd
    print(command1)
    exec = subprocess.Popen(command1, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = exec.communicate()

    # Display the output
    print("Output of Commands:")
    print(stdout.decode())
    print(stderr.decode())
