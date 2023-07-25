import subprocess
import time
# Define the terminal command and its parameters
time.sleep(5)
print("Running this command")
command = 'bash -c "source /home/prasun/catkin_ws/devel/setup.bash; rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image \'m.image\'"'

# Initialize the subprocess.Popen object
process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Wait for the process to complete and fetch the output and errors
output, errors = process.communicate()

# Print the output and errors
print("Output:", output.decode())
print("Errors:", errors.decode())
