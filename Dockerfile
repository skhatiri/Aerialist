FROM skhatiri/px4

RUN sudo apt-get update -y &&\
    sudo apt-get install python3.8 python3-setuptools python3.8-dev -y &&\
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1 &&\
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2 &&\
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 3 
RUN python3 -m pip install --upgrade pip
RUN echo "export ROS_PACKAGE_PATH=/src/aerialist/aerialist/inter_images:\$ROS_PACKAGE_PATH" >> /root/.bashrc

RUN sudo apt install ros-melodic-stereo-image-proc ros-melodic-image-view -y
# RUN sudo apt-get install python-rospy
#&&\
# sudo -H pip3 install --upgrade pip

#Setting up the current tool 
COPY ./requirements.txt /src/aerialist/requirements.txt
WORKDIR /src/catkin_ws/src/avoidance/
RUN catkin_create_pkg intermediate_image_save std_msgs rospy roscpp cv_bridge
WORKDIR /src/catkin_ws/src/avoidance/intermediate_image_save/src
RUN mkdir -p nodes
WORKDIR /src/catkin_ws/src/avoidance/intermediate_image_save/src/nodes
COPY aerialist/intermediate_image_script/scripts/ .
RUN catkin build -w /src/catkin_ws &&\
    echo "source /src/catkin_ws/devel/setup.bash" >> ~/.bashrc  
WORKDIR /src/aerialist/
RUN pip3 install -r requirements.txt
RUN pip2 install roslibpy
RUN pip3 install roslibpy
COPY . .
RUN chmod +x ./aerialist/__main__.py
COPY ./template.env ./.env
RUN mkdir -p /io/ ./results/logs/

RUN cd /home &&\
    git clone https://github.com/saurabhprasun20/gazebo.git &&\
    mkdir -p .gazebo &&\
    cp -r * /home/gazebo/* /home/.gazebo
# RUN catkin_create_pkg /catkin_ws/src/aerialist/src intermediate_image_save std_msgs rospy roscpp cv_bridge

# ENTRYPOINT [ "./run.py" ]
# CMD [ "--help" ]




