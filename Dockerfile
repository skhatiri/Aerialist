### Using the base image containing ROS Melodic and PX4 development environment 
FROM skhatiri/px4

RUN sudo apt-get update -y &&\
    sudo apt-get install python3.8 python3-setuptools python3.8-dev -y &&\
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1 &&\
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2 &&\
    sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 3 
RUN python3 -m pip install --upgrade pip

#Setting up the current tool 
COPY ./requirements.txt /src/aerialist/requirements.txt
WORKDIR /src/aerialist/
RUN pip3 install -r requirements.txt
RUN pip3 install --upgrade anyio>4.0.0
COPY . .
RUN chmod +x ./aerialist/__main__.py
COPY ./template.env ./.env
RUN mkdir -p /io/ ./results/logs/ tmp/




