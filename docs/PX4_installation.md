# PX4 Installation

Installing [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), [PX4-Avoidance](https://github.com/PX4/PX4-Avoidance) and their requirements including ROS and Gazebo could be problematic in many cases. We only suggest this full installation to the users who needs direct visual access to the simulator or are curious to visually monitor the UAVs during the flight. Othervise, test execution, extractin the flight logs and ploting them can be achieved by the [docker exection](../README.md#dockerized-test-execution) as well.

In order to install PX4 autopilot with PX4 avoidance, you can use the [px4_avoidance_setup.sh](../setup_script/px4_avoidance_setup.sh) script.

You need to make the script executable by running below command in the terminal:

```
chmod +x px4_avoidance_setup.sh
```

Now you can run the script by running below command:

```
./px4_avoidance_setup.sh
```

***Please note that if you install PX4 and Avoidance separately using above command, you will have to run a separate
script to install Aerialist which is described in the [next section](#aerialist-installation). 
However, if you want to install PX4 and Aerialist together, please skip above commands and follow the steps described in [this section](#installation-using-bash-script).***

# Aerialist Installation:

If you have installed PX4 and PX4-Avoidance using above script and want to install Aerialist,
you can use [aerialist_setup.sh](../setup_script/aerialist_setup.sh) to install Aerialist.

Again, make the file executable by using chmod command by typing:

```
chmod +x aerialist_setup.sh
```

and then you can execute the executable script by running below command in the terminal:

```
./aerialist_setup.sh
```

## Virtual Machine

We have prepared a ready-to-use virtual machine ([download link](https://zhaw-my.sharepoint.com/:f:/g/personal/mazr_zhaw_ch/EnxLqlyju6RMhUYV_SXTqBEBfxundq_-X67eRQAwCPjHvg?e=9953JZ)) to help users onboaord fast. You can move on to using [dockerized test executions](../README.md#dockerized-test-execution) if you don'n need the simulation visualizations anymore.

- Use the VMWare Workstation Player(free version) to import the VM in Windows and Ubuntu and VMWare Fusion(free trial version) to import the VM in MacOS.
- Make sure that "Accelarate 3D Graphic" under Edit Virtual Machine settings --> Hardware --> Display is unchecked. It might cause Gazebo simulator to crash if selected.

- **Requirements:** Ubuntu 18

## Installation Using bash script

We have prepared a [bashsript](../setup_script/px4_avoidance+aerialist_setup.sh) to automate all the steps of installing PX4 and Aerialist in one shot. This has only been tested in a fresh Ubuntu 18.04, so may not fully work for your specific configurations.

We recommend you to use a fresh installed Ubuntu environment to set up Aerialist in order to avoid dependency issues.
The script was tested in a virtual machine in VMWare.
You can use VMWare Workstation Player(free version) to setup the clean Ubuntu environment..

Copy [the script](../setup_script/px4_avoidance+aerialist_setup.sh) into your system's home directory and make it executable. To make it executable, open the terminal and navigate to file's location and type below command:

```
chmod +x px4_avoidance+aerialist_setup.sh
```

***Now you can run the executable script by typing below command :***

```
./px4_avoidance+aerialist_setup.sh 
```

***You will be prompted to enter the root password.*** 

After that the installation will continue for at least half an hour to around one hour depending on your Internet speed.

Once the script has completed successfully, you can close the terminal and open the new terminal.
In the new terminal, you can see the conda base environment has been now activate.

***You can switch to Aerialist environment by typing below command:***

```
conda activate Aerialist.
```

Now navigate into Aerialist directory which is located in your home directory by entering below command in the terminal.

```
cd ~/Aerialist
```

***Now you are ready to use and test Aerialist.*** 


## Step by Step Installation

If you can not use the above script, these are step by step guides to install PX4 and its dependencies.

1. [install python >= 3.8](https://www.itsupportwale.com/blog/how-to-upgrade-to-python-3-8-on-ubuntu-18-04-lts/) (if not already installed)

- [if terminal doesn't show up anymore](https://askubuntu.com/questions/1132349/terminal-not-opening-up-after-upgrading-python-to-3-7) (use VS code terminal)

2. [setup PX4 development toolchain](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#ros-melodic-ubuntu-18-04)

3. [clone and run PX4-Autopilot](https://docs.px4.io/main/en/dev_setup/building_px4.html) (skip the steps already done)

- checkout version v1.12.3: `git checkout tags/v1.12.3 --recursive-modules`
- fixing potential errors you may encounter:
  - [Build error GSTREAMER](https://github.com/PX4/PX4-Autopilot/issues/13117)
  - [jMAVsim fails with Java 11](https://github.com/PX4/jMAVSim/issues/96#issuecomment-500788800) and then [this](https://github.com/PX4/PX4-Autopilot/issues/9557#issuecomment-512137607) and [this](https://github.com/PX4/PX4-Autopilot/issues/9557#issuecomment-589559521)

5. [install QGC](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu)

- on ubuntu 18, check [this](https://github.com/mavlink/qgroundcontrol/issues/9847#issuecomment-918133080) if you can not run

6. [setup PX4-Avoidance](https://github.com/PX4/PX4-Avoidance#installation) (start from point 7)

7. check if everything works