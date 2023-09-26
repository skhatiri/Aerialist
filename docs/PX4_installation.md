# PX4 Installation

Installing [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), [PX4-Avoidance](https://github.com/PX4/PX4-Avoidance) and their requirements including ROS and Gazebo could be problematic in many cases. We only suggest this full installation to the users who needs direct visual access to the simulator or are curious to visually monitor the UAVs during the flight. Othervise, test execution, extractin the flight logs and ploting them can be achieved by the [docker exection](../README.md#dockerized-test-execution) as well.

## Virtual Machine

We have prepared a ready-to-use virtual machine ([download link](https://zhaw-my.sharepoint.com/:f:/g/personal/mazr_zhaw_ch/EnxLqlyju6RMhUYV_SXTqBEBfxundq_-X67eRQAwCPjHvg?e=9953JZ)) to help users onboaord fast. You can move on to using [dockerized test executions](../README.md#dockerized-test-execution) if you don'n need the simulation visualizations anymore.

- Use the VMWare Workstation Player(free version) to import the VM in Windows and Ubuntu and VMWare Fusion(free trial version) to import the VM in MacOS.
- Make sure that "Accelarate 3D Graphic" under Edit Virtual Machine settings --> Hardware --> Display is unchecked. It might cause Gazebo simulator to crash if selected.

- **Requirements:** Ubuntu 18

## Installation Using bash script

We have prepared a [bash script](../setup_script/px4_avoidance_aerialist_setup.sh) to automate all the steps of installing PX4 and Aerialist in one shot. This has only been tested in a fresh Ubuntu 18.04, so may not fully work for your specific configurations.

We recommend you to use a fresh installed Ubuntu environment to set up Aerialist in order to avoid dependency issues.
The script was tested in a virtual machine in VMWare.
You can use VMWare Workstation Player(free version) to set up the clean Ubuntu environment..

### Full Installation

To install PX4, PX4 avoidance along with Aerialist in one shot, download all these three script [px4_avoidance](../setup_script/px4_avoidance_setup.sh), [aerialist](../setup_script/aerialist_setup.sh) and [installation script](../setup_script/px4_avoidance_aerialist_setup.sh) in the same folder and run below command in the terminal:

```
chmod +x px4_avoidance_aerialist_setup.sh
./px4_avoidance_aerialist_setup.sh 
```

Now the installation will continue for around half an hour to one hour depending on your Internet speed.

Once the script has completed successfully, you can close the terminal and open a new terminal.
In the new terminal, you can see the conda base environment has been now activated.

You can switch to Aerialist environment by typing below command:

```
conda activate Aerialist
```

Now navigate into Aerialist directory which is located in your home directory by entering below command in the terminal.

```
cd ~/Aerialist
```

### Separate Installation:
We also provide a separate [script]((../setup_script/px4_avoidance_setup.sh)) to install only PX4 and PX4 avoidance.

Download this script and run below command in the terminal to install:

```
chmod +x px4_avoidance_setup.sh
./px4_avoidance_setup.sh
```

If you also want to install Aerialist, you can download and run [aerialist_setup.sh](../setup_script/aerialist_setup.sh) using below command:

```
chmod +x aerialist_setup.sh
./aerialist_setup.sh
```


## Step-by-Step Installation

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