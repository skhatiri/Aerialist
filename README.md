# AERIALIST - The UAV Test Bench

AERIALIST (Autonomous aERIAL vehIcle teST-bench) is a modular and extensible test bench for UAV software.
Below figure demonstrates the overall architecture of AERIALIST. The implementation currently supports [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), but can be extended to support other UAV platforms as well.
The input is the test case description, which can be in a config file, set in specific environment variables, or provided directly to the Command Line Interface (CLI) as parameters.

AERIALIST prepares the environment for running the test in the Test Runner subsystem, which abstracts any dependencies to the actual UAV, its software platform, specific flight modes and simulation environment. After setting up the simulation environment as described in the test description (if testing a simulated UAV), Test Runner connects to the drone (simulated or physical) and configures it as instructed at startup, and stats sending runtime commands. It also monitors UAV state during the flight, and extracts the flight logs at the end of the test for future analysis.

![Aerialist Architecture](docs/architecture.png)

## Test Runner

To evaluate a test definition, we generate and execute the corresponding simulated test case automatically. The test case automates all necessary steps: setting up the test environment, building/running the firmware code, running/configuring the simulator with the simulated world properties, connecting the simulated UAV to the firmware, and applying the UAV configurations from the test case properties at startup.
Then, the test case commands are scheduled and sent to the UAV, the flight is monitored for any issues, and after test completion, the flight log file is extracted.
Test Description. The de-facto testing standard of UAVs relies on manually-written system-level tests to test UAVs in the field. These tests are defined as specific software configurations (using parameters, config files, etc.), in a specific environment setup (e.g., obstacles placement, lightning conditions), and a set of runtime commands. Such runtime commands received during the UAV flight (from RC, GCS, onboard computers, etc.), make the UAV fly with a specific human observable behavior (e.g., flight trajectory, speed, distance to obstacles). We model a UAV test case as a set of test properties (e.g., ⟨configuration, environment, commands⟩) that control the flight and a set of pre-defined UAV expected states (e.g., ⟨trajectory positions⟩) during the flight. More specifically we define a test case by the following properties, which are fed to AERIALIST as an input file, or command arguments, and refer to them as test description:

- Configs: Drone configuration at startup (all parameter values and configuration files required to start the simulation).
- Commands: Timestamped external commands from the ground station or the remote controller to the drone during the flight (e.g., change flight mode, go in a specific direction, enter mission mode).
- Environment (optional): Simulated world’s configurations (e.g., used simulator, obstacles’ position and shape, wind speed and direction).
- Expectation (optional): time series of certain sensor reading that the test flights are expected to follow closely.

### Generator

The Generator module deals with setting up the simulated world before testing UAVs in SIL mode.
It sets up and prepares the simulation environment as described in the test description, in a specific simulator (e.g., Gazebo, jMAVSim), along with the described static and dynamic objects and simulated UAV.
Configurator. module is responsible for setting up and initialising the UAV under test (either simulated or real) before flying the UAV, according to the  instructions in the test description. This includes building the code, connecting to the drone via MAVLink, setting the parameters, uploading any needed resources, etc.

### Commander

module is responsible for all the runtime communications to the UAV, including scheduling and sending the Remote Control (RC) commands (e.g., manual sticks, flight mode changes, arm/disarm), communications from Ground Controll Station (GCS) or the offboard commands coming from a companion computer.

### Monitor

is the module responsible for runtime analysis of UAV state during the flight. Using MAVLink, we are able to subscribe to any messages communicated between PX4 modules, including sensor values. These messages allow monitoring any runtime checks described in the test description to evaluate monitoring functionalities before deploying on the UAV. The tested and finalised monitoring solutions can then be developed
directly in the PX4 firmware as an on board module, or as a ROS module running on the companion computer.

### Analyst

is responsible for any post-flight analysis, mostly based on the extracted flight log. It parses the ULog
files, and extracts any important and relevant data to analyse test result based on the given expectations in the
test description.

## Setup

### Local

The toolkit requires python >= 3.8 and has been tested with PX4 development environment on ubuntu 18.04 and 20.04

1. Setup PX4 development environment. Follow the instrudctions [here](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html)
2. Clone this repository and cd into its root directory
3. `pip3 install -r requiremetns.txt`
4. `sudo apt-get install python3.8-tk`
5. Create a file named .env in the repository's root directory. Then copy and customize contents of [template.env](template.env) into it.

**Note:** Update *PX4_HOME* and *RESULTS_DIR* variables according to your installation.

### Using Docker

You can use the dockerfile to build a Docker image with all the requirements.

1. `docker build . -t aerialist`
2. `docker run -it aerialist bash`

You can now execute all the following commands in the containers bash.

**Note:** Your user should be able to run docker commands without sudo. [check here](https://docs.docker.com/engine/install/linux-postinstall/)
**Note:** The .env for the docker image come from [template.env](template.env). You can customize them using environment variables for the Docker container.

### Using Pip

1. `pip3 install git+https://github.com/skhatiri/Aerialist.git`

## command-line interface

You can utilize the toolkit with following command line options:

**Note:** Before running any command, make sure you are at the root directory of the repository:

`cd Aerialist/`

You can use `python3 aerialist --help` option anywhere to get help on the command parameters.

- Replaying a pre-recorder PX4 log:

`python3 aerialist --log data/t0.ulg`

- Running an existing series of commands stored in a csv file. Look [here](data/t0_commands.csv) for an example (corresponding to previous .ulg file).

`python3 aerialist --log resources/logs/t0_commands.csv`

- Replaying a pre-recorder PX4 log with collission prevention enabled:

`python3 aerialist --simulator ros --drone ros --log data/ta0.ulg`

<!-- - running a manual flight through keyboard commands:

`python3 run.py experiment manual`

Look [here](https://github.com/skhatiri/drone-experiments/blob/5b7950dc99318d08dacab61ea8686c6d65402438/px4/drone.py#L76) for possible commands to send -->
