# AERIALIST (UAV Test Bench)

The reposotory contains tools to do experiments on drone platforms,
specifically it supports [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) running on both actual drones (currently Crazyflie 2.1 tested) and in SITL mode leveraging simulators (currently Gazebo and JMavSim tested)

The package simplifies the conection to drones, as well as sending commands, extracting data from logs, replaying recorded flights from logs, etc., using the [Experiment](px4/experiment.py) interface.

## Setup

The toolkit requires python > 3.7 and has been tested with PX4 development environment on ubuntu 18.04 and 20.04

1. Setup PX4 development environment. Follow the instrudctions [here](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html)
2. Clone this repository and cd into its root directory
3. `pip3 install -r requiremetns.txt`
4. `sudo apt-get install python3.7-tk`
5. Create a file named .env in the repository's root directory. Then copy and customize contents of [template.env](template.env) into it.

**Note:** Update *PX4_HOME* and *RESULTS_DIR* variables according to your installation.

## command-line interface

You can utilize the toolkit with following command line options:

**Note:** Before running any command, make sure you are at the root directory of the repository:

`cd aerialist/`

You can use `--help` option anywhere to get help on the command parameters.

- Replaying a pre-recorder PX4 log:

`python3 run.py --log resources/logs/t0.ulg experiment replay`

- Running an existing series of commands stored in a csv file. Look [here](resources/logs/t0_commands.csv) for an example (corresponding to previous .ulg file).

`python3 run.py --log resources/logs/t0_commands.csv experiment replay`

- Replaying a pre-recorder PX4 log with collission prevention enabled:

`python3 run.py --env avoidance --drone ros --log resources/logs/ta0.ulg experiment replay`

- running a manual flight through keyboard commands:

`python3 run.py experiment manual`

Look [here](https://github.com/skhatiri/drone-experiments/blob/5b7950dc99318d08dacab61ea8686c6d65402438/px4/drone.py#L76) for possible commands to send

**Note:** remaining commands are still actively on development and not stable.

- Search for best command serie to reconstruct a flight scenario:

`python3 run.py --log resources/logs/t0.ulg search --rounds 10`

## Using Docker

You can use the dockerfile to build a Docker image with all the requirements.

1. `docker build . -t aerialist`
2. `docker run -it aerialist bash`

You can now execute all the commands in the containers bash.

**Note:** Your user should be able to run docker commands without sudo. [check here](https://docs.docker.com/engine/install/linux-postinstall/)
**Note:** The .env for the docker image come from [template.env](template.env). You can customize them using environment variables for the Docker container.
