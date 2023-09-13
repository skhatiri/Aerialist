# AERIALIST: UAV Test Bench

<!-- ## [Demo Video](https://youtu.be/pmBspS2EiGg) -->

**Aerialist** (unmanned AERIAL vehIcle teST bench) is a novel test bench for UAV software that automates all the necessary UAV testing steps: setting up the test environment, building and running the UAV firmware code, configuring the simulator with the simulated world properties, connecting the simulated UAV to the firmware and applying proper UAV configurations at startup, scheduling and executing runtime commands, monitoring the UAV at runtime for any issues, and extracting the flight log file after the test completion.

With Aerialist, we aim to provide researchers with an easy platform to automate the running of tests on both simulated and real UAVs, which allows them to do experiments required to overcome the UAV simulation-based testing challenges.

## Table of Contents

- [Introduction](#introduction)
  - [UAV Tests](#uav-tests)
- [Getting Started](#setup)
  - [Local Test Execution](#local-test-execution)
  - [Docker Test Execution](#docker-test-execution)
  - [Kubernetes Test Execution](#kubernetes-test-execution)
- [Usage](#usage)
  - [Test Description File](#test-description-file)
  - [Command Line Interface](#command-line-interface)
  - [Python API](#using-aerialist-in-your-code)
- [References](#references)
- [Contributing](#contributing)
- [License](#license)

## Introduction
<!-- **Aerialist** (unmanned AERIAL vehIcle teST bench) is a modular and extensible test bench for UAV software and it aims to facilitate and automate all the necessary steps of definition, generation, execution, and analysis of system-level test cases for UAVs. -->
Below figure demonstrates Aerialist's software architecture, with the current implementation supporting UAVs powered by [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (a widely used open-source UAV firmware).

The input is a [**Test Description**](#test-description-file) file which defines the UAV and environment configurations and the test steps.
The **Test Runner** subsystem, which abstracts any dependencies to the actual UAV, its software platform, and the simulation environment prepares the environment for running the test case as described in the test description.  
After setting up the simulation environment (if testing a simulated UAV), Test Runner connects to the (simulated or physical) UAV and configures it according to the startup instructions. Then, it sends runtime commands, monitors the UAV's state during the flight, and extracts flight logs at the end of the test for future analysis. Each module is detailed in the [Architecture Documents](docs/architecture.md).

![Aerialist Architecture](docs/architecture.png)

### UAV Tests

The de-facto testing standard of UAVs relies on *manually-written system-level tests* to test UAVs *in the field*.
These tests are defined as **software configurations** (using parameters, config files, etc.), in a specific **environment** setup (e.g., obstacles placement, lighting conditions), and a set of runtime **commands**.
The runtime commands received during the UAV flight (e.g, from a remote controller) make the UAV fly with a specific human observable **behavior** (e.g., trajectory, speed, distance to obstacles).

Hence, Aerialist models a UAV test case with the following set of *test properties* and uses a *yaml* structure to describe the test.

- Drone: Software configurations of the UAV model, including all [Autopilot parameters](https://docs.px4.io/main/en/advanced_config/parameter_reference.html) and configuration files (e.g., mission plan) required to setup the drone for the test.

- Environment: Simulation settings such as the used simulator, physics of the simulated UAV, simulation world (e.g., surface material, UAV’s initial position), surrounding objects (e.g., obstacles size, position), weather condition (e.g., wind, lighting), etc.

- Commands: Timestamped external commands from the ground control station (GCS) or the remote controller (RC) to the UAV during the flight (e.g, change  flight mode, go in a specific direction, enter mission mode).

- Expectation (optional): time series of certain sensor reading that the test flights are expected to follow closely.

## Setup

You can execute UAV test cases with Aerialist in three different ways:

- [Local Test Execution](#local-test-execution): Execute Test Cases using PX4 dependencies installed on the same machine.
This gives you the opportunity to easily develop new functionalities, and run the UAV simulators with the graphical interface, so you can visually follow the UAV behavior.

- [Docker Test Execution](#docker-test-execution): Execute Test Cases in pre-built Docker containers, without the need to install PX4 dependencies on your machine.
This option only supports headless simulation (without the graphical interface).

- [Kubernetes Test Execution](#kubernetes-test-execution): You can also deploy your test execution to a kubernetes cluster for more scale.

### Local Test Execution

**Note:** Installing [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), [PX4-Avoidance](https://github.com/PX4/PX4-Avoidance) and their requirements including ROS and Gazebo could be problematic in many cases. We only suggest this full installation to the users who needs direct visual access to the simulator or are curious to visually monitor the UAVs during the flight. Othervise, test execution, extracting the flight logs and ploting them can be achieved by the [docker exection](#docker-test-execution) as well.

- We have prepared a ready-to-use virtual machine ([download link](https://zhaw-my.sharepoint.com/:f:/g/personal/mazr_zhaw_ch/EnxLqlyju6RMhUYV_SXTqBEBfxundq_-X67eRQAwCPjHvg?e=9953JZ)) to help users onboaord fast. You can move on to using [dockerized test executions](#docker-test-execution) if you don'n need the simulation visualizations anymore.

If you prefer to run the simulations and PX4 on your own machine, follow [PX4 instalation guide](./docs/PX4_installation.md).

- Requirements:
  - Ubuntu 18

1. We have prepared a [bashsript](./setup_script/full_setup.sh) to automate all the steps of installing PX4 and Aerialist in one shot. [Follow the instructions](./docs/PX4_installation.md#instalation-using-bash-script).

- You can also follow a [step by step guide](./docs/PX4_installation.md#step-by-step-instlation) if needed.

2. Clone this repository and cd into its root directory
3. `pip3 install -r requiremetns.txt`
4. Create a file named `.env` in the repository's root directory. Then copy and customize the contents of [`template.env`](template.env) into it.

- Update *PX4_HOME* and *RESULTS_DIR* variables according to your installation.

5. You can now use the [Command Line Interface](#command-line-interface).

### Docker Test Execution

- Requirements: [Docker](https://docs.docker.com/engine/install/)

#### Using Docker Container's CLI

1. `docker run -it skhatiri/aerialist bash`

- You can now use the [Command Line Interface](#command-line-interface) in the container's bash.
- check `python3 aerialist exec --help`

**Note:** The .env for the docker image come from [template.env](template.env). You can customize them using [environment variables](https://docs.docker.com/engine/reference/commandline/run/#env) for the Docker container.

#### Using Host's CLI

Alternatively, you can use the CLI on your local machine and instruct Aerialist to execute test cases inside a docker container.

- Requirements:
  - Your user should be able to run docker commands without sudo. [check here](https://docs.docker.com/engine/install/linux-postinstall/)

1. Clone this repository and cd into its root directory
2. `pip3 install -r requiremetns.txt`
3. Create a file named `.env` in the repository's root directory. Then copy and customize the contents of [`template.env`](template.env) into it.
4. You can use the dockerfile to build a Docker image with all the requirements, or instead pull the latest image from the Image repository.

- `docker build . -t skhatiri/aerialist`
- or `docker pull skhatiri/aerialist`

5. you can now instruct Aerialist to execute test cases inside a docker container

- Just add `--docker` to the command line or update the test-description file (`agent.engine:docker`).
- You can now use the [Command Line Interface](#command-line-interface) in your local bash.
- check `python3 aerialist exec --help`
 
### Kubernetes Test Execution

Aerialist can also depoy test executions on a Kubernetes cluster to facilitate running tests in the cloud. Speciffically, as can be seen in the below figure, Aerialist can run multiple executions of the same test case in isolated Kubernets pods in parallel, and gather test results for further processing.

This feature is specifically helpfull when performing test generation tasks, where UAV's flight could be subject to non-determinism and multiple simulations are required.

<img src="docs/deployment.png" alt="Kubernetes Deployment" width="60%" style="display:block; margin:auto;"/>
<!-- ![Kubernetes Deployment](docs/deployment.png) -->

Aerialist can conect both to a cloud Kubernetes cluster, or a local instance (more useful during development).

- Requirements:
  - [kubectl](https://kubernetes.io/docs/tasks/tools/#kubectl)
    - [Set default context and namespace](https://kubernetes.io/docs/reference/kubectl/cheatsheet/#kubectl-context-and-configuration) to your prefered clster and namespace if needed
  - [yq](https://github.com/mikefarah/yq#install)

      ```bash
      wget https://github.com/mikefarah/yq/releases/download/v4.22.1/yq_linux_amd64 -O /usr/bin/yq &&\
          chmod +x /usr/bin/yq
      ```

#### Using Local Kubernetes Instance

**TODO**

#### Using Cloud Kubernetes Cluster

Aerialist uses a [NextCloud](https://nextcloud.com/) instance to share files between the main container, and the parallel test executers. You can get a free account in [a cloud provider](https://nextcloud.com/sign-up/) or deploy your own [dockerized instance](https://hub.docker.com/_/nextcloud).

1. Set your NextCloud credentials and address in as a k8s-Secret: `kubectl create secret generic webdav --from-literal=host=https://[your-nextcloud-address]/remote.php/dav/files/[your-account-id]/ --from-literal=user=[username] --from-literal=pass=[password]`

2. Upload your [`k8s-config.yaml`](https://kubernetes.io/docs/concepts/configuration/organize-cluster-access-kubeconfig/) as a k8s-ConfigMap: `kubectl create configmap k8s-config --from-file k8s-config.yaml`

3. You can now use `--k8s` in the commands to run the simulations in your k8s-cluster.
`python3 aerialist exec --id case0-manual --obstacle 1 1 1 -8.1 3.1 0 0 --path webdav:// --mission samples/flights/mission1.plan --log samples/flights/mission1.ulg --params samples/flights/mission1-params.csv --commands samples/flights/mission1-commands.csv --drone ros --simulator ros --agent k8s -n 5`

## Usage

### Test Description File

Using a predefined [test-description yaml file](samples/tests/template-test.yaml) is the easiest way to define the test case.

```yaml
# template-test.yaml
drone:
  port: sitl # type of the drone to conect to {sitl, ros, cf}
  #params: #PX4 parameters : https://docs.px4.io/main/en/advanced_config/parameter_reference.html
    # {parameter_name}: {parameter_value} #(keep datatype -> e.g, 1.0 for float, 1 for int)
    # CP_DIST: 1.0
    # POS_MOD: 2.5
  params_file: samples/flights/mission1-params.csv #csv file with the same structure as above 
  mission_file: samples/flights/mission1.plan # input mission file address

simulation:
  simulator: ros # the simulator environment to run {gazebo,jmavsim,ros} 
  speed: 1 # the simulator speed relative to real time
  headless: true # whether to run the simulator headless
  obstacles:
  - size: # Object 1 size in l,w,h
      l: 5
      w: 5
      h: 5
    position: # Object 1 position in x,y,z and it's rotation
      x: 10
      y: 5
      z: 0
      angle: 0
  - size: # Object 2 size in l,w,h
      l: 5
      w: 5
      h: 5
    position:  # Object 2 position in x,y,z and it's rotation
      x: -20
      y: 50
      z: 4
      angle: 0
  # home_position: # home position to place the drone [lat,lon,alt]  
test:
  commands_file: samples/flights/mission1-commands.csv # runtime commands file address
  speed: 1 # the commands speed relative to real time

assertion:
  log_file: samples/flights/mission1.ulg # reference log file address
  # variable: trajectory # reference variables to compare 

agent:
  engine: k8s # where to run the tests {k8s, docker, local}
  count: 5 # no. of parallel runs (only for k8s)
  path: webdav://test/ # cloud output path to copy logs (only for k8s)
  id: yaml-test # k8s job id (only for k8s)

```

### Command-Line Interface

#### Test Execution

You can utilize the toolkit with following command line options:

**Note:** Before running any command, make sure you are at the root directory of the repository:

`cd Aerialist/`

The simplest way to execute a UAV test by Aerialist is by following command:
`python3 aerialsit exect --test [test-file.yaml]`

More sample tests can be found [here](samples/tests/)

You can use `python3 aerialist exec --help` anywhere to get help on the command parameters.

#### Other CLI Arguments

Alternatively, the above test properties can also be set using commandline aruguments as below.

|argument   | input type            | description                   |
|-----------|-----------------------|------------------------------ |
| --test    | path/to/file.yml      | test description yaml file    |
| --drone   | {**sitl**, ros, cf}   | type of the drone to conect to|
| --mission | path/to/file.plan     | input mission file address    |
| --params  | path/to/file.csv      | params file address           |
| --simulator| {**gazebo**,jmavsim,ros} | the simulator environment to run|
| --obstacle| float [l,w,h,x,y,z,r] | obstacle size, position, and angle to put in simulation environment|
| --obstacle2| float [l,w,h,x,y,z,r]| obstacle size, position, and angle to put in simulation environment|  
| --headless| _                     | whether to run the simulator headless|
| --speed   | float = 1.0           | the simulator speed relative to real time|
| --home    | float [lat,lon,alt]   | home position to place the drone|
| --commands| path/to/file.{ulg,csv}| runtime commands file address |
| --log     | path/to/file.ulg      | reference log file address    |
| --agent   |{local,docker,k8s}     | where to run the tests        |
| -n        | int = 1               | no. of parallel runs          |
| --path    | path/to/folder/       | cloud output path to copy logs|
| --id      | string                | k8s job id                    |

Some of the common combination of the following arguments are listed here as sample test cases:

- Replaying a pre-recorded manual flight log:

`python3 aerialist exec --commands samples/flights/manual1.ulg --simulator gazebo --drone sitl`

- Running an existing series of manual commands stored in a csv file. Look [here](samples/flights/manual1-commands.csv) for an example (corresponding to previous .ulg file).

`python3 aerialist exec --commands samples/flights/manual1-commands.csv --simulator gazebo --drone sitl`

- Replaying a pre-recorded manual flight log with collission prevention enabled:

`python3 aerialist exec --commands samples/flights/manual2.ulg --simulator ros --drone ros`

- Executing a pre-planed autonomous flight log with obstacle avoidance enabled:

`python3 aerialist exec --commands samples/flights/mission1-commands.csv --mission samples/flights/mission1.plan --params samples/flights/mission1-params.csv --log samples/flights/mission1.ulg --simulator ros --drone ros`

<!-- - running a manual flight through keyboard commands:

`python3 run.py experiment manual`

Look [here](https://github.com/skhatiri/drone-experiments/blob/5b7950dc99318d08dacab61ea8686c6d65402438/px4/drone.py#L76) for possible commands to send -->

#### Plotting Executed Test Cases

`python3 aerialist plot --test [test-file.yaml] --log [test-log.ulg]`

### Using Aerialist in Your Code

You can integrate Aerialist's python package in your own code and directly define and execute UAV test cases with it.
This can be speccifically suefull when you are working on test generation approaches for UAVs. An example of such usage of Aerialist can be found in [Surrealist](https://github.com/skhatiri/Surrealist).

1. `pip3 install git+https://github.com/skhatiri/Aerialist.git`
2. Create an instance of [DroneTest](./aerialist/px4/drone_test.py) class and define your test case
3. configure and execute the test case using your prefered test execution agent ([LocalAgent](./aerialist/px4/local_agent.py), [DockerAgent](./aerialist/px4/docker_agent.py), [K8sAgent](./aerialist/px4/k8s_agent.py))

- Take a look at the [CLI code](./aerialist/entry.py) for more insights.

## References

If you use this tool in your research, please cite the following papers:

- **Sajad Khatiri**, Sebastiano Panichella, and Paolo Tonella, "Simulation-based Test Case Generation for Unmanned Aerial Vehicles in the Neighborhood of Real Flights," *In 2023 IEEE 16th International Conference on Software Testing, Verification and Validation (ICST)*
  - [Preprint](https://skhatiri.ir/papers/surrealist.pdf)
  - [Experiments Dataset](https://doi.org/10.5281/zenodo.6525021)

````{code-block} bibtex
@inproceedings{khatiri2023simulation,
  title={Simulation-based test case generation for unmanned aerial vehicles in the neighborhood of real flights},
  author={Khatiri, Sajad and Panichella, Sebastiano and Tonella, Paolo},
  booktitle={2023 16th IEEE International Conference on Software Testing, Verification and Validation (ICST)},
  year={2023},
}
````

## License

The software we developed is distributed under MIT license. See the [license](./LICENSE.md) file.

## Contacts

- Sajad Khatiri
  - Zurich University of Applied Science (ZHAW), Switzerland - <mazr@zhaw.ch>

## Contributing

TODO
