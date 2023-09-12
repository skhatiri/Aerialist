# Aerialist Command-Line Interface

## Test Execution

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