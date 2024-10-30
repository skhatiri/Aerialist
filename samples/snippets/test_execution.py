from aerialist.px4.drone_test import DroneTest
from aerialist.px4.docker_agent import DockerAgent
from aerialist.px4.plot import Plot


### load a yaml test file
test = DroneTest.from_yaml("samples/tests/mission1-local.yaml")

### manipulate test properties
### just a few samples here
# test.simulation.obstacles = [Obstacle()]
# test.agent.engine=test.agent.DOCKER
# test.drone.mission_file = new_mission_file_address
#

### execute the test using the docker agent
agent = DockerAgent(test)
test_results = agent.run()

### plot the test
Plot.plot_test(test, test_results)

### get the test flight trajectory
trajectory = test_results[0].record

### plot the trajecotory
Plot.plot_trajectory([trajectory])

### get minimum drone distance to the obstacles
distance = trajectory.distance_to_obstacles(test.simulation.obstacles)
