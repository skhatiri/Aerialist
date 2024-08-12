# Q&A

### 1) I am not sure if I get what you meant by Hard Fail? It is mentioned as if there is a collision with any of the obstacles in the environment. How can we check for that? What is the output of the simulator in this case? Will simulation continue after crashing to an obstacle and what is the output or min distance in this case?
Regarding the crashing scenarios, in most cases, the simulation will continue as usual and you can detect the collision using the distance.
The trick is that the obstacles have a collision-free border, so even if the drone hits them physically, most probably it will not have any physical damage that makes the simulation end with error.
Of course, there could be some exceptions and some random behavior could happen, but I don't have a systematic way to detect them yet.
you can simply ignore such behavior
you will get a very low distance (<0.3 or so -because of the size of the drone itself- or simply 0) if the drone crashes into the obstacle.

### 2) Are these test cases with a long time considered invalid or a failure?
Such test cases are normally considered invalid, or simply not challenging -> there is no penalty, nor positive point for generating those. If the drone gets in an unsafe state during the time, it can be also counted as an unsafe scenario.

### 3) We produced an example in which the obstacle is not too tall (h=5) and the UAV flies over the obstacle without any collision. However, the code returns that the minimum distance with the obstacle is 0.
The current implementation of the distance only considers the (x,y) dimensions. The reason is that we expected the obstacles to be tall enough that the drone had no way to go above them.

### 4) We observed that, if we give an invalid test with two overlapping objects, the simulation does not tell us that it is invalid, but it produces a weird path for the UAV. How can we know if a test is invalid?
The simulator behaves non-deterministically when the obstacles are overlapping since they can not be physically placed in the provided position. They may fall on each other, or the drone, or be unstable. These test cases are considered invalid.
The platform does not internally check the validity of the placements, but you need to do this and make sure you are generating valid test cases.
you can check the intersection of the obstacles using this method, or develop your own module for these kinds of checks.

### 5) We produced a test in which the minimum distance is 0, but the UAV reaches the destination. The figure shows that the drone goes through the building.
It seems that probably the drone flies over one of the obstacles, and because of the distance issue mentioned above, you get the 0 distance.

### 6) When I run the random_generator provided as an example generator, it generates a results folder which contains logs and png files for case studies. Do you expect the same output format for the new test generators?
The most important output you need to report is the test case itself, which in Aerialist is represented as a YAML file (similar to the case studies).
Here is the code to save the YAML representation of each test case (which also includes the obstacle properties).
You can also load test case properties from YAML similar to the code here. You will then have access to the obstacle properties (if any).

### 7) During generation, do we have access to the plan file? I.e., can we rely on the knowledge of the origin, destination, and  waypoints?
yes, you have access to the case study (test description yaml file), as well as all the files references in it, including the mission plan, parameters, commands, etc.

### 8) What does the curve with the “original” label represent in the generated results?
if you provide a reference "ulg" file in the 'assertion' property of the test cases, you will have the flight trajectory of the given flight plotted as the "original" flight. This only matters if you wish to visually compare the current flight with a previous one.

### 9) How is the budget computed?
The budget is the total number of simulations you are allowed to do during the test generation.
Each individual simulation executed during the test generation is calculated once. If you run the same test case n times - n simulations - (to check for a nondeterministic behavior), it will be calculated n times.

### 10) Does the UAV have a size (i.e., we need to consider its occupancy) or can we consider it as a point without size?
The UAV used in the simulations is called Iris (https://docs.px4.io/v1.12/en/simulation/gazebo.html). You can find its physical characteristics. The distances calculated by the platform consider it as a point (in the center). You can also consider its size in your test generation process if you prefer, but there is no obligation.

### 11) How do we know if the flight mission is physically feasible? Are there some constraints on the distance between objects? How do we know if a generated test is invalid? Is waiting for the timeout the only way?
Currently, there is no automated way of predicting the feasibility of the test cases. If the simulation takes too long (time out), especially if more than once, you can conclude that the simulation did not finish correctly, and probably the timeout passed.
You can add some simple sanity checks to the obstacle configurations (e.g., having no overlaps)

### 12) Do we need to use the class TestCase in our generator or can we implement our own?
This is only a sample code to help you. You can implement your own class or extend it as you wish, but you should keep using the Aerialist functionalities and built-in classes (specifically DroneTest) for test case properties.

### 13) How is the budget managed? Do we need to set a maximum number of possible simulations inside our code (as in the random generator given as an example)? Or will the pipeline of the competition stop our generator after a maximum number of simulations have been called?
The random generator is a good example of how the interface of your final test generator should look like. You need to take the case study, and the generation budget as inputs, and stop the generation process before running out of budget. The platform will not check the budget automatically, but we will make sure that your code does not go over the budget.

### 14) I still get some results like the image below, in which the drone is passing through the obstacle. While the height of my obstacles are between 15 and 30 meters.
The reason is that you are probably directly changing the Obstacle properties in your code, instead of creating a new Obstacle object. The Obstacle class is not designed to be reusable. i.e., you should not use the same object and directly change its size or position.

Check the below code for clarification:
size = Obstacle.Size(l=10, w=10, h=20)
position = Obstacle.Position(x=10, y=10, z=0, r=0)
obstacle = Obstacle(size, position)

Here, any changes to obstacle.size or obstacle.position will not yield a valid config. Specifically, It has effects on the simulation world (hence, changing the flight trajectory), but not correctly on the plots and distance calculation.
#### Wrong:
obstacle.size=Obstacle.Size(l=15, w=10, h=20) # will yield to problematic plots and distance calculation
or
obstacle.position = Obstacle.Position(x=15, y=10, z=0, r=0) # will yield to problematic plots and distance calculation
or 
obstacle.size.l=15 # will yield to problematic plots and distance calculation
or
obstacle.position.x=15 # will yield to problematic plots and distance calculation

Instead, You should create a new object from the Obstacle class with your updated size and position.

#### Right:
obstacle=Obstacle(Obstacle.Size(l=15, w=10, h=20), Obstacle.Position(x=15, y=10, z=0, r=0))
