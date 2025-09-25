# Extension Guide for Other Usecase

The current implimentation is targeting PX4 usecase. 

Aerialist (and Surrealist) has also been privately extended to generate and execute navigation tests for the **[ANYmal robot](https://www.anybotics.com/robotics/anymal/)**. 


Here is a short guideline for future extensions to other robotic usescase based on the ANYmal experience:  

- Implement usecase specific **Docker Agent** for handling the test execution taking [docker_agent.py](../aerialist/px4/docker_agent.py) as parent. You can optionally extend to use the K8S to scale the test exection taking [k8s_agent.py](../aerialist/px4/k8s_agent.py) as a sample.  
- Implement usecase specific **Trajectory** class to extract the robot trajectory from the simulation logs taking [trajectory.py](../aerialist/px4/trajectory.py) as parent.
- Define potential usecase specific test properties by extending [aerialist_test .py](../aerialist/px4/aerialist_test .py). 
- Search for `# extension_hint:` in the code base to find places needing some small updates. 
- Define usecase specific test scenarios based on the samples in the [sample folder](../samples/). 
- Extend [Surrealist](https://github.com/skhatiri/Surrealist/) to support generating and executing test suites for your robotic usecase. Follow [Surrealist Extension Guide](https://github.com/skhatiri/Surrealist/blob/sync_aerialist/docs/extension.md) for details. 
