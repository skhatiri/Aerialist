# Using Aerialist in Your Code

You can integrate Aerialist's python package in your own code and directly define and execute UAV test cases with it.
This can be speccifically useful when you are working on test generation approaches for UAVs. An example of such usage of Aerialist can be found in [Surrealist](https://github.com/skhatiri/Surrealist).

1. `pip3 install git+https://github.com/skhatiri/Aerialist.git`
2. Create an instance of [DroneTest](./aerialist/px4/drone_test.py) class and define your test case
3. configure and execute the test case using your prefered test execution agent ([LocalAgent](./aerialist/px4/local_agent.py), [DockerAgent](./aerialist/px4/docker_agent.py), [K8sAgent](./aerialist/px4/k8s_agent.py))

- Take a look at our [code snippets](./samples/snippets/) for more details and sample codes.
