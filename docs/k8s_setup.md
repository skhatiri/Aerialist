# Kubernetes Test Execution

Aerialist can also deploy test executions on a Kubernetes cluster to facilitate running tests in the cloud. Specifically, as can be seen in the below figure, Aerialist can run multiple executions of the same test case in isolated Kubernets pods in parallel, and gather test results for further processing.

This feature is specifically helpful when performing test generation tasks, where UAV's flight could be subject to non-determinism and multiple simulations are required.

<p align="center"><img src="deployment.png" alt="Kubernetes Deployment" width="60%"/></p>
<!-- ![Kubernetes Deployment](docs/deployment.png) -->

Aerialist can connect both to a cloud Kubernetes cluster, or a local instance (more useful during development).

- Requirements:
  - [kubectl](https://kubernetes.io/docs/tasks/tools/#kubectl)
    - [Set default context and namespace](https://kubernetes.io/docs/reference/kubectl/cheatsheet/#kubectl-context-and-configuration) to your prefered clster and namespace if needed
  - [yq](https://github.com/mikefarah/yq#install)

      ```bash
      wget https://github.com/mikefarah/yq/releases/download/v4.22.1/yq_linux_amd64 -O /usr/bin/yq &&\
          chmod +x /usr/bin/yq
      ```

## Using Local Kubernetes Instance

**TODO**

## Using Cloud Kubernetes Cluster

Aerialist uses a [NextCloud](https://nextcloud.com/) instance to share files between the main container, and the parallel test executers. You can get a free account in [a cloud provider](https://nextcloud.com/sign-up/) or deploy your own [dockerized instance](https://hub.docker.com/_/nextcloud).

1. Set your NextCloud credentials and address in as a k8s-Secret:
  `kubectl create secret generic webdav --from-literal=host=https://[your-nextcloud-address]/remote.php/dav/files/[your-account-id]/ --from-literal=user=[username] --from-literal=pass=[password]`

2. Upload your [`k8s-config.yaml`](https://kubernetes.io/docs/concepts/configuration/organize-cluster-access-kubeconfig/) as a k8s-ConfigMap:
  `kubectl create configmap k8s-config --from-file k8s-config.yaml`

3. You can now use `--agent k8s` in the commands to run the simulations in your k8s-cluster.
  `python3 aerialist exec --test samples/tests/mission1.yaml --agent k8s -n 5 --id mission-test --path webdav://`
