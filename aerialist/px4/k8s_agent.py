import asyncio
from copy import deepcopy
import os
import os.path as path
import logging
import subprocess
from typing import List
from decouple import config
from .docker_agent import DockerAgent
from .drone_test import AgentConfig, DroneTest, DroneTestResult, SimulationConfig
from . import file_helper

logger = logging.getLogger(__name__)


class K8sAgent(DockerAgent):
    KUBE_CMD = 'yq \'.metadata.name = "{name}" | .spec.template.spec.containers[0].env |= map(select(.name == "COMMAND").value = "{command}") | .spec.completions = {runs} | .spec.parallelism = {runs}\' {template} | kubectl apply -f - --validate=true'
    KUBE_LOCAL_CMD = 'yq \'.metadata.name = "{name}" | .spec.template.spec.volumes[0].hostPath.path = "{host_volume}" | .spec.template.spec.containers[0].env |= map(select(.name == "COMMAND").value = "{command}") | .spec.completions = {runs} | .spec.parallelism = {runs}\' {template} | kubectl apply -f - --validate=true'

    CMD = "timeout {timeout} python3 aerialist exec --test {test_file}"
    WEBDAV_LOCAL_DIR = config("WEBDAV_DL_FLD", default="tmp/")
    DEFAULT_KUBE_TEMPLATE = config(
        "KUBE_TEMPLATE", default="aerialist/resources/k8s/k8s-job.yaml"
    )
    DEFAULT_LOCAL_KUBE_TEMPLATE = config(
        "LOCAL_KUBE_TEMPLATE", default="aerialist/resources/k8s/k8s-job-local.yaml"
    )
    ROS_KUBE_TEMPLATE = config(
        "ROS_KUBE_TEMPLATE", default="aerialist/resources/k8s/k8s-job-avoidance.yaml"
    )
    ROS_LOCAL_KUBE_TEMPLATE = config(
        "ROS_LOCAL_KUBE_TEMPLATE",
        default="aerialist/resources/k8s/k8s-job-avoidance-local.yaml",
    )
    USE_VOLUME = config("KUBE_USE_VOLUME", cast=bool, default=False)
    VOLUME_PATH = config("KUBE_VOLUME_PATH", default="/src/aerialist/results/")

    def __init__(self, config: DroneTest) -> None:
        self.config = config
        self.results: List[DroneTestResult] = []
        self.container_config = self.import_config()

    def run(self):
        cmd = self.CMD.format(
            test_file=self.container_test_yaml,
            timeout=self.DOCKER_TIMEOUT,
        )
        logger.debug("docker command:" + cmd)

        host_volume_prefix = ""

        if self.USE_VOLUME:
            if self.config.simulation.simulator == SimulationConfig.ROS:
                template = self.ROS_LOCAL_KUBE_TEMPLATE
            else:
                template = self.DEFAULT_LOCAL_KUBE_TEMPLATE

            host_volume_prefix = "/host_mnt"
            kube_cmd = self.KUBE_LOCAL_CMD.format(
                name=self.config.agent.id,
                command=cmd,
                host_volume=host_volume_prefix + self.config.agent.path,
                runs=self.config.agent.count,
                template=template,
            )

        else:
            if self.config.simulation.simulator == SimulationConfig.ROS:
                template = self.ROS_KUBE_TEMPLATE
            else:
                template = self.DEFAULT_KUBE_TEMPLATE

            kube_cmd = self.KUBE_CMD.format(
                name=self.config.agent.id,
                command=cmd,
                runs=self.config.agent.count,
                template=template,
            )

        logger.debug("k8s command:" + kube_cmd)
        logger.info("creating k8s job")
        kube_prc = subprocess.run(kube_cmd, shell=True)
        if kube_prc.returncode == 0:
            logger.info("waiting for k8s job to finish ...")
            loop = asyncio.get_event_loop()
            succes = loop.run_until_complete(self.wait_success(self.config.agent.id))
            logger.info("k8s job finished")
            if self.USE_VOLUME:
                local_folder = self.config.agent.path
            else:
                local_folder = f"{self.WEBDAV_LOCAL_DIR}{self.config.agent.id}/"
                os.makedirs(local_folder, exist_ok=True)
                logger.info(f"downloading simulation logs to {local_folder}")
                file_helper.download_dir(self.container_config.agent.path, local_folder)
                logger.debug("files downloaded")

            for test_log in os.listdir(local_folder):
                if test_log.endswith(".ulg") and (
                    self.config.mission is None
                    or self.config.mission.commands_file is None
                    or path.basename(test_log)
                    != path.basename(self.config.mission.commands_file)
                ):
                    self.results.append(DroneTestResult(local_folder + test_log))
            if len(self.results) == 0:
                logger.error(f"k8s job {self.config.agent.id} failed")
                raise Exception(f"k8s job {self.config.agent.id} failed")
            return self.results

        else:
            logger.error(f"k8s process failed with code {kube_prc.returncode}")
            if kube_prc.stdout:
                logger.error(kube_prc.stdout.decode("ascii"))
            if kube_prc.stderr:
                logger.error(kube_prc.stderr.decode("ascii"))
            raise Exception(f"k8s process failed with code {kube_prc.returncode}")

    async def run_async(self):
        pass

    def import_config(self):
        if self.config.agent.id is None:
            self.config.agent.id = file_helper.time_filename()
        else:
            self.config.agent.id += "-" + file_helper.time_filename()
        if not self.config.agent.path.endswith("/"):
            self.config.agent.path += "/"
        self.config.agent.path += self.config.agent.id + "/"

        if self.USE_VOLUME:
            container_config = self.copy_files_volume()
        else:
            container_config = self.copy_files_cloud()
        return container_config

    def copy_files_cloud(self):
        cloud_folder = self.config.agent.path

        container_config = deepcopy(self.config)
        file_helper.create_dir(cloud_folder)

        # Drone Config
        if self.config.drone is not None:
            if self.config.drone.mission_file is not None:
                container_config.drone.mission_file = file_helper.upload(
                    self.config.drone.mission_file, cloud_folder
                )
            if (
                self.config.drone.params is None
                and self.config.drone.params_file is not None
            ):
                container_config.drone.params_file = file_helper.upload(
                    self.config.drone.params_file, cloud_folder
                )

        # Test Config
        if self.config.mission is not None:
            self.config.mission.save_commands_list_if_needed(self.WEBDAV_LOCAL_DIR)
            if (
                self.config.mission.commands_file is not None
                and self.config.mission.commands is not None
                and len(self.config.mission.commands)
                > self.config.mission.MAX_INLINE_COMMANDS
            ):
                container_config.mission.commands_file = file_helper.upload(
                    self.config.mission.commands_file, cloud_folder
                )

        # Assertion Config
        container_config.assertion = None
        # if self.config.assertion is not None:
        #     if self.config.assertion.log_file is not None:
        #         k8s_config.assertion.log_file = file_helper.upload(
        #             self.config.assertion.log_file, cloud_folder
        #         )

        if container_config.agent is not None:
            container_config.agent.engine = AgentConfig.LOCAL
            container_config.agent.count = 1

        self.container_test_yaml = container_config.to_yaml(
            f"{self.WEBDAV_LOCAL_DIR}{self.config.agent.id}.yaml"
        )
        self.container_test_yaml = file_helper.upload(
            self.container_test_yaml, cloud_folder
        )
        logger.info(f"files uploaded")
        return container_config

    async def wait_success(self, job_id):
        completed = await asyncio.create_subprocess_shell(
            f"kubectl wait --for=condition=complete  --timeout=1000s job.batch/{job_id}"
        )
        failed = await asyncio.create_subprocess_shell(
            f"kubectl wait --for=condition=failed  --timeout=1000s job.batch/{job_id}"
        )
        await asyncio.wait(
            [completed.communicate(), failed.communicate()],
            return_when=asyncio.FIRST_COMPLETED,
        )
        try:
            completed.kill()
        except:
            pass
        try:
            failed.kill()
        except:
            pass

        if completed.returncode == 0:
            return True
        else:
            return False
