import asyncio
from copy import deepcopy
import os
import os.path as path
import logging
import subprocess
from decouple import config
from .docker_agent import DockerAgent
from .drone_test import DroneTest, DroneTestResult, SimulationConfig
from . import file_helper

logger = logging.getLogger(__name__)


class K8sAgent(DockerAgent):

    KUBE_CMD = 'yq \'.metadata.name += "-{name}" | .spec.template.spec.containers[0].env |= map(select(.name == "COMMAND").value="{command}") | .spec.completions={runs} | .spec.parallelism={runs}\' {template} | kubectl apply -f - --validate=false'
    WEBDAV_DIR = config("WEBDAV_UP_FLD", default=None)
    WEBDAV_LOCAL_DIR = config("WEBDAV_DL_FLD", default="tmp/")
    DEFAULT_KUBE_TEMPLATE = config("KUBE_TEMPLATE")
    ROS_KUBE_TEMPLATE = config("ROS_KUBE_TEMPLATE")

    def __init__(self, config: DroneTest) -> None:
        super().__init__(config)
        self.k8s_config = self.import_config()

    def run(self, config: DroneTest):
        cmd = self.format_command(
            self.k8s_config.drone.port,
            self.k8s_config.simulation.simulator,
            self.k8s_config.simulation.speed,
            self.k8s_config.assertion.log_file,
            self.k8s_config.test.commands_file,
            self.k8s_config.drone.params_file,
            self.k8s_config.simulation.obstacles,
            self.k8s_config.drone.mission_file,
            self.k8s_config.runner.path,
        )

        logger.debug("docker command:" + cmd)
        kube_cmd = self.KUBE_CMD.format(
            name=self.config.runner.id,
            command=cmd,
            runs=self.config.runner.count,
            template=self.ROS_KUBE_TEMPLATE
            if self.config.simulation.simulator == SimulationConfig.ROS
            else self.DEFAULT_KUBE_TEMPLATE,
        )
        logger.debug("k8s command:" + kube_cmd)
        logger.info("creating k8s job")
        kube_prc = subprocess.run(kube_cmd, shell=True)
        if kube_prc.returncode == 0:
            logger.info("waiting for k8s job to finish ...")
            loop = asyncio.get_event_loop()
            succes = loop.run_until_complete(self.wait_success(self.config.runner.id))
            logger.info("k8s job finished")
            local_folder = f"{self.WEBDAV_LOCAL_DIR}{self.config.runner.id}/"
            os.mkdir(local_folder)
            logger.info(f"downloading simulation logs to {local_folder}")
            file_helper.download_dir(self.k8s_config.runner.path, local_folder)
            logger.debug("files downloaded")

            for test_log in os.listdir(local_folder):
                if test_log.endswith(".ulg") and (
                    self.config.assertion.log_file is None
                    or path.basename(test_log)
                    != path.basename(self.config.assertion.log_file)
                ):
                    self.results.append(DroneTestResult(local_folder + test_log))
            if len(self.results) == 0:
                logger.error(f"k8s job {self.config.runner.id} failed")
                raise Exception(f"k8s job {self.config.runner.id} failed")
            return self.results[0]

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
        k8s_config = deepcopy(self.config)
        self.config.runner.id  # += file_helper.time_filename()
        # cloud_folder = f"{self.WEBDAV_DIR}{self.config.runner.job_id}/"
        # k8s_config.runner.path = cloud_folder
        cloud_folder = self.config.runner.path

        # Drone Config
        if self.config.drone is not None:
            if self.config.drone.mission_file is not None:
                k8s_config.drone.mission_file = file_helper.upload(
                    self.config.drone.mission_file, cloud_folder
                )

            if self.config.drone.params_file is not None:
                k8s_config.drone.params_file = file_helper.upload(
                    self.config.drone.params_file, cloud_folder
                )

        # Test Config
        if self.config.test is not None:
            if self.config.test.commands_file is not None:
                k8s_config.test.commands_file = file_helper.upload(
                    self.config.test.commands_file, cloud_folder
                )

        # Assertion Config
        if self.config.assertion is not None:
            if self.config.assertion.log_file is not None:
                k8s_config.assertion.log_file = file_helper.upload(
                    self.config.assertion.log_file, cloud_folder
                )
        logger.info(f"files uploaded")
        return k8s_config

    async def wait_success(self, job_id):
        completed = await asyncio.create_subprocess_shell(
            f"kubectl wait --for=condition=complete  --timeout=1000s job.batch/{config('KUBE_JOB_NAME')}-{job_id}"
        )
        failed = await asyncio.create_subprocess_shell(
            f"kubectl wait --for=condition=failed  --timeout=1000s job.batch/{config('KUBE_JOB_NAME')}-{job_id}"
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
