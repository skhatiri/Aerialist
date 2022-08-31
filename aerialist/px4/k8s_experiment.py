import asyncio
import os
import os.path as path
import logging
import subprocess
from typing import List
from decouple import config
from .docker_experiment import DockerExperiment
from .trajectory import Trajectory
from . import ulog_helper

logger = logging.getLogger(__name__)


class K8sExperiment(DockerExperiment):

    KUBE_CMD = 'yq \'.metadata.name += "-{name}" | .spec.template.spec.containers[0].env |= map(select(.name == "COMMAND").value="{command}") | .spec.completions={runs} | .spec.parallelism={runs}\' {template} | kubectl apply -f - --validate=false'
    WEBDAV_DIR = config("WEBDAV_UP_FLD", default=None)

    def __init__(
        self,
        drone: str = config("DRONE", default="sim"),
        env: str = config("SIMULATOR", default="gazebo"),
        headless: bool = True,
        speed: float = config("SPEED", default=1, cast=int),
        log: str = None,
        is_jmavsim=False,
        params_csv=None,
        obstacles: List[float] = None,
        mission_file: str = None,
        experiment_log: str = None,
    ) -> None:

        self.simulator = env
        self.drone = drone
        self.speed = speed
        self.headless = headless
        self.original_log = log
        self.params_csv = params_csv
        self.mission = mission_file
        self.obstacles = obstacles
        self.log = experiment_log
        self.original_trajectory = None
        if self.original_log is not None and self.original_log.endswith(".ulg"):
            self.original_trajectory = Trajectory.extract_from_log(
                self.original_log, is_jmavsim=is_jmavsim
            )
        if self.log:
            self.trajectory = Trajectory.extract_from_log(
                self.log,
            )

    def replay(self):
        pass

    async def replay_async(self):
        pass

    def load_commands(self):
        pass

    @classmethod
    def replay_parallel(
        cls,
        runs: int = 1,
        drone: str = config("DRONE", default="sim"),
        env: str = config("SIMULATOR", default="gazebo"),
        headless: bool = True,
        speed: float = config("SPEED", default=1, cast=int),
        log: str = None,
        is_jmavsim=False,
        params_csv=None,
        obstacles: List[float] = None,
        mission_file: str = None,
        job_prefix: str = "",
    ):
        job_id = job_prefix + ulog_helper.time_filename()
        cloud_folder = f"{cls.WEBDAV_DIR}{job_id}/"
        ulog_helper.create_dir(cloud_folder)
        logger.info(f"uploading cloud files to {cloud_folder}")
        cloud_log = ulog_helper.upload(log, cloud_folder) if log else None
        cloud_mission_file = (
            ulog_helper.upload(mission_file, cloud_folder) if mission_file else None
        )
        cloud_params_csv = (
            ulog_helper.upload(params_csv, cloud_folder) if params_csv else None
        )
        logger.info(f"files uploaded")

        cmd = cls.format_command(
            drone,
            env,
            speed,
            cloud_log,
            cloud_params_csv,
            obstacles,
            cloud_mission_file,
            True,
            cloud_folder,
        )

        logger.debug("docker command:" + cmd)
        kube_cmd = cls.KUBE_CMD.format(
            name=job_id,
            command=cmd,
            runs=runs,
            template=config("KUBE_TEMPLATE"),
        )
        logger.debug("k8s command:" + kube_cmd)
        logger.info("creating k8s job")
        kube_prc = subprocess.run(kube_cmd, shell=True)
        if kube_prc.returncode == 0:
            logger.info("waiting for k8s job to finish ...")
            loop = asyncio.get_event_loop()
            succes = loop.run_until_complete(cls.wait_success(job_id))
            logger.info("k8s job finished")
            local_folder = f'{config("WEBDAV_DL_FLD")}{job_id}/'
            os.mkdir(local_folder)
            logger.info(f"downloading simulation logs to {local_folder}")
            ulog_helper.download_dir(cloud_folder, local_folder)
            logger.debug("files downloaded")

            experiments: List[K8sExperiment] = []
            for exp_log in os.listdir(local_folder):
                if exp_log.endswith(".ulg") and (
                    log is None or path.basename(exp_log) != path.basename(log)
                ):
                    experiments.append(
                        K8sExperiment(
                            drone,
                            env,
                            headless,
                            speed,
                            log,
                            is_jmavsim,
                            params_csv,
                            obstacles,
                            mission_file,
                            local_folder + exp_log,
                        )
                    )
            if len(experiments) == 0:
                logger.error(f"k8s job {job_id} failed")
                raise Exception(f"k8s job {job_id} failed")
            return experiments

        else:
            logger.error(f"k8s process failed with code {kube_prc.returncode}")
            if kube_prc.stdout:
                logger.error(kube_prc.stdout.decode("ascii"))
            if kube_prc.stderr:
                logger.error(kube_prc.stderr.decode("ascii"))
            raise Exception(f"k8s process failed with code {kube_prc.returncode}")

    @classmethod
    async def wait_success(cls, job_id):
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
