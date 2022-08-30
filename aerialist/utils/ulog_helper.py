from datetime import datetime
from time import sleep
from decouple import config
from pandas.core.frame import DataFrame
from pyulog import ulog2csv
import pandas as pd
import os.path
import shutil
import logging
from webdav3.client import Client
import webdav3.exceptions
from os import path
import os

logger = logging.getLogger(__name__)
MAX_WEBDAV_RETRIES = 5
RETRIES = 0


def init_webdav():
    if config("WEBDAV_PASS", default=None):
        try:
            global webdav_client
            options = {
                "webdav_hostname": config("WEBDAV_HOST"),
                "webdav_login": config("WEBDAV_USER"),
                "webdav_password": config("WEBDAV_PASS"),
                "webdav_timeout": 600,
            }
            webdav_client = Client(options)
        except Exception as e:
            logger.error(e)


init_webdav()


def extract(log_address: str, topic: str, use_cache=True) -> DataFrame:
    """extracts specific messages from the input log and returns the csv object"""

    csv_add = f"{log_address[:-4]}_{topic}_0.csv"
    if not (use_cache and os.path.isfile(csv_add)):
        ulog2csv.convert_ulog2csv(log_address, topic, None, ",")

    if os.path.isfile(csv_add):
        data = pd.read_csv(csv_add)
        return data
    else:
        return None


def copy(src_file, dest_file) -> bool:
    try:
        path = shutil.copy2(src_file, dest_file)
        logger.debug("File copied successfully.")
        return path
    except shutil.SameFileError:
        logger.debug("Source and destination represents the same file.")
    except IsADirectoryError:
        logger.debug("Destination is a directory.")
    except PermissionError:
        logger.error("Permission denied.")
    except:
        logger.error("Error occurred while copying file.")

    return None


def time_filename(add_host=False):
    name = datetime.now().strftime("%d-%m-%H-%M-%S")
    if add_host:
        name += "-" + os.uname().nodename.split("-")[-1][:5]
    return name


def upload(src_file, dest_path) -> str:
    global RETRIES
    dest_path += path.basename(src_file)
    try:
        webdav_client.upload_file(dest_path, src_file)
        RETRIES = 0
    except webdav3.exceptions.NoConnection as e:
        logger.error(f"webdav connection lost: retrying {RETRIES}")
        sleep(20)
        RETRIES += 1
        if RETRIES <= MAX_WEBDAV_RETRIES:
            init_webdav()
            return upload(src_file, dest_path)
        else:
            raise (e)
    return dest_path


def download(src_file, dest_path) -> str:
    global RETRIES
    dest_path += path.basename(src_file)
    try:
        webdav_client.download_file(src_file, dest_path)
        RETRIES = 0
    except webdav3.exceptions.NoConnection as e:
        logger.error(f"webdav connection lost: retrying {RETRIES}")
        sleep(20)
        RETRIES += 1
        if RETRIES <= MAX_WEBDAV_RETRIES:
            init_webdav()
            return download(src_file, dest_path)
        else:
            raise (e)
    return dest_path


def download_dir(src_path, dest_path) -> str:
    global RETRIES
    try:
        webdav_client.download_directory(src_path, dest_path)
        RETRIES = 0
    except webdav3.exceptions.NoConnection as e:
        logger.error(f"webdav connection lost: retrying {RETRIES}")
        sleep(20)
        RETRIES += 1
        if RETRIES <= MAX_WEBDAV_RETRIES:
            init_webdav()
            return download_dir(src_path, dest_path)
        else:
            raise (e)
    return dest_path


def create_dir(path):
    global RETRIES
    try:
        if not webdav_client.check(path):
            webdav_client.mkdir(path)
        RETRIES = 0
    except webdav3.exceptions.NoConnection as e:
        logger.error(f"webdav connection lost: retrying {RETRIES}")
        sleep(20)
        RETRIES += 1
        if RETRIES <= MAX_WEBDAV_RETRIES:
            init_webdav()
            return create_dir(path)
        else:
            raise (e)


def get_logs_address(path):
    files = [path + f for f in os.listdir(path) if f.endswith(".ulg")]
    return files