from datetime import datetime
from time import sleep
from typing import List
from decouple import config
from pandas.core.frame import DataFrame
from pyulog import ULog
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


def get_local_file(file_path: str):
    if is_webdav_address(file_path):
        local_add = download(file_path, config("WEBDAV_DL_FLD", default="tmp/"))
        return local_add
    elif path.exists(file_path):
        return file_path
    else:
        raise Exception(f"path does not exist:{file_path}")


def is_webdav_address(address: str):
    return address.startswith("webdav://")


def get_webdav_path(address: str):
    return address.replace("webdav://", "")


def get_local_folder(folder_path: str):
    if is_webdav_address(folder_path):
        if folder_path.endswith("/"):
            folder = folder_path.split("/")[-2]
        else:
            folder = folder_path.split("/")[-1]
        local_add = download_dir(
            folder_path, f'{config("WEBDAV_DL_FLD", default="tmp/")}{folder}/'
        )
        return local_add
    elif path.exists(folder_path):
        return folder_path
    else:
        raise Exception("path does not exist")


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


def extract(log_address: str, topic: str, columns: List[str] = None) -> DataFrame:
    """extracts specific messages from the input log and returns the dataframe object"""
    ulog = ULog(log_address, topic, True)
    if len(ulog.data_list) == 0:
        return None
    if columns is None:
        df = pd.DataFrame(ulog.data_list[0].data)
    else:
        df = pd.DataFrame({c: ulog.data_list[0].data[c] for c in columns})
    return df


def copy(src_file: str, dest_file: str) -> bool:
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


def upload(src_file: str, dest_path: str) -> str:
    global RETRIES
    dest_path += path.basename(src_file)
    cloud_path = dest_path
    if is_webdav_address(dest_path):
        cloud_path = get_webdav_path(dest_path)
    try:
        webdav_client.upload_file(cloud_path, src_file)
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


def upload_dir(src_dir: str, dest_dir: str) -> str:
    global RETRIES
    if is_webdav_address(dest_dir):
        cloud_dir = get_webdav_path(dest_dir)
        try:
            webdav_client.upload_sync(local_path=src_dir, remote_path=cloud_dir)
        except webdav3.exceptions.NoConnection as e:
            logger.error(f"webdav connection lost: retrying {RETRIES}")
            sleep(20)
            RETRIES += 1
            if RETRIES <= MAX_WEBDAV_RETRIES:
                init_webdav()
                return upload_dir(src_dir, dest_dir)
            else:
                raise (e)
    return dest_dir


def download(src_file: str, dest_path: str) -> str:
    global RETRIES
    cloud_path = src_file
    if is_webdav_address(src_file):
        cloud_path = get_webdav_path(src_file)
    dest_path += path.basename(cloud_path)
    try:
        webdav_client.download_file(cloud_path, dest_path)
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


def download_dir(src_path: str, dest_path: str) -> str:
    global RETRIES
    cloud_path = src_path
    if is_webdav_address(src_path):
        cloud_path = get_webdav_path(src_path)
    try:
        webdav_client.download_directory(cloud_path, dest_path)
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


def create_dir(path: str):
    global RETRIES
    cloud_path = path
    if is_webdav_address(path):
        cloud_path = get_webdav_path(path)
    try:
        if not webdav_client.check(cloud_path):
            current_path = ""
            folders = cloud_path.split("/")
            # Iterate through each folder and create it if it doesn't exist
            for folder in folders:
                current_path = os.path.join(current_path, folder)
                if not webdav_client.check(current_path):
                    webdav_client.mkdir(current_path)
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
