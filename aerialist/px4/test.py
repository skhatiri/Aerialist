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
from webdav4.client import Client
from requests.exceptions import RequestException
# import webdav4.exceptions
from os import path
import os
import zipfile

logger = logging.getLogger(__name__)
MAX_WEBDAV_RETRIES = 5
RETRIES = 0
webdav_client = None

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
    try:
        global webdav_client
        options = {
            "base_url": config("WEBDAV_HOST"),
            "auth": (config("WEBDAV_USER"), config("WEBDAV_PASS")),
            "timeout": 600,
        }
        webdav_client = Client(options)
        print(f"webdav client initialized as {webdav_client}")
        src_file = "webdav://test/case_studies/scenario4-5_10m.plan"
        dest_path = "/home/prasun/Documents/Aerialist/aerialist/px4/"

        global RETRIES
        cloud_path = src_file
        if is_webdav_address(src_file):
            cloud_path = get_webdav_path(src_file)
        dest_path += path.basename(cloud_path)
        try:
            with open(dest_path, 'wb') as file_obj:
                webdav_client.download_from(cloud_path, file_obj)
            RETRIES = 0
        except RequestException as e:
            logger.error(f"webdav connection lost: retrying {RETRIES}")
            sleep(20)
            RETRIES += 1
            if RETRIES <= MAX_WEBDAV_RETRIES:
                init_webdav()
                return download(src_file, dest_path)
            else:
                raise (e)
        return dest_path

    except Exception as e:
        logger.error(e)





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
        with open(src_file, 'rb') as file_obj:
            webdav_client.upload_to(cloud_path, file_obj)
        RETRIES = 0
    except RequestException as e:
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
            for root, dirs, files in os.walk(src_dir):
                remote_root = os.path.join(cloud_dir, os.path.relpath(root, src_dir)).replace("\\", "/")
                if not webdav_client.exists(remote_root):
                    webdav_client.mkdir(remote_root)

                for file in files:
                    local_file_path = os.path.join(root, file)
                    remote_file_path = os.path.join(remote_root, file).replace("\\", "/")
                    with open(local_file_path, 'rb') as file_obj:
                        webdav_client.upload_to(remote_file_path, file_obj)
        except RequestException as e:
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
        with open(dest_path, 'wb') as file_obj:
            webdav_client.download_from(cloud_path, file_obj)
        RETRIES = 0
    except RequestException as e:
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
    except RequestException as e:
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
    except RequestException as e:
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


def zip_files_folders(file_path_list):
    zip_list = []
    for temp_file_path in file_path_list:
        if os.path.isfile(temp_file_path):
            base_dir = os.path.dirname(temp_file_path)
            file_name = os.path.basename(temp_file_path)

            archive_path = os.path.join(base_dir, file_name + ".zip")
            with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as zipf:
                zipf.write(temp_file_path, arcname=file_name)
            temp_file_path_ts = archive_path

        else:
            temp_file_path_ts = (
                    temp_file_path + "_" + str(datetime.now().strftime("%Y%m%d%H%M%S"))
            )
            shutil.make_archive(temp_file_path_ts, "zip", root_dir=temp_file_path)
            temp_file_path_ts = temp_file_path_ts + ".zip"
        zip_list.append(temp_file_path_ts)
    return zip_list


print("CALLing init")
init_webdav()
print("finished calling init")
# download("webdav://test/case_studies/scenario4-5_10m.plan","/home/prasun/Documents/Aerialist/aerialist/px4/")