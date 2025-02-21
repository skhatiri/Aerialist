from datetime import datetime
import fnmatch
from time import sleep
from typing import List
from decouple import config
from pandas.core.frame import DataFrame
from pyulog import ULog
from bagpy import bagreader
import pandas as pd
import os.path
import shutil
import logging
from webdav4.client import Client
from requests.exceptions import RequestException
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
            webdav_client = Client(
                base_url=config("WEBDAV_HOST"),
                auth=(config("WEBDAV_USER"), config("WEBDAV_PASS")),
                timeout=600,
            )
        except Exception as e:
            logger.error(e)


init_webdav()


def extract(log_address: str, topic: str, columns: List[str] = None) -> DataFrame:
    if log_address.endswith(".bag"):
        return extract_bag(log_address, topic, columns)
    else:
        return extract_ulg(log_address, topic, columns)


def extract_ulg(log_address: str, topic: str, columns: List[str] = None) -> DataFrame:
    """extracts specific messages from the input log and returns the dataframe object"""
    ulog = ULog(log_address, topic, True)
    if len(ulog.data_list) == 0:
        return None
    if columns is None:
        df = pd.DataFrame(ulog.data_list[0].data)
    else:
        df = pd.DataFrame({c: ulog.data_list[0].data[c] for c in columns})
    return df


def extract_bag(log_address: str, topic: str, columns: List[str] = None) -> DataFrame:
    """extracts specific messages from the input bag file and returns the dataframe object"""
    bag = bagreader(log_address)
    topic_csv = bag.message_by_topic(topic)
    df = pd.read_csv(topic_csv)
    if columns is not None:
        df = df[columns]
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
        webdav_client.upload_file(src_file, cloud_path, overwrite=True)
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
    else:
        cloud_dir = dest_dir

    try:
        for root, dirs, files in os.walk(src_dir):
            # Create the directory structure in WebDAV
            relative_path = os.path.relpath(root, src_dir)
            webdav_folder_path = os.path.join(cloud_dir, relative_path).replace(
                "\\", "/"
            )

            # Create the directory on WebDAV if it doesn't exist
            try:
                if not webdav_client.exists(webdav_folder_path):
                    webdav_client.mkdir(webdav_folder_path)
            except RequestException as e:
                logger.error(
                    f"webdav connection lost while creating directory: retrying {RETRIES}"
                )
                sleep(20)
                RETRIES += 1
                if RETRIES <= MAX_WEBDAV_RETRIES:
                    init_webdav()
                    return upload_dir(src_dir, dest_dir)
                else:
                    raise (e)

            # Upload each file in the directory
            for file in files:
                local_file_path = os.path.join(root, file)
                remote_file_path = os.path.join(webdav_folder_path, file).replace(
                    "\\", "/"
                )
                try:
                    webdav_client.upload_file(
                        local_file_path, remote_file_path, overwrite=True
                    )
                except RequestException as e:
                    logger.error(
                        f"webdav connection lost while uploading file: retrying {RETRIES}"
                    )
                    sleep(20)
                    RETRIES += 1
                    if RETRIES <= MAX_WEBDAV_RETRIES:
                        init_webdav()
                        return upload_dir(src_dir, dest_dir)
                    else:
                        raise (e)

        RETRIES = 0
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
    # Ensure the destination directory exists locally
    os.makedirs(dest_path, exist_ok=True)
    dest_path += path.basename(cloud_path)
    try:
        webdav_client.download_file(cloud_path, dest_path)
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
        # Ensure the destination directory exists locally
        os.makedirs(dest_path, exist_ok=True)

        # Fetch the directory's contents
        for item in webdav_client.ls(cloud_path):
            item_path = item["href"].rstrip("/")
            item_name = os.path.basename(item_path)
            remote_item_path = f"{cloud_path}/{item_name}"
            local_item_path = os.path.join(dest_path, item_name)

            if item["type"] == "directory":
                # Recursively download directories
                download_dir(remote_item_path, local_item_path)
            else:
                # Download files
                try:
                    webdav_client.download_file(remote_item_path, local_item_path)
                except RequestException as e:
                    logger.error(f"webdav connection lost: retrying {RETRIES}")
                    sleep(20)
                    RETRIES += 1
                    if RETRIES <= MAX_WEBDAV_RETRIES:
                        init_webdav()
                        return download_dir(src_path, dest_path)
                    else:
                        raise (e)

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
        if not webdav_client.exists(cloud_path):
            current_path = ""
            folders = cloud_path.split("/")
            # Iterate through each folder and create it if it doesn't exist
            for folder in folders:
                current_path = os.path.join(current_path, folder)
                if not webdav_client.exists(current_path):
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


def list_files_in_folder(
    folder: str,
    name_pattern: str,
    search_root: bool = True,
    search_subfolders: bool = False,
    search_recursive: bool = False,
) -> List[str]:
    """
    List files in a folder matching a name pattern, with flexible search options.

    Args:
        folder (str): The folder to search for files.
        name_pattern (str): Pattern to match file names (e.g., '*.yaml').
        search_root (bool): Whether to include files in the root folder.
        search_subfolders (bool): Whether to search in direct subfolders.
        recursive (bool): Whether to search all subfolders recursively.

    Returns:
        List[str]: List of paths to the located files.
    """
    # Ensure the folder exists
    if not os.path.exists(folder) or not os.path.isdir(folder):
        raise ValueError(f"The folder '{folder}' does not exist or is not a directory.")

    located_files = []

    # Search in the root folder
    if search_root:
        for file in os.listdir(folder):
            if os.path.isfile(os.path.join(folder, file)) and fnmatch.fnmatch(
                file, name_pattern
            ):
                located_files.append(os.path.join(folder, file))

    # Search in direct subfolders
    if search_subfolders and not search_recursive:
        for subfolder in os.listdir(folder):
            subfolder_path = os.path.join(folder, subfolder)
            if os.path.isdir(subfolder_path):
                for file in os.listdir(subfolder_path):
                    if os.path.isfile(
                        os.path.join(subfolder_path, file)
                    ) and fnmatch.fnmatch(file, name_pattern):
                        located_files.append(os.path.join(subfolder_path, file))

    # Search recursively in all subfolders
    if search_recursive:
        for root, _, files in os.walk(folder):
            for file in files:
                if fnmatch.fnmatch(file, name_pattern):
                    located_files.append(os.path.join(root, file))
    located_files.sort()
    return located_files
