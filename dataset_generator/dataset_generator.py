import os
import pandas as pd
from multiprocessing import Pool
from datetime import datetime

try:
    from log_parser import LogParser
except:
    from training.log_parser import LogParser


class DatasetGenerator(object):
    METADATA = None

    def __init__(
        self,
        data_folder: str,
        metadata: str = None,
        output_folder_name: str = "output",
        overwrite: bool = False,
        window_length: int = 10000000,
        window_overlap: int = 5000000,
    ):
        self.dataset_folder = data_folder
        self.overwrite = overwrite
        self.output_folder = os.path.join(
            os.path.dirname(data_folder), output_folder_name
        )
        os.makedirs(self.output_folder, exist_ok=True)
        if metadata is not None:
            self.metadata = pd.read_csv(metadata, skipinitialspace=True)

        self.window_length = window_length
        self.window_overlap = window_overlap

        self.data_structure = self.discover_structure(data_folder)

    @classmethod
    def discover_structure(cls, data_folder: str) -> dict:
        file_dict = {}
        for root, dirs, files in os.walk(data_folder):
            for file in files:
                if file.endswith(".ulg"):
                    file_dict.setdefault(root, []).append(file)
        return file_dict

    def run_extraction(self) -> bool:
        folder_df = None

        for folder, files in sorted(self.data_structure.items(), key=lambda x: x[0]):
            print(f"Running {folder} containing {len(files)} logs")
            if folder_df is None:
                folder_df = self.subfolder_csv(folder, files)
            else:
                folder_df = pd.concat(
                    [folder_df, self.subfolder_csv(folder, files)],
                    ignore_index=True,
                )

        folder_df.to_csv(
            os.path.join(
                self.output_folder,
                f"{datetime.now().strftime('%m-%d-%Y-%H-%M-%S')}_wl_{self.window_length}_wo_{self.window_overlap}.csv",
            )
        )
        return True

    def run_parallel_extraction(self) -> bool:
        folder_df = None
        results = []

        with Pool() as pool:
            for folder, files in sorted(
                self.data_structure.items(), key=lambda x: x[0]
            ):
                results.append(pool.apply_async(self.subfolder_csv, (folder, files)))
            pool.close()
            pool.join()

        folder_df = pd.concat([r.get() for r in results])

        folder_df.to_csv(
            os.path.join(
                self.output_folder,
                f"{datetime.now().strftime('%m-%d-%Y-%H-%M-%S')}_wl_{self.window_length}_wo_{self.window_overlap}.csv",
            )
        )
        return True

    def log_analysis(self, folder_path, log_file):
        log_parser = LogParser(os.path.join(folder_path, log_file), self.metadata)
        log_parser.extract_dataset(self.window_length, self.window_overlap)
        return log_parser.dataset

    def subfolder_csv(self, folder_path: str, file_list: list) -> bool:
        print(f"Running {folder_path} containing {len(file_list)} logs")

        log_dfs = [self.log_analysis(folder_path, log_file) for log_file in file_list]
        subfolder_df = pd.concat(log_dfs)

        subfolder_df.to_csv(
            os.path.join(
                self.output_folder,
                f"{os.path.basename(folder_path)}_{datetime.now().strftime('%m-%d-%Y-%H-%M-%S')}_wl_{self.window_length}_wo_{self.window_overlap}.csv",
            )
        )
        print(f"Finished {folder_path} containing {len(file_list)} logs")
        return subfolder_df
