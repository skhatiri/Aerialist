import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="aerialist",
    version="0.0.96",
    author="Sajad Khatiri",
    author_email="s.khatiri@gmail.com",
    description="UAV Test Bench",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/skhatiri/Aerialist",
    project_urls={"Bug Tracker": "https://github.com/skhatiri/Aerialist/issues"},
    license="GNU GPLv3",
    packages=setuptools.find_packages(),
    python_requires=">=3.7",
    include_package_data=True,
    install_requires=[
        "similaritymeasures",
        "matplotlib",
        "pyulog",
        "mavsdk",
        "keyboard",
        "python-decouple",
        "csv-logger",
        "statsmodels",
        "webdavclient3",
        "ruptures",
        "shapely",
        "validators",
        "pyyaml",
        "munch",
        "rospkg",
        "defusedxml",
    ],
    entry_points={"console_scripts": ["aerialist=aerialist.entry:main"]},
)
