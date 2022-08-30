import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="Aerialist",
    version="0.0.1",
    author="Sajad Khatiri",
    author_email="s.khatiri@gmail.com",
    description="UAV Test Bench",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/skhatiri/Aerialist",
    project_urls={"Bug Tracker": "https://github.com/skhatiri/Aerialist/issues"},
    license="MIT",
    packages=["aerialist"],
    install_requires=["requests"],
)
