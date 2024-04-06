import os
from glob import glob
from setuptools import find_packages, setup

package_name = "freefly_challenge"

setup(
    name = package_name,
    version = "0.0.0",
    packages = find_packages(exclude = ["test"]),
    data_files = [
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "mission"), glob("mission/*.csv")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py"))],
    install_requires = ["setuptools"],
    zip_safe = True,
    maintainer = "Varundev Sukhil",
    maintainer_email = "vsukhil@gmail.com",
    description = "TODO: Package description",
    license = "TODO: License declaration",
    tests_require = ["pytest"],
    entry_points = {"console_scripts": [
            f"mission = {package_name}.mission:main",
            f"visualizer = {package_name}.visualizer:main",],},)
