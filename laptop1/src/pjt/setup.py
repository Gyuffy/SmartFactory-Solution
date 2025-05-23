from setuptools import find_packages, setup
from glob import glob

package_name = "pjt"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ssafy",
    maintainer_email="dan360@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dobot_homing_node = pjt.dobot_homing:main",
            "socket_server_node = pjt.socket_server:main",
            "detect_panel_node = pjt.realsensewithYolo11_roi:main",
        ],
    },
)
