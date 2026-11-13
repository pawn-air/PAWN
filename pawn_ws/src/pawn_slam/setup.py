from setuptools import find_packages, setup

package_name = "pawn_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Burkay Tunçtürk",
    maintainer_email="burkay.tuncturk@gmail.com",
    description="PAWN monocular SLAM for DJI Tello drones, run offboard on the ground station.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
