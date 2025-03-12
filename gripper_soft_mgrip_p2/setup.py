from setuptools import find_packages, setup

__maintainers__ = ["Enrico Saccon", "Davide De Martini", "Marco Roveri", "Davide Nardi"]

package_name = "gripper_soft_mgrip_p2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/ur_scripts", ["ur_scripts/close.script"]),
        ("share/" + package_name + "/ur_scripts", ["ur_scripts/open.script"]),
        ("share/" + package_name + "/ur_scripts", ["ur_scripts/neutral.script"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enrico Saccon",
    maintainer_email="enrico.saccon@unitn.it",
    description="Service server to control the Soft Robotics mGrip P2",
    license="Apache 2.0",
    entry_points={
        "console_scripts": ["mgrip_p2 = gripper_soft_mgrip_p2.mgrip_p2:main"],
    },
)
