from setuptools import find_packages, setup

__maintainers__ = ["Enrico Saccon", "Davide De Martini"]

package_name = 'gripper_robotiq_2f85'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico Saccon',
    maintainer_email='enrico.saccon@unitn.it',
    description='Service server to control the Robotiq 2F-85 gripper',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'robotiq_2f85 = gripper_robotiq_2f85.robotiq_2f85:main'
        ],
    },
)
