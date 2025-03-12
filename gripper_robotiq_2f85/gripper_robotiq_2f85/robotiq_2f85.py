# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2025 Enrico Saccon
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -- END LICENSE BLOCK ------------------------------------------------

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2019 FZI Forschungszentrum Informatik
# Created on behalf of Universal Robots A/S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This code is based on the following website:
# https://dof.robotiq.com/discussion/92/controlling-the-robotiq-2-finger-gripper-with-modbus-commands-in-python

__maintainers__ = ["Enrico Saccon", "Davide De Martini"]
    
#!/usr/bin/env python3

import subprocess
import os
import time
import serial
import binascii

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from gripper_interfaces.srv import Gripper

class Robotiq2f85(Node):
    def __init__(self):
        super().__init__("Robotiq2f85")
                
        self.interface_path = os.path.join(get_package_share_directory('gripper_robotiq_2f85'), 'ttyUR0')

        self.get_logger().info(f"interface: {self.interface_path}")

        # Declare and read parameters
        self.declare_parameter("host")  # Robot IP
        self.declare_parameter("port", value=54321)  # Default port
        self.declare_parameter("device_name", value=self.interface_path)  # Default device name

        self.robot_ip = self.get_parameter("host").value
        self.tcp_port = str(self.get_parameter("port").value)
        self.local_device = self.get_parameter("device_name").value

        self.get_logger().info(f"Robot IP: {self.robot_ip}")
        self.get_logger().info(f"Port: {self.tcp_port}")

        self.get_logger().info(f"Remote device will be available at '{self.local_device}'")

        # Configure the socat command
        cfg_params = ["pty", f"link={self.local_device}", "raw", "ignoreeof", "waitslave"]
        cmd = ["socat", ",".join(cfg_params), ":".join(["tcp", self.robot_ip, self.tcp_port])]

        self.get_logger().info(f"Starting socat with the following command:\n{' '.join(cmd)}")
        try:
            subprocess.Popen(cmd)
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Failed to start socat: {e}")
        
        self.get_logger().info("Connecting to gripper")
        
        self.ser = serial.Serial(
            port=self.interface_path,
            baudrate=115200,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        self.init_communication()
        
        self.srv = self.create_service(Gripper, 'r2f85_gripper', self.gripper_callback)
    
    def gripper_callback(self, request, response):
        # Open request
        if request.command.lower().startswith('o'):
            self.get_logger().info("Opening gripper")
            self.open_gripper()
            response.success = True
            response.status = "Gripper opened, moving up"
            return response
            
        # Close request
        elif request.command.lower().startswith('c'):
            self.get_logger().info("Closing gripper")
            self.close_gripper()
            response.success = True
            response.status = "Gripper closed"
            return response
            
        # Initialize communication
        elif request.command.lower().startswith('a'):
            self.get_logger().info("Initializing communication")
            self.init_communication()
            self.get_logger().info("Communication initialized")
            response.success = True
            response.status = "Gripper connected"
            return response
        
        # Get gripper status
        elif request.command.lower().startswith('s'):
            self.get_logger().info("Getting gripper status")
            status = self.get_status()
            response.success = True
            response.status = status if status is not None else "Gripper status retrieved, but empty"
            return response
            
        else:
            self.get_logger().info("Invalid command received")
            response.success = False
            response.status = "Invalid command"
            return response
                
            
    def init_communication(self) -> None:
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()  # Convert bytes to string
        self.get_logger().info(f"Enabled gripper: {data}")
        time.sleep(0.01)

    def get_status(self) -> None:
        self.ser.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        self.get_logger().info(f"Gripper status: {data}")
        time.sleep(1)
        
    def close_gripper(self) -> str:
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        self.get_logger().info(f"Gripper closed: {data}")
        return data
    
    def open_gripper(self) -> None:
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        self.get_logger().info(f"Gripper open: {data}")
        

def main(args=None):
    rclpy.init(args=args)
    node = Robotiq2f85()
    rclpy.spin(node)

    # Shutdown ROS2 properly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
