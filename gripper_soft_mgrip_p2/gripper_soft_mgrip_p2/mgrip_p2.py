"""
Copyright 2025 Enrico Saccon

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

__maintainers__ = ["Enrico Saccon", "Davide De Martini", "Marco Roveri", "Davide Nardi"]

#!/usr/bin/env python3

import os
import socket
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger

from gripper_interfaces.srv import Gripper


class GripperController(Node):
    def __init__(self):
        super().__init__("gripper_controller_execution")

        self.STATE = 1 # 0 = close, 1 = neutral, 2 = open

        # Create parameters 
        self.declare_parameter("host") # The IP of the robot
        self.declare_parameter("port", value=30002) # The port of the robot
        self.declare_parameter("gripper_service_name", value="soft_gripper") # The name of the service

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.gripper_service_name = self.get_parameter("gripper_service_name").value

        # Create a socket connection
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.host, self.port))

        # Path to your scripts
        self.path = os.path.join(
            get_package_share_directory("gripper_soft_mgrip_p2"), "ur_scripts"
        )

        # Create service server
        self.sub = self.create_service(Gripper, self.gripper_service_name, self.callback)

        self.get_logger().info("Soft gripper server is running")

    def callback(self, request, response):
        print(f"received: {request.command}")
        script = ""

        # Check command if it's possible to execute it
        if request.command.lower().startswith('o') and self.STATE < 2:
            script = os.path.join(self.path, "open.script")
            response.status = "Opening gripper"
        elif request.command.lower().startswith('o') and self.STATE == 2:
            response.status = "Gripper already open"
            response.success = False
            self.get_logger().info(response.status)
            return response
        
        elif request.command.lower().startswith('c') and self.STATE > 0:
            script = os.path.join(self.path, "close.script")
            response.status = "Closing gripper"
        elif request.command.lower().startswith('c') and self.STATE == 0:
            response.status = "Gripper already closed"
            response.success = False
            self.get_logger().info(response.status)
            return response
        
        else:
            response.status = f"Invalid argument {request.command}"
            response.success = False
            self.get_logger().info(response.status)
            return response

        # Send the script to the robot
        try:
            with open(script, "rb") as f:
                l = f.read(2024)
                while l:
                    self.s.send(l)
                    l = f.read(2024)
                    
            #! TODO check if this is really necessary
            time.sleep(1.0)
                    
            # Call service /io_and_status_controller/resend_robot_program sending std_srvs/srv/Trigger
            client = self.create_client(Trigger, "io_and_status_controller/resend_robot_program")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Service not available, waiting again...")
            new_request = Trigger.Request()
            future = client.call_async(new_request)
            
        except FileNotFoundError:
            self.get_logger().error(f"Script file {script} not found.")
            response.status = F"Error: script {script} not found"
            response.success = False
        except Exception as e:
            self.get_logger().error(f"Error while sending the script: {e}")
            response.status = f"Error while sending the script {e}"
            response.success = False
        response.success = True

        # Update the state
        if request.command.lower().startswith('o'):
            self.STATE += 1
        elif request.command.lower().startswith('c'):
            self.STATE -= 1

        assert self.STATE >= 0 and self.STATE <= 2, f"Invalid state {self.STATE} after command {request.command}"

        self.get_logger().info(f"New state is {self.STATE}")

        return response


def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()

    rclpy.spin(gripper_controller)

    # Clean up and shutdown
    gripper_controller.s.close()
    gripper_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
