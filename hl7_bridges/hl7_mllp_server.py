# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import asyncio
import rclpy
from rmf_msgs.msg import HL7V2

from .hl7_bridge import HL7Bridge

class HL7MLLPServer(HL7Bridge):
    def __init__(self, node_name):
        self.hl7_pub = None
        super().__init__(node_name)

        self.hl7_pub = self.create_publisher(HL7V2, 'hl7')

    async def handle_connection(self):
        await asyncio.start_server(self.connection_task,
            '', self.port)
        self.get_logger().info(f'listening on {self.port}')

    async def rx_msg_handler(self, hl7msg):
        self.hl7_pub.publish(hl7msg)

def main():
    rclpy.init()
    HL7MLLPServer('hl7_mllp_server').main()

if __name__ == '__main__':
    main()
