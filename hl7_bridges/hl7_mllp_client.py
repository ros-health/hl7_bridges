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

class HL7MLLPClient(HL7Bridge):
    def __init__(self, node_name):
        self.server = None
        super().__init__(node_name)

        # verify that we have the parameters we need
        server_param = self.get_parameter('server')
        if server_param.value is None:
            raise ValueError('HL7Bridge needs "server" param: hostname/IP')

        self.server = server_param.value
        self.get_logger().info('subscribing to hl7 output messages')
        self.create_subscription(HL7V2, 'hl7', self.hl7_cb)

    async def handle_connection(self):
        try:
            while True:
                try:
                    self.get_logger().info(
                        f'connecting to {self.server}:{self.port}')
                    reader, writer = await asyncio.open_connection(
                        self.server, self.port)
                    await self.connection_task(reader, writer)
                except OSError as e:
                    self.get_logger().error(str(e))
                finally:
                    await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass

    def hl7_cb(self, msg):
        # this callback fires when HL7V2 message arrives from ROS2
        self.get_logger().info('hl7_cb(): {}'.format(msg))
        # first, convert the message fields to an MSH segment
        msh_segment = 'MSH|^~\\&|'
        msh_segment += '{}|{}|{}|{}|'.format(
            msg.sending_application, msg.sending_facility,
            msg.receiving_application, msg.receiving_facility)
        msh_segment += '{}|{}|{}|{}|'.format(
            msg.timestamp, msg.security,
            msg.message_type, msg.message_control_id)
        msh_segment += '{}|{}|{}|{}|{}|{}|'.format(
            msg.processing_id, msg.version_id,
            msg.sequence_number, msg.continuation_pointer,
            msg.accept_ack_type, msg.application_ack_type)
        msh_segment += '{}|{}|{}||'.format(
            msg.country_code, msg.character_set, msg.language)
        segments = [msh_segment] + msg.segments
        if self.mllp_send_queue is None:
            self.get_logger().info('not forwarding to hl7')
            return
        self.get_logger().info('forwarding to hl7')
        self.mllp_send_queue.sync_q.put(segments)

def main():
    rclpy.init()
    HL7MLLPClient('hl7_mllp_client').main()

if __name__ == '__main__':
    main()
