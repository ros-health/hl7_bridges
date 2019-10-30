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
from enum import Enum
import threading

import janus
import rclpy
from rclpy.node import Node
from rmf_msgs.msg import HL7V2


class HL7Bridge(Node):
    def __init__(self, node_name):
        self.port = None
        self.mllp_event_loop = None
        self.mllp_send_queue = None
        self.mllp_thread = None
        super().__init__(node_name)

        # verify that we have the parameters we need
        port_param = self.get_parameter('port')
        if port_param.value is None:
            raise ValueError('HL7Bridge needs "port" param')

        self.port = port_param.value

    def mllp_thread_func(self):
        self.mllp_event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.mllp_event_loop)
        self.get_logger().info('entering MLLP worker thread')
        self.mllp_send_queue = janus.Queue()

        self.get_logger().info('running async event loop until completion...')
        asyncio.ensure_future(self.handle_connection())
        self.mllp_event_loop.run_forever()

        # clean up pending tasks
        all_tasks = asyncio.Task.all_tasks()
        if all_tasks:
            cleanup = asyncio.ensure_future(asyncio.wait(all_tasks))
            self.mllp_event_loop.run_until_complete(cleanup)
        self.get_logger().info('exiting mllp thread')

    async def handle_connection(self):
        raise NotImplementedError('to be implemented by subclasses')

    def mllp_spin(self):
        self.get_logger().info('entering MLLP spin loop... Ctrl+C to exit')
        self.mllp_thread = threading.Thread(target=self.mllp_thread_func)
        self.mllp_thread.start()
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        self.get_logger().info('cancelling all async tasks')
        self.mllp_event_loop.call_soon_threadsafe(
            lambda: [task.cancel() for task in asyncio.Task.all_tasks()])
        self.mllp_event_loop.stop()
        self.get_logger().info('joining MLLP thread...')
        self.mllp_thread.join()
        self.get_logger().info('MLLP server thread joined. Goodbye!')

    async def mllp_send(self, segments, writer):
        msg_bytes = b'\x0b' + '\r'.join(segments).encode('utf-8') + b'\x1c\x0d'

        self.get_logger().info(
            'sending {}-byte message. Segments:'.format(len(msg_bytes)))
        for segment in segments:
            self.get_logger().info('  {}'.format(segment))

        try:
            writer.write(msg_bytes)
            await writer.drain()
        except Exception as ex:
            self.get_logger().error('exception in send_mllp: {}'.format(ex))
            raise

    async def rx_msg_handler(self, hl7msg):
        pass

    async def rx_msg(self, mllp_msg, writer):
        self.get_logger().info(
            'received message of length {}'.format(len(mllp_msg)))
        if mllp_msg[0:4] != 'MSH|':
            self.get_logger().info("OH NO, weird message:")
            self.get_logger().info(mllp_msg)
            return
        self.get_logger().info('**************************\nreceived:')
        self.get_logger().info('\n'.join(mllp_msg.split('\r')))
        segments = mllp_msg.split('\r')
        msh = segments[0]
        msh_fields = msh.split('|')
        msh_encoding = msh_fields[1]
        sending_application = msh_fields[2]
        sending_facility = msh_fields[3]
        receiving_application = msh_fields[4]
        receiving_facility = msh_fields[5]
        msg_time = msh_fields[6]
        msg_security = msh_fields[7]
        msg_type = msh_fields[8]
        msg_control_id = msh_fields[9]
        msg_processing_id = msh_fields[10]
        msg_version_id = msh_fields[11]
        # The rest of the fields are optional. Let's ignore them for now.
        if 'ACK' in msg_type:
            self.get_logger().info("received ack.")
            return  # No need for further processing. Don't ACK an ACK.

        ack_msh_fields = [
            'MSH', msh_encoding,
            receiving_application, receiving_facility,
            sending_application, sending_facility,
            msg_time, msg_security,
            'ACK^A01',
            msg_time,  # todo: Should this be msg_control_id? Not clear yet.
            msg_processing_id, msg_version_id,
            '','','AL','NE'
        ]

        ack_msh_seg = '|'.join(ack_msh_fields)
        ack_msa_fields = [ 'MSA', 'AA', msg_control_id ]
        ack_msa_seg = '|'.join(ack_msa_fields)
        ack_segs = [ ack_msh_seg, ack_msa_seg ]
        await self.mllp_send(ack_segs, writer)

        # now create a ROS 2 message and copy the fields in
        ros2_msg = HL7V2()
        ros2_msg.sending_application = sending_application
        ros2_msg.sending_facility = sending_facility
        ros2_msg.receiving_application = receiving_application
        ros2_msg.receiving_facility = receiving_facility
        ros2_msg.timestamp = msg_time
        ros2_msg.security = msg_security
        ros2_msg.message_type = msg_type
        ros2_msg.message_control_id = msg_control_id
        ros2_msg.processing_id = msg_processing_id
        ros2_msg.version_id = msg_version_id

        # todo: add the optional fields if they are here
        ros2_msg.sequence_number = ''
        ros2_msg.continuation_pointer = ''
        ros2_msg.accept_ack_type = ''
        ros2_msg.application_ack_type = ''
        ros2_msg.country_code = ''
        ros2_msg.character_set = ''
        ros2_msg.language = ''
        ros2_msg.segments = segments[1:]  # All of the useful data is here.

        await self.rx_msg_handler(ros2_msg)

    async def mllp_send_queue_get_handler(self, writer):
        while True:
            outbound_msg = await self.mllp_send_queue.async_q.get()
            self.get_logger().info('mllp send_queue had something')
            await self.mllp_send(outbound_msg, writer)

    async def mllp_recv_handler(self, reader, writer):
        parser_state = self.MLLP_ParserState(self.get_logger())
        while True:
            try:
                self.get_logger().info('waiting on read()...')
                data = await reader.read(1024)
            except asyncio.CancelledError:
                self.get_logger().info('read cancelled')
                raise
            except Exception as ex:
                self.get_logger().error('woah read exception: {}'.format(ex))

            if not data:
                self.get_logger().info('no data. client hung up.')
                writer.close()
                return

            self.get_logger().info('received {} bytes'.format(len(data)))

            # todo: some smarter way to detect and ignore HL7 v3 traffic
            # if data[0] == ord('<'):
            #     self.get_logger.info('this smells like XML. goodbye')
            #     writer.close()
            #     break

            for b in data:
                await parser_state.rx_byte(b, self.rx_msg, writer)

    async def connection_task(self, reader, writer):
        self.get_logger().info('peer connected!')
        # todo: figure out how to set TCP_NODELAY
        mllp_send_task = asyncio.ensure_future(
            self.mllp_send_queue_get_handler(writer))
        mllp_recv_task = asyncio.ensure_future(
            self.mllp_recv_handler(reader, writer))
        _, pending = await asyncio.wait(
            [mllp_send_task, mllp_recv_task],
            return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        self.get_logger().info('exiting connection_task()')

    def main(self):
        self.mllp_spin()

    class MLLP_ParserState():

        class MLLP_ParserStateEnum(Enum):
            SOM = 1  # start of message
            MSG = 2  # in message body
            EOM = 3  # end of message

        def __init__(self, logger):
            self.logger = logger
            self.msg = ""
            self.state = self.MLLP_ParserStateEnum.SOM

        async def rx_byte(self, b, rx_msg_cb, writer):
            # print("state {} rx {}".format(self.state, b))
            if self.state is self.MLLP_ParserStateEnum.SOM:
                if b == 0xb:
                    self.msg = ""
                    self.state = self.MLLP_ParserStateEnum.MSG
                else:
                    self.logger.warn(
                        "OH NO, unexpected char in state SOM: {}".format(b))
            elif self.state is self.MLLP_ParserStateEnum.MSG:
                if b == 0x1c:
                    self.state = self.MLLP_ParserStateEnum.EOM
                    await rx_msg_cb(self.msg, writer)
                else:
                    self.msg += chr(b)
            elif self.state is self.MLLP_ParserStateEnum.EOM:
                if b == 0x0d:
                    self.state = self.MLLP_ParserStateEnum.SOM
                else:
                    self.logger.warn(
                        "OH NO, unexpected char in state EOM: {}".format(b))
            else:
                self.logger.warn("OH NO, invalid parser state")
