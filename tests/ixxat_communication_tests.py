"""
Before running this test, one must connect the ESP32 with the installed firmware
via USB to the computer. Also the ixxat device must be connected to the CAN bus 
and via USB to the computer. 

The ixxat interface sometimes causes issues when using the python library. A 
workaround is to open canAnalyser3mini and establish a connection also from 
there. Then the driver is set up properly.

Then press the reset button of the ESP. On CAN analyser 3 mini you will se some 
error messages. This is due to the ESP CAN controller that is corrupting the
bus during initialization. I haven't found a solution for that yet. The data of
the last message however should be 7F, which is the pre-operational flag.

If this is setup and you can see the last message as described, start the test.
"""

import can 
import unittest 
import time
import pytest

NODE_ID = 127
NMT_RESET_NODE = 0x81
NMT_OPERATIONAL = 0x01
NMT_PRE_OPERATIONAL = 0x80

STATE_OPERATIONAL = 0x05
STATE_PRE_OPERATIONAL = 0x7F

VENDOR_ID = 0
PRODUCT_CODE = 1111111111
REVISION_NUMBER = 0x3031 << 16
SERIAL_NUMBER = 1

bus = None 
tx_msg = can.Message(is_extended_id = False, is_remote_frame=False, is_error_frame=False)


class TestCANopenMethods(unittest.TestCase):

    def setUp(self):
        global bus
        bus = can.Bus(channel=0, interface="ixxat", bitrate=125000, receive_own_messages=False)

    def test_set_operational(self):
        tx_msg.arbitration_id = 0x0
        tx_msg.dlc = 2
        tx_msg.data = bytearray([NMT_OPERATIONAL, NODE_ID])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], STATE_OPERATIONAL)

    def test_set_pre_operational(self):
        tx_msg.arbitration_id = 0x0
        tx_msg.dlc = 2
        tx_msg.data = bytearray([NMT_PRE_OPERATIONAL, NODE_ID])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], STATE_PRE_OPERATIONAL)

    def test_node_guarding(self):
        # Set operational
        tx_msg.arbitration_id = 0x0
        tx_msg.dlc = 2
        tx_msg.data = bytearray([NMT_OPERATIONAL, NODE_ID])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], STATE_OPERATIONAL)

        # Define node guarding message
        tx_msg.arbitration_id = 0x700 + NODE_ID
        tx_msg.dlc = 0
        tx_msg.data = bytearray()
        status = 0x5
        for i in range(10):
            bus.send(tx_msg)
            rx_msg = bus.recv(timeout=1)
            status ^= 1<<7
            self.assertEqual(rx_msg.data[0], status)

    def test_sdo_request_0x1018(self):
        # Prepare message
        tx_msg.arbitration_id = 0x600 + NODE_ID
        tx_msg.dlc = 8

        # Request VENDOR_ID
        tx_msg.data = bytearray([0x40, 0x18, 0x10, 0x1])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], 0x43)
        self.assertEqual(int.from_bytes(tx_msg.data[1:3], "little"), 0x1018)
        self.assertEqual(rx_msg.data[3], 0x1)
        self.assertEqual(int.from_bytes(rx_msg.data[4:8], "little"), VENDOR_ID)
        self.assertEqual(rx_msg.arbitration_id, 0x580+NODE_ID)

        # Request PRDOCUT_CODE
        tx_msg.data = bytearray([0x40, 0x18, 0x10, 0x2])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], 0x43)
        self.assertEqual(int.from_bytes(tx_msg.data[1:3], "little"), 0x1018)
        self.assertEqual(rx_msg.data[3], 0x2)
        self.assertEqual(int.from_bytes(rx_msg.data[4:8], "little"), PRODUCT_CODE)
        self.assertEqual(rx_msg.arbitration_id, 0x580+NODE_ID)

        # Request REVISION_NUMBER
        tx_msg.data = bytearray([0x40, 0x18, 0x10, 0x3])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], 0x43)
        self.assertEqual(int.from_bytes(tx_msg.data[1:3], "little"), 0x1018)
        self.assertEqual(rx_msg.data[3], 0x3)
        self.assertEqual(int.from_bytes(rx_msg.data[4:8], "little"), REVISION_NUMBER)
        self.assertEqual(rx_msg.arbitration_id, 0x580+NODE_ID)

        # Request SERIAL_NUMBER
        tx_msg.data = bytearray([0x40, 0x18, 0x10, 0x4])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], 0x43)
        self.assertEqual(int.from_bytes(tx_msg.data[1:3], "little"), 0x1018)
        self.assertEqual(rx_msg.data[3], 0x4)
        self.assertEqual(int.from_bytes(rx_msg.data[4:8], "little"), SERIAL_NUMBER)
        self.assertEqual(rx_msg.arbitration_id, 0x580+NODE_ID)

    def test_sdo_write(self):
        # Writing new value to object 0x1800 subindex 2
        tx_msg.arbitration_id = 0x600 + NODE_ID
        tx_msg.dlc = 8
        tx_msg.data = bytearray([0x2B, 0x00, 0x18, 0x2, 0x2, 0x0, 0x0, 0x0])
        bus.send(tx_msg, timeout=1)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.arbitration_id, 0x580 + NODE_ID)
        self.assertEqual(rx_msg.data[0], 0x60)
        self.assertEqual(rx_msg.data[1], 0x00)
        self.assertEqual(rx_msg.data[2], 0x18)
        self.assertEqual(rx_msg.data[3], 0x2)
        self.assertEqual(int.from_bytes(rx_msg.data[4:8], 'little'), 0)

        # Reading value that was changed
        tx_msg.arbitration_id = 0x600 + NODE_ID
        tx_msg.dlc = 8
        tx_msg.data = bytearray([0x40, 0x00, 0x18, 0x2])
        bus.send(tx_msg, timeout=1)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.arbitration_id, 0x580 + NODE_ID)
        self.assertEqual(rx_msg.data[0], 0x43)
        self.assertEqual(rx_msg.data[1], 0x00)
        self.assertEqual(rx_msg.data[2], 0x18)
        self.assertEqual(rx_msg.data[3], 0x2)
        self.assertEqual(int.from_bytes(rx_msg.data[4:8], 'little'), 2)

    def test_tpdo_service(self):
        # Set operational
        tx_msg.arbitration_id = 0x0
        tx_msg.dlc = 2
        tx_msg.data = bytearray([NMT_OPERATIONAL, NODE_ID])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(rx_msg.data[0], STATE_OPERATIONAL)

        tx_msg.arbitration_id = 0x80
        tx_msg.dlc = 0
        tx_msg.data = bytearray([])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        self.assertEqual(int.from_bytes(rx_msg.data[0:4],'little'), 2**16-1)
        self.assertEqual(rx_msg.arbitration_id, 0x180+NODE_ID)

    def test_lss_change_node(self):
        pass

    def test_nmt_reset(self):
        time.sleep(.1)
        tx_msg.arbitration_id = 0x0
        tx_msg.dlc = 2
        tx_msg.data = bytearray([NMT_RESET_NODE, NODE_ID])
        bus.send(tx_msg)
        rx_msg = bus.recv(timeout=1)
        # Node should automatically be in pre-operative mode
        self.assertEqual(int.from_bytes(rx_msg.data), STATE_PRE_OPERATIONAL)


    def tearDown(self):
        bus.shutdown()




if __name__ ==  "__main__":
    unittest.main()
