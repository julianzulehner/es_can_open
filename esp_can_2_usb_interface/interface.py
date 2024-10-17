import serial
import asyncio
import time
import can
import struct 
from typing import Callable, List, Optional, Sequence, Union

from can import (
    BusABC,
    BusState,
    CyclicSendTaskABC,
    Message,
)

TWAI_MESSAGE_SIZE = 20 # size of twai_message_t C struct
TWAI_MAX_DLC = 8 # maximum size of message data

class SerialConnectionError(Exception):
    """Exception raised for errors in the serial connection."""
    def __init__(self, message="Unable to connect to the serial of ESP32 \
                 microcontroller. Please check USB connection."):
        self.message = message
        super().__init__(self.message)

class ESP32Bus(BusABC):
    """The CAN Bus implemented for the ESP32 interface.

    This interface requires an ESP32 controller with external CAN transceiver.
    This code was tested using a ESP32-C3 SUPER MINI development board and the
    SN65HVD230 CAN transceiver.


    TODO: Add here the link for the repository of the ESP code.
    """
    channel_info = "To use configure channel as follows. 'COMXX' for Windows, \
                    '/dev/ttyXXXX' for Linux"
    def __init__(
        self,
        channel: str, 
        can_filters=None, 
        receive_own_messages: bool = False,
        extended: bool = True,
        bitrate: int = 125000,
        serial_bitrate: int = 115200,
        **kwargs):

        """
        :param channel: 
            The port of the device. 'COMX' for Windows, '/dev/ttyX' for Linux.
        
        :param can_filters:
            See :meth: `can.BusABC.set_filters`
        
        :param receive_own_messages:
            Enable self reception of sent messages.
        
        :param extended:
            Default True, enables the capability to use extended IDs.
        
        :param fd:
            Default False, enables CAN-FD usage (CAN FD not supported yet)

        :param bitrate: 
            Bitrate of CAN transceiver in bit/s.

        :param serial_bitrate:
            Bitrate of USB-serial communication ub bit/s.
        """
        super().__init__(channel=channel, **kwargs)
        try:
            self._ser = serial.Serial(channel, 
                                baudrate=serial_bitrate,)
        except serial.SerialException:
            raise SerialConnectionError
        
        self._rx_msg = Message()
        self._tx_msg = Message()
        self._receive_own_messages = receive_own_messages



    def send(self, msg: Message, timeout=None):
        """ Send individual messages """
        if isinstance(msg, Message):
            msg_bytes = self._message_to_bytes(msg)
            if timeout:
                self._ser.timeout = timeout
            self._ser.write(msg_bytes)
        else:
            raise TypeError(f"Argument message must be of type can.Message")


    def _twai_message_from_bytes(self, raw_msg):
        flags, id, dlc = struct.unpack('I I B', raw_msg[:9])
        data = bytes(struct.unpack(f'{dlc}B', raw_msg[9:9+dlc]))
        self._rx_msg.data = data
        self._rx_msg.timestamp = time.time()
        self._rx_msg.arbitration_id = id
        self._rx_msg.dlc = dlc
        self._rx_msg.is_extended_id = flags & (1 << 0) != 0
        self._rx_msg.is_remote_frame = flags & (1 << 1) != 0,
    
    def _message_to_bytes(self, msg: Message):
        """
        Converts can.Message into twai_message_t bytestream. 
        For more details s. https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/twai.html#_CPPv418twai_status_info_t
        """
        flags = 0 
        flags |= msg.is_extended_id << 0
        flags |= msg.is_remote_frame << 1
        #flags |= 1 << 2 # uncomment if single shot message
        flags |= self._receive_own_messages << 3
        #flags |= 1 << 4 # uncomment if dlc > 8 (not ISO 11898-1 compliant)
        data = b''
        for byte in msg.data:
            data += struct.pack('B', byte)
        padding = b'\0\0\0'
        data = data.ljust(TWAI_MAX_DLC, b'\0')
        return (
            struct.pack('I', flags) + 
            struct.pack('I', msg.arbitration_id) +
            struct.pack('B', msg.dlc) +
            data +
            padding
        )

    def _recv_internal(self, timeout):
        """Receive individual messages"""
        is_prefiltered = False
        self._ser.timeout = timeout
        raw_msg = self._ser.read(TWAI_MESSAGE_SIZE)
        if len(raw_msg) == TWAI_MESSAGE_SIZE:
            self._twai_message_from_bytes(raw_msg)
        elif (len(raw_msg) != 0 ):
            print("TO BE IMPLEMENTED")
            return None, is_prefiltered
        else:
            return None, is_prefiltered
        return self._rx_msg,is_prefiltered
    
    def flush_tx_buffer(self):
        raise NotImplementedError("This method is n ot implemented yet.")
    
    def shutdown(self):
        """Stops CAN bus and closes serial connection"""
        raise NotImplementedError("This method is n ot implemented yet.")
    
    # def _apply_filters(self):
    #     """To apply efficient filters on hardware level"""
    #     raise NotImplementedError("This method is n ot implemented yet.")
    
    def state(self):
        """Allows reading and or changing the bus state"""
        raise NotImplementedError("This method is n ot implemented yet.")


    