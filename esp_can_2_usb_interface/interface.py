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



    def send(self):
        """Send individual messages"""
        raise NotImplementedError("This method is n ot implemented yet.")

    
    def _recv_internal(self, timeout):
        """Receive individual messages"""
        is_prefiltered = False
        self._ser.timeout = timeout
        raw_msg = self._ser.read(TWAI_MESSAGE_SIZE)
        if raw_msg:
            flags, id, dlc = struct.unpack('I I B', raw_msg[:9])
            data = struct.unpack(f'{dlc}B', raw_msg[9:9+dlc])
        else:
            return None, is_prefiltered
        return Message(
            timestamp = time.time(),
            arbitration_id = id,
            dlc=dlc,
            data = data,
            is_extended_id = flags & (1 << 0) != 0, 
            is_remote_frame = flags & (1 << 1) != 0,
        ), is_prefiltered
    
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


    