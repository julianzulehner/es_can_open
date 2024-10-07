import can
import time

FILTER_1_IDS = [0x0, 0x80]
SDO_IDS = list(range(0x600, 0x6FF))
HEARTBEAT_IDS = list(range(0x700, 0x7FF))
LSS_IDS = [0x7E5]
FILTER_2_IDS = SDO_IDS + HEARTBEAT_IDS + LSS_IDS 
ACCEPTED_IDS = FILTER_1_IDS + FILTER_2_IDS

ALL_IDS = list(range(0x0, 0x800))

bus = can.Bus(channel=0, interface="ixxat", bitrate=125000, receive_own_messages=False)

for id in ALL_IDS:
    msg = can.Message(arbitration_id=id, is_extended_id = False)
    bus.send(msg)
    time.sleep(0.01)


bus.shutdown()


