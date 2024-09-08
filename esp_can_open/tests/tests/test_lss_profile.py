import can 
import time

# DRYER DRAIN
# NODE_ID = 74
# VENDOR_ID = 1253
# PRODUCT_CODE = 1625810030
# REVISION_NUMBER = 808517632
# SERIAL_NUMBER = 1343650048

# AFTERCOOLER DRAIN
# NODE_ID = 73
# VENDOR_ID = 1253
# PRODUCT_CODE = 1625810030
# REVISION_NUMBER = 808517632
# SERIAL_NUMBER = 2316005719

# PRESSURE SENSOR
# CURRENT_NODE_ID = 127
# NEW_NODE_ID = 9
# VENDOR_ID = 16777832
# PRODUCT_CODE = 1625810060
# REVISION_NUMBER = 808517632
# SERIAL_NUMBER = 3171506

# TEMPERATURE SENSOR
# CURRENT_NODE_ID = 47
# NEW_NODE_ID = 127
# VENDOR_ID = 1280
# PRODUCT_CODE = 1625810019
# REVISION_NUMBER = 808517632
# SERIAL_NUMBER = 107418

# TRINAMICS STC
# CURRENT_NODE_ID = 6
# NEW_NODE_ID = 6
# VENDOR_ID = 646
# PRODUCT_CODE = 1273
# REVISION_NUMBER = 196634
# SERIAL_NUMBER = 0

# DELTA LINE STC
# CURRENT_NODE_ID = 6
# NEW_NODE_ID = 6
# VENDOR_ID = 1396
# PRODUCT_CODE = 123076641
# REVISION_NUMBER = 0
# SERIAL_NUMBER = 37355783

# OPS3 CANopen
CURRENT_NODE_ID = 127
NEW_NODE_ID = 127
VENDOR_ID = 0
PRODUCT_CODE = 1111111111
REVISION_NUMBER = 808517632
SERIAL_NUMBER = 1

RECEIVE_OWN_MESSAGES = True
UNKNOWN = can.Message(arbitration_id=2021, is_extended_id=False, dlc=1, data=[94])
GLOBAL_SWITCH_CONFIGURATION_MODE = can.Message(arbitration_id=2021, is_extended_id=False, data=[0x4, 0x1]) # switch to configuration mode
INQUIRE_VENDOR_ID = can.Message(arbitration_id=2021, is_extended_id=False, data=[90])
INQUIRE_PRODUCT_CODE = can.Message(arbitration_id=2021, is_extended_id=False, data=[91])
INQUIRE_REVISION_NUMBER = can.Message(arbitration_id=2021, is_extended_id=False, data=[92])
INQUIRE_SERIAL_NUMBER = can.Message(arbitration_id=2021, is_extended_id=False, data=[93])
SWITCH_MODE_SELECTIVE_VENDOR_ID = can.Message(arbitration_id=2021, is_extended_id=False, data= bytes([64]) + VENDOR_ID.to_bytes(length=4, byteorder="little"), dlc=5)
SWITCH_MODE_SELECTIVE_PRODUCT_CODE = can.Message(arbitration_id=2021,  is_extended_id=False,data= bytes([65]) + PRODUCT_CODE.to_bytes(length=4, byteorder="little"), dlc=5)
SWITCH_MODE_SELECTIVE_REVISION_NUMBER = can.Message(arbitration_id=2021, is_extended_id=False, data= bytes([66]) + REVISION_NUMBER.to_bytes(length=4, byteorder="little"), dlc=5)
SWITCH_MODE_SELECTIVE_SERIAL_NUMBER = can.Message(arbitration_id=2021,  is_extended_id=False, data= bytes([67]) + SERIAL_NUMBER.to_bytes(length=4, byteorder="little"), dlc=5)
SWITCH_NODE_ID = can.Message(arbitration_id=2021, is_extended_id=False, dlc=8, data=[17 ,NEW_NODE_ID])
STORE_CONFIGURATION = can.Message(arbitration_id=2021, is_extended_id=False, dlc=8, data=[23])
RESET_NMT = can.Message(arbitration_id=0, is_extended_id=False, data=[129, 0] )
SDO_VENDOR_ID = can.Message(arbitration_id = 0x600 + CURRENT_NODE_ID, is_extended_id=False, data=[0x40, 0x18, 0x10, 0x1])
SDO_PRODUCT_CODE = can.Message(arbitration_id = 0x600 + CURRENT_NODE_ID, is_extended_id=False, data=[0x40, 0x18, 0x10, 0x2])
SDO_REVISION_NUMBER = can.Message(arbitration_id = 0x600 + CURRENT_NODE_ID, is_extended_id=False, data=[0x40, 0x18, 0x10, 0x3])
SDO_SERIAL_NUMBER = can.Message(arbitration_id = 0x600 + CURRENT_NODE_ID, is_extended_id=False, data=[0x40, 0x18, 0x10, 0x4])

def custom_receive(bus):
    if RECEIVE_OWN_MESSAGES:
        response = bus.recv(timeout=1)
    response = bus.recv(timeout=1)
    return response 

def read_lss_data(bus):
    bus.send(SDO_VENDOR_ID)
    response = custom_receive(bus)

    if response:
        VENDOR_ID = int.from_bytes(response.data[4:8],"little")
        print(f"INFO: VENDOR ID    {VENDOR_ID}")
    else:
        print("ERROR: NO RESPONSE FROM NODE {NODE_ID}")
        return
    bus.send(SDO_PRODUCT_CODE)
    response = custom_receive(bus)
    if response:
        PRODUCT_CODE = int.from_bytes(response.data[4:8],"little")
        print(f"INFO: PRODUCT CODE    {PRODUCT_CODE}")
    else:
        print("ERROR: NO RESPONSE FROM NODE {NODE_ID}")
        return
    bus.send(SDO_REVISION_NUMBER)
    response = custom_receive(bus)
    if response:
        REVISION_NUMBER = int.from_bytes(response.data[4:8],"little")
        print(f"INFO: REVISION NUMBER    {REVISION_NUMBER}")
    else:
        print("ERROR: NO RESPONSE FROM NODE {NODE_ID}")
        return
    bus.send(SDO_SERIAL_NUMBER)
    response = custom_receive(bus)
    if response:
        SERIAL_NUMBER = int.from_bytes(response.data[4:8],"little")
        print(f"INFO: SERIAL NUMBER    {SERIAL_NUMBER}")
    else:
        print("ERROR: NO RESPONSE FROM NODE {NODE_ID}")
        return

    
def switch_selective_mode(bus):
    #bus.send(UNKNOWN)
    bus.send(SWITCH_MODE_SELECTIVE_VENDOR_ID)
    if RECEIVE_OWN_MESSAGES:
        bus.recv(1)
    bus.send(SWITCH_MODE_SELECTIVE_PRODUCT_CODE)
    if RECEIVE_OWN_MESSAGES:
        bus.recv(1)
    bus.send(SWITCH_MODE_SELECTIVE_REVISION_NUMBER)
    if RECEIVE_OWN_MESSAGES:
        bus.recv(1)
    bus.send(SWITCH_MODE_SELECTIVE_SERIAL_NUMBER)
    if RECEIVE_OWN_MESSAGES:
        bus.recv(1)
    response = bus.recv(1)
    if response:
        if response.data[0] == 68:
            match response.data[1]:
                case 0:
                    print("INFO: DEVICE IS IN OPERATION MODE")
                case 1:
                    print("INFO: DEVICE IS IN CONFIGURATION MODE")
        else:
            print(f"ERROR: WRONG RESPONSE '{response.data[0]}' OF DEVICE. EXPECTED '68'.")
    else:
        print("ERROR: TIMEOUT - NO RESPONSE FROM DEVICE AFTER SWITCH SELECTIVE")

def switch_node_id(bus):
    bus.send(SWITCH_NODE_ID)
    response = None
    response = custom_receive(bus)
    if response:
        if response.data[0] == 17:
            match response.data[1]:
                case 0:
                    print("INFO: NODE ID WAS SUCCESSFULLY SET")
                case 1:
                    print("ERROR: NODE ID OUT OF RANGE")
                case _:
                    print("ERROR: RESERVED ERROR")
        else:
            print(f"ERROR: WRONG RESPONSE '{response.data[0]}' OF DEVICE. EXPECTED '17'.")
    else:
        print("ERROR: TIMEOUT - NO RESPONSE FROM DEVICE AFTER SWITCH NODE ID")

def store_configuration(bus):
    bus.send(STORE_CONFIGURATION)
    response = None
    response = custom_receive(bus)
    if response:
        if response.data[0] == 23:
            match response.data[1]:
                case 0:
                    print("INFO: CONFIGURATION SUCCESSFULLY STORED ON DEVICE")
                case 1:
                    print("ERROR: STORE CONFIGURATION IS NOT SUPPORTED")
                case 2:
                    print("ERROR: STORAGE MEDIA ACCESS ERROR")
                case _:
                    print("ERROR: RESERVED ERROR")
        else:
            print(f"ERROR: WRONG RESPONSE '{response.data[0]}' OF DEVICE. EXPECTED '23'.")
    else:
        print("ERROR: TIMEOUT - NO RESPONSE FROM DEVICE AFTER STORE CONFIGURATION")

def inquire_lss_data(bus):
    bus.send(INQUIRE_VENDOR_ID)
    response = custom_receive(bus)
    print(response)

def main():
    bus = can.Bus(channel=0, interface="ixxat", bitrate=125000, receive_own_messages=RECEIVE_OWN_MESSAGES)
    read_lss_data(bus)
    switch_selective_mode(bus)
    switch_node_id(bus)
    store_configuration(bus)
    bus.send(RESET_NMT)
    time.sleep(5)

    bus.shutdown()
    

if __name__ == "__main__":
    main()