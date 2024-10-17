from interface import ESP32Bus
import asyncio
import time

INTERVALL = 0.005 # in seconds
# class TwaiMessage:
#     def __init__(self, flags, identifier, data_length_code, data):
#         self.flags = flags
#         self.identifier = identifier
#         self.data_length_code = data_length_code
#         self.data = data 

#     @classmethod
#     def from_bytes(cls, data):
#         flags, identifier, data_length_code = struct.unpack('I I B', data[:9])
#         data = struct.unpack(f'{data_length_code}B', data[9:9+data_length_code])
#         return cls(flags, identifier, data_length_code, data)
    
#ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0.1)
bus = ESP32Bus(channel="/dev/ttyACM0", extended=False, bitrate=125000)


async def read():
    while True:

        msg = bus.recv(timeout=INTERVALL)
        print(msg)
        #res = ser.read(20)
        #if res:
            #msg = TwaiMessage.from_bytes(res)
            #print(f"Flags: {msg.flags}, Identifier: {msg.identifier}, DLC: {msg.data_length_code}, Data: {msg.data}")


        await asyncio.sleep(INTERVALL)

async def write():
    while True:
        msg = b"Test"
        #ser.write(msg)
        bus._ser.write(msg)

        #print("Message Sent")
        await asyncio.sleep(INTERVALL)

async def main():
    write_task = asyncio.create_task(write())
    read_task = asyncio.create_task(read())
    print(f"Started at {time.strftime('%X')}")
    await asyncio.gather(write_task, read_task)


if __name__  ==  "__main__":
    asyncio.run(main())
