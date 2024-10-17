from interface import ESP32Bus
import asyncio
import time
from can import Message


INTERVALL = 0.1 # in seconds
bus = ESP32Bus(channel="/dev/ttyACM0", extended=False, bitrate=125000, receive_own_messages=True)


async def read():
    while True:

        msg = bus.recv(timeout=INTERVALL)
        if msg:
            print(msg)

        await asyncio.sleep(INTERVALL)

async def write():
    while True:
        msg = Message(arbitration_id = 0x80, is_extended_id=False, is_remote_frame=False, 
                      data=[1,2,3,4,5,6,7], dlc=7)
        bus.send(msg, 1)
        await asyncio.sleep(3)

async def main():
    write_task = asyncio.create_task(write())
    read_task = asyncio.create_task(read())
    print(f"Started at {time.strftime('%X')}")
    await asyncio.gather(write_task, read_task)


if __name__  ==  "__main__":
    asyncio.run(main())
