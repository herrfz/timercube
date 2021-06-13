import asyncio
import serial
from serial.serialutil import SerialException


async def read_data():
    try:
        with serial.Serial('COM4', baudrate=115200, timeout=0) as ser:
            while True:
                res = ser.readline().decode()
                print(res, end='')
                await asyncio.sleep(1)
    except SerialException:
        print('Connection problem, retrying..')


if __name__ == '__main__':
    try:
        asyncio.run(read_data())
    except KeyboardInterrupt:
        print('Bye.')