import asyncio
from bleak import BleakScanner, BleakClient
import bleak
import bleak.backends
import bleak.backends.device
import bleak.backends.service



import lib.qbeadBLE as qble

serviceUuid = "e30c1fc6-359c-12be-2544-63d6aa088d45";

accUuid = "e30c1fc9-359c-12be-2544-63d6aa088d45";
sphUuid = "e30c1fc8-359c-12be-2544-63d6aa088d45";
colUuid = "e30c1fc7-359c-12be-2544-63d6aa088d45";

#add as many arguments as you like. just match it to the connectToDevice call below
async def testBLE(client:BleakClient,one, two, three=None):
        qble.printServices(client)
        qble.printCharacteristics(client)
        color = await qble.readColor(client)
        print(f'Color: {color}')
        await qble.writeColor(client, 255, 0, 0)
        color = await qble.readColor(client)
        print(f'Color: {color}')
        coords = await qble.readAcceleration(client)
        print(f'Acceleration: {coords}')
        theta,phi = await qble.readSphericalCoordinates(client)
        print(f'Spherical coordinates: {theta}, {phi}')
        await qble.setAccelerationNotification(client, AccelerationNotificationCallback)
        while True:
            await asyncio.sleep(1)
        

async def AccelerationNotificationCallback(sender, data):
    unpacked = qble.unpackAccelerationData(data)
    print(f'Acceleration: {unpacked}')

async def main():
    devices = await qble.discover()
    closest_device = qble.selectClosestDevice(devices)
    print(f'Closest device: {closest_device}')
    await qble.ConnectToDevice(closest_device,testBLE, "one", "two", three="three")

asyncio.run(main())

