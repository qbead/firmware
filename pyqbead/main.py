import asyncio
from bleak import BleakScanner, BleakClient
import bleak
import bleak.backends
import bleak.backends.device
import bleak.backends.service

serviceUuid = "e30c1fc6-359c-12be-2544-63d6aa088d45";

accUuid = "e30c1fc9-359c-12be-2544-63d6aa088d45";
sphUuid = "e30c1fc8-359c-12be-2544-63d6aa088d45";
colUuid = "e30c1fc7-359c-12be-2544-63d6aa088d45";

async def discover():
    devices = await BleakScanner.discover(return_adv=True)
    for device, adv in devices.values():
        if device.name is not None and "qbead" in device.name:
            print(f'Found device: {device}, RSSI: {adv.rssi}')
    return devices

def selectClosestDevice(devices):
    max_rssi = 0
    closest_device = None
    for device,adv in devices.values():
        if device.name is not None and "qbead" in device.name and adv.rssi < max_rssi:
            max_rssi = adv.rssi
            closest_device = device
    return closest_device

async def ConnectToDevice(device):
    print(f'Connecting to device: {device}')
    assert type(device) is bleak.backends.device.BLEDevice
    address = device.address
    async with BleakClient(address) as client:
        print(f'Connected to device: {device}')
        services = client.services
        print("Services:")
        for service in services:
            print(service)
        service = services.get_service(serviceUuid)
        print('Characteristics:')
        for characteristic in service.characteristics:
            print(characteristic)
        col = service.get_characteristic(colUuid)
        color = await client.read_gatt_char(col)
        print(f'Color: {color}')
        await client.write_gatt_char(col, bytearray([255, 255, 255]))
        color = await client.read_gatt_char(col)
        print(f'Color: {color}')

async def main():
    devices = await discover()
    closest_device = selectClosestDevice(devices)
    print(f'Closest device: {closest_device}')
    await ConnectToDevice(closest_device)

asyncio.run(main())

