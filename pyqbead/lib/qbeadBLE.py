import asyncio
from bleak import BleakScanner, BleakClient
import bleak
import bleak.backends
import bleak.backends.device
import bleak.backends.service
import lib.qbeadBLE as qble
import struct
from typing import Callable

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

async def ConnectToDevice(device, blefun:Callable[[BleakClient],None],*args, **kwargs):
    print(f'Connecting to device: {device}')
    assert type(device) is bleak.backends.device.BLEDevice
    address = device.address
    async with BleakClient(address) as client:
        print(f'Connected to device: {device}')
        await blefun(client,*args, **kwargs)

def listServices(client:BleakClient):
    services = client.services
    return services

def printServices(client:BleakClient):
    services = client.services
    print("Services:")
    for service in services:
        print(service)

def listCharacteristics(client:BleakClient, uuid=None):
    if uuid is None:
        uuid = serviceUuid
    services = client.services
    service = services.get_service(uuid)
    return service.characteristics

def printCharacteristics(client:BleakClient, uuid=None):
    characteristics = listCharacteristics(client, uuid)
    print('Characteristics:')
    for characteristic in characteristics:
        print(characteristic)

async def readColor(client:BleakClient):
    services  = client.services
    col = services.get_characteristic(colUuid)
    color = await client.read_gatt_char(col)
    return list(color)

async def writeColor(client:BleakClient, r, g, b):
    assert r >= 0 and r <= 255
    assert g >= 0 and g <= 255
    assert b >= 0 and b <= 255
    
    services  = client.services
    colservice = services.get_characteristic(colUuid)
    await client.write_gatt_char(colservice, bytearray([r, g, b]))

async def readAcceleration(client:BleakClient):
    services  = client.services
    acc = services.get_characteristic(accUuid)
    accdata = await client.read_gatt_char(acc)
    return unpackAccelerationData(accdata)
    
def unpackAccelerationData(accdata):
    assert len(accdata) == 12
    #seperate into 3 floats
    x = struct.unpack('<f', accdata[0:4])
    y = struct.unpack('<f', accdata[4:8])
    z = struct.unpack('<f', accdata[8:12])
    return x,y,z

async def setAccelerationNotification(client:BleakClient, fun:Callable[[BleakClient,bytearray],None]):
    services  = client.services
    acc = services.get_characteristic(accUuid)
    await client.start_notify(acc, fun)

async def readSphericalCoordinates(client:BleakClient):
    services  = client.services
    sph = services.get_characteristic(sphUuid)
    sphdata = await client.read_gatt_char(sph)
    return unpackSphericalData(sphdata)

def unpackSphericalData(sphdata):
    #seperate bytearray into uint8:
    assert len(sphdata) == 2
    thetha = sphdata[0]
    phi = sphdata[1]
    return thetha, phi
    


