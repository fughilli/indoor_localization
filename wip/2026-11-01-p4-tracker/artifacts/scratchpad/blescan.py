import asyncio
from bleak import BleakScanner
async def main():
    devs = await BleakScanner.discover(timeout=8.0, return_adv=True)
    for d, adv in devs.values():
        name = adv.local_name or d.name or ""
        if "Widget" in name or "LED" in name.upper() or "splanc" in name.lower():
            print(name, d.address, adv.rssi, adv.service_uuids)
asyncio.run(main())
