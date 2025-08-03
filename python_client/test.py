import asyncio
import platform
from bleak import BleakScanner, BleakClient

async def scan_devices():
    """Enhanced BLE scanner with debug information"""
    print(f"Running on: {platform.system()} {platform.release()}")
    print("Scanning for BLE devices...")
    print("=" * 50)
    
    try:
        # Scan for a longer time with more aggressive settings
        devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
        
        print(f"Found {len(devices)} BLE devices:")
        print()
        
        esp32_devices = []
        
        for device_address, (device, advertisement_data) in devices.items():
            name = device.name or "Unknown"
            rssi = advertisement_data.rssi if hasattr(advertisement_data, 'rssi') else "N/A"
            
            print(f"Device: {name}")
            print(f"  Address: {device_address}")
            print(f"  RSSI: {rssi}")
            
            # Check if this could be our ESP32
            if any(keyword in name.upper() for keyword in ["ESP32", "IMU", "TEST"]):
                esp32_devices.append((device, advertisement_data))
                print("  *** POTENTIAL ESP32 DEVICE ***")
            
            # Print advertisement data
            if hasattr(advertisement_data, 'local_name') and advertisement_data.local_name:
                print(f"  Local Name: {advertisement_data.local_name}")
                
            if hasattr(advertisement_data, 'service_uuids') and advertisement_data.service_uuids:
                print(f"  Service UUIDs: {advertisement_data.service_uuids}")
                
            print()
        
        print("=" * 50)
        
        if esp32_devices:
            print(f"Found {len(esp32_devices)} potential ESP32 devices!")
            for i, (device, _) in enumerate(esp32_devices):
                print(f"{i+1}. {device.name} ({device.address})")
                
            # Try to connect to the first ESP32 device found
            target_device = esp32_devices[0][0]
            print(f"\nAttempting to connect to: {target_device.name} ({target_device.address})")
            
            try:
                async with BleakClient(target_device.address) as client:
                    print(f"Connected: {client.is_connected}")
                    
                    # List all services
                    services = await client.get_services()
                    print(f"Found {len(services.services)} services:")
                    
                    for service in services.services:
                        print(f"  Service: {service.uuid}")
                        for char in service.characteristics:
                            print(f"    Characteristic: {char.uuid} - {char.properties}")
                            
            except Exception as e:
                print(f"Connection failed: {e}")
        else:
            print("No ESP32 devices found!")
            print("\nAll devices found:")
            for device_address, (device, _) in devices.items():
                print(f"  {device.name or 'Unknown'} ({device_address})")
                
    except Exception as e:
        print(f"Scanning failed: {e}")

if __name__ == "__main__":
    asyncio.run(scan_devices())