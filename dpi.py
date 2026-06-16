from pywinusb import hid


def handler(data):
    print(data)


for device in hid.HidDeviceFilter(
    vendor_id=0x30FA,
    product_id=0x1440
).get_devices():

    try:
        device.open()
        device.set_raw_data_handler(handler)

        print("Opened:", device)

    except Exception as e:
        print(e)

input("Move mouse and press DPI buttons...")
