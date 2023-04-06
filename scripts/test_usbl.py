#!/usr/bin/env python3

"""
Demonstration of interaction with the USBL devices.
"""

from delivery.usbl import USBL
from delivery.script_utils import usbl_info, usbl_setup, usbl_ping

#
# To find the /dev file for a specific seatrac:
# ls -l /dev/serial/by-id/usb-FTDI_USB-RS232_Cable*
#
X_ONE_FIFTY_SERIAL = 28244  # FT76BBVB
X_ONE_TEN_SERIAL = 28253    # AU04GX3Z
BEACON_ID = {
    X_ONE_FIFTY_SERIAL: 15,
    X_ONE_TEN_SERIAL: 1
}

usbl = USBL('Unknown', '/dev/ttyUSB0', 115200)

try:
    device_info = usbl_info(usbl)
    print(device_info)

    serial_number = device_info.hardware['serial_number']
    beacon_id = BEACON_ID[serial_number]

except Exception as e:
    print(f"usbl_info failed: {e}")

try:
    usbl_setup(usbl=usbl, beacon_id=beacon_id)

except Exception as e:
    print(f"usbl_setup failed: {e}")

try:
    if device_info.hardware['serial_number'] == X_ONE_FIFTY_SERIAL:
        print("PING-ing the X110")

        for pings in range(2):
            usbl_status, beacon_fix = usbl_ping(
                usbl=usbl,
                beacon_id=BEACON_ID[X_ONE_TEN_SERIAL])
            print("\nX110 position")
            # print(usbl_status)
            print(beacon_fix)

except Exception as e:
    print(f"usbl_ping failed: {e}")

usbl._disconnect()
