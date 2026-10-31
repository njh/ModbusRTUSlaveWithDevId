#!/usr/bin/env python3
"""Read device info and temperature from an Arduino Uno running ModbusFakeTemperature.ino.

Opening the Uno's USB serial port resets the board, so the script waits for
the bootloader to finish before sending anything.

Usage:
    ./modbus_read_temperature.py
    ./modbus_read_temperature.py --watch 2
    ./modbus_read_temperature.py --test-individual
"""

import argparse
import inspect
import logging
import sys
import time

from pymodbus.client import ModbusSerialClient
from pymodbus.constants import DeviceInformation, MoreData
from pymodbus.exceptions import ModbusException

TEMPERATURE_REGISTER = 0
RESET_DELAY = 2.0   # seconds for the Uno bootloader to hand over to the sketch

# pymodbus renamed the slave= argument to device_id= in versions after 3.8
UNIT_ARG = ("device_id" if "device_id" in inspect.signature(
    ModbusSerialClient.read_device_information).parameters else "slave")

OBJECT_NAMES = {
    0x00: "VendorName",
    0x01: "ProductCode",
    0x02: "MajorMinorRevision",
    0x03: "VendorUrl",
    0x04: "ProductName",
    0x05: "ModelName",
    0x06: "UserApplicationName",
}

CONFORMITY = {0x01: "basic", 0x02: "regular", 0x03: "extended"}


def read_device_info(client, unit):
    """Read every object, following More Follows across responses."""
    objects, object_id = {}, 0x00
    while True:
        result = client.read_device_information(
            read_code=DeviceInformation.EXTENDED, object_id=object_id, **{UNIT_ARG: unit})
        if result.isError():
            sys.exit(f"Read device information failed: {result}")
        objects.update(result.information)
        if result.more_follows != MoreData.KEEP_READING:
            return result.conformity, objects
        object_id = result.next_object_id


def test_individual_access(client, unit, objects):
    """Check code 04 returns each object alone, and exception 02 for an unknown ID.

    Returns the number of failures.
    """
    failures = 0

    def check(ok, message):
        nonlocal failures
        print(f"  {'ok  ' if ok else 'FAIL'} {message}")
        failures += not ok

    print("Individual access (code 04):")
    for object_id, value in sorted(objects.items()):
        result = client.read_device_information(
            read_code=DeviceInformation.SPECIFIC, object_id=object_id, **{UNIT_ARG: unit})
        if result.isError():
            check(False, f"0x{object_id:02X}: {result}")
            continue
        check(result.information == {object_id: value}
              and result.more_follows == MoreData.NOTHING,
              f"0x{object_id:02X} returned {result.information}")

    unknown_id = next(i for i in range(0x100) if i not in objects)
    pymodbus_log = logging.getLogger("pymodbus.logging")
    pymodbus_log.disabled = True   # an exception response is expected; don't log it
    try:
        result = client.read_device_information(
            read_code=DeviceInformation.SPECIFIC, object_id=unknown_id, **{UNIT_ARG: unit})
    finally:
        pymodbus_log.disabled = False
    check(result.isError() and result.exception_code == 2,
          f"0x{unknown_id:02X} (not in table) gave exception "
          f"{getattr(result, 'exception_code', None)}, expected 2")
    return failures


def read_temperature(client, unit):
    result = client.read_input_registers(TEMPERATURE_REGISTER, count=1, **{UNIT_ARG: unit})
    if result.isError():
        sys.exit(f"Read temperature failed: {result}")
    raw = result.registers[0]
    if raw & 0x8000:   # int16, tenths of a degree C
        raw -= 0x10000
    return raw / 10


def print_device_info(conformity, objects):
    level = CONFORMITY.get(conformity & 0x7F, "unknown")
    access = ", individual access" if conformity & 0x80 else ""
    print(f"Conformity level: 0x{conformity:02X} ({level}{access})")
    for object_id, value in sorted(objects.items()):
        name = OBJECT_NAMES.get(object_id, f"Private 0x{object_id:02X}")
        print(f"  {name:20} {value.decode('ascii', errors='replace')}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--unit", type=int, default=1)
    parser.add_argument("--watch", type=float, metavar="SECONDS",
                        help="keep reading the temperature at this interval")
    parser.add_argument("--test-individual", action="store_true",
                        help="also read each object individually (code 04) and check it")
    args = parser.parse_args()

    client = ModbusSerialClient(args.port, baudrate=args.baud, timeout=1)
    if not client.connect():
        sys.exit(f"Could not open {args.port}")

    try:
        time.sleep(RESET_DELAY)

        conformity, objects = read_device_info(client, args.unit)
        print_device_info(conformity, objects)
        if args.test_individual and test_individual_access(client, args.unit, objects):
            sys.exit("Individual access test failed")
        while True:
            print(f"Temperature: {read_temperature(client, args.unit):.1f} °C", flush=True)
            if not args.watch:
                break
            time.sleep(args.watch)
    except ModbusException as e:
        sys.exit(f"Modbus error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
