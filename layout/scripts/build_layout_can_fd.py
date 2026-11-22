#!/usr/bin/env python3
"""
Physical Design Layout & GDSII Generator for ISO 11898-1:2015 CAN-FD Controller ASIC
Generates full-chip layout stream format (GDSII stream version 7) including:
- Die Boundary (1350 um x 1350 um)
- Power Ring (VDD / VSS on Metal 4 & Metal 5)
- Standard Cell Placement Rows
- Sub-block modules (CAN-FD FSM, 64-Byte Message FIFO, CRC-17/21, Bit Timing, TDC Engine)
- 44 I/O Wirebond Pads
"""

import struct
import os
import sys

def gds_record(record_type, data_type, data=b""):
    length = len(data) + 4
    header = struct.pack(">HH", length, (record_type << 8) | data_type)
    return header + data

def write_gds(filename):
    out = bytearray()

    # HEADER (record 0x00, type 0x02)
    out.extend(gds_record(0x00, 0x02, struct.pack(">H", 600)))

    # BGNLIB (record 0x01, type 0x02)
    out.extend(gds_record(0x01, 0x02, struct.pack(">12H", 2026, 11, 22, 12, 0, 0, 2026, 11, 22, 12, 0, 0)))

    # LIBNAME (record 0x02, type 0x06)
    lib_name = b"CAN_FD_CONTROLLER_LIB\x00"
    if len(lib_name) % 2 != 0: lib_name += b"\x00"
    out.extend(gds_record(0x02, 0x06, lib_name))

    # UNITS (record 0x03, type 0x05)
    out.extend(gds_record(0x03, 0x05, struct.pack(">dd", 0.001, 1e-9)))

    # BGNSTR (record 0x05, type 0x02)
    out.extend(gds_record(0x05, 0x02, struct.pack(">12H", 2026, 11, 22, 12, 0, 0, 2026, 11, 22, 12, 0, 0)))

    # STRNAME (record 0x06, type 0x06)
    str_name = b"can_fd_controller_top\x00"
    if len(str_name) % 2 != 0: str_name += b"\x00"
    out.extend(gds_record(0x06, 0x06, str_name))

    def add_box(layer, datatype, x1, y1, x2, y2):
        b = bytearray()
        b.extend(gds_record(0x08, 0x00)) # BOUNDARY
        b.extend(gds_record(0x0D, 0x02, struct.pack(">H", layer)))
        b.extend(gds_record(0x0E, 0x02, struct.pack(">H", datatype)))
        coords = struct.pack(">10i", x1, y1, x2, y1, x2, y2, x1, y2, x1, y1)
        b.extend(gds_record(0x10, 0x03, coords))
        b.extend(gds_record(0x11, 0x00)) # ENDEL
        return b

    # 1. Die Boundary (Layer 100): 1350 um x 1350 um (1350000 nm x 1350000 nm)
    out.extend(add_box(100, 0, 0, 0, 1350000, 1350000))

    # 2. Core Area (Layer 108): 100 um margin -> 100,000 to 1,250,000
    out.extend(add_box(108, 0, 100000, 100000, 1250000, 1250000))

    # 3. Power Rings (Metal 4: Layer 31, Metal 5: Layer 32)
    # VDD Ring
    out.extend(add_box(31, 0, 90000, 90000, 1260000, 106000))
    out.extend(add_box(31, 0, 90000, 1244000, 1260000, 1260000))
    out.extend(add_box(32, 0, 90000, 90000, 106000, 1260000))
    out.extend(add_box(32, 0, 1244000, 90000, 1260000, 1260000))

    # VSS Ring
    out.extend(add_box(31, 1, 112000, 112000, 1238000, 128000))
    out.extend(add_box(31, 1, 112000, 1222000, 1238000, 1238000))
    out.extend(add_box(32, 1, 112000, 112000, 128000, 1238000))
    out.extend(add_box(32, 1, 1222000, 112000, 1238000, 1238000))

    # 4. Functional Subsystem Floorplanning Blocks (Layer 40)
    # 64-Byte Message FIFO Subsystem (North: 500 um x 350 um)
    out.extend(add_box(40, 1, 420000, 850000, 920000, 1200000))
    # CAN-FD Protocol FSM (Center: 400 um x 350 um)
    out.extend(add_box(40, 2, 470000, 450000, 870000, 800000))
    # Dual-Rate Bit Timing & Prescaler (West: 250 um x 300 um)
    out.extend(add_box(40, 3, 140000, 480000, 390000, 780000))
    # Multi-Polynomial CRC Engine (East: 280 um x 300 um)
    out.extend(add_box(40, 4, 930000, 480000, 1210000, 780000))
    # Acceptance Filter, TDC & Fault Confinement (South: 550 um x 250 um)
    out.extend(add_box(40, 5, 400000, 150000, 950000, 400000))

    # 5. Standard Cell Placement Rows (Metal 1: Layer 11)
    for y in range(150000, 1200000, 8000):
        out.extend(add_box(11, 0, 140000, y, 1210000, y + 4000))

    # 6. I/O Wirebond Pads (Layer 70: 70 um x 70 um)
    # 11 pads per side = 44 pads total
    for x in range(130000, 1220000, 100000):
        out.extend(add_box(70, 0, x, 15000, x + 70000, 85000))
        out.extend(add_box(70, 0, x, 1265000, x + 70000, 1335000))
    for y in range(130000, 1220000, 100000):
        out.extend(add_box(70, 0, 15000, y, 85000, y + 70000))
        out.extend(add_box(70, 0, 1265000, y, 1335000, y + 70000))

    # ENDSTR (record 0x07, type 0x00)
    out.extend(gds_record(0x07, 0x00))

    # ENDLIB (record 0x04, type 0x00)
    out.extend(gds_record(0x04, 0x00))

    with open(filename, "wb") as f:
        f.write(out)

    print(f"[SUCCESS] Generated binary GDSII layout stream: {filename} ({len(out)} bytes)")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    gds_dir = os.path.abspath(os.path.join(script_dir, "..", "gds"))
    os.makedirs(gds_dir, exist_ok=True)
    target_gds = os.path.join(gds_dir, "can_fd_controller_top.gds")
    write_gds(target_gds)
