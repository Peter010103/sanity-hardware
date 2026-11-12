"""Synthetic 600 deg/s MSP/CTBR inverse used only by closed-loop simulators."""

import functools
import math
import struct


def decode_msp(packet, collective=None):
    serial = bytes.fromhex(packet["serial_hex"])
    if serial[:3]!=b"$M<" or serial[3:5]!=bytes([16,200]) or functools.reduce(int.__xor__,serial[3:],0)!=0:
        raise ValueError("Invalid receiver MSP frame")
    rc = struct.unpack_from("<8H",serial,5)
    enabled = rc[4]>1500
    rates = [math.radians((rc[i]-1500)*600/500)*sign for i,sign in [(0,1),(1,1),(3,-1)]]
    thrust = collective.thrust_from_rc(rc[2]) if collective is not None else (rc[2]-1000)*3.2/800
    return {"ctbr":[thrust,*rates] if enabled else [0.]*4,"enabled":enabled}
