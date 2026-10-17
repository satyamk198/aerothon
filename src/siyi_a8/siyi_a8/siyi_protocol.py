"""SIYI gimbal SDK packet encoding/decoding (no ROS).

Packet format, from SIYI's SDK document as quoted in ArduPilot's AP_Mount_Siyi.h:

    STX      2 bytes  0x55 0x66
    CTRL     1 byte   bit 0: need_ack, bit 1: is an ack
    Data_len 2 bytes  little-endian
    SEQ      2 bytes  little-endian
    CMD_ID   1 byte
    DATA     Data_len bytes
    CRC16    2 bytes  little-endian, CRC-16/CCITT (poly 0x1021, init 0) over all previous bytes

Commands used here (IDs from ArduPilot AP_Mount_Siyi and mzahana/siyi_sdk):
    0x0C  photo: data = 1 byte function, 0 = take picture (no ack is sent)
    0x0D  acquire gimbal attitude: reply = yaw, pitch, roll, yaw rate, pitch rate, roll rate,
          each int16 little-endian in 0.1 deg (deg/s)
    0x0E  set gimbal attitude: data = yaw, pitch as int16 little-endian in 0.1 deg.
          A8 mini limits: yaw -135..135 deg, pitch -90 (straight down)..25 deg
    0x0F  absolute zoom: data = integer part, tenths of the zoom multiple (2.5x -> 02 05).
          A8 mini: 1..6x digital zoom. ArduPilot does not parse the reply, nor do we.
"""
import struct

HEADER = b"\x55\x66"
CMD_PHOTO = 0x0C
CMD_ACQUIRE_GIMBAL_ATTITUDE = 0x0D
CMD_SET_GIMBAL_ATTITUDE = 0x0E
CMD_ABSOLUTE_ZOOM = 0x0F
PHOTO_TAKE_PICTURE = 0

A8_MINI_YAW_LIMITS = (-135.0, 135.0)
A8_MINI_PITCH_LIMITS = (-90.0, 25.0)
A8_MINI_ZOOM_LIMITS = (1.0, 6.0)


def crc16(data: bytes) -> int:
    """CRC-16/CCITT, polynomial 0x1021, initial value 0 (as in ArduPilot crc16_ccitt(..., 0))."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(cmd_id: int, data: bytes = b"", seq: int = 0, need_ack: bool = True) -> bytes:
    body = HEADER + struct.pack("<BHHB", 1 if need_ack else 0, len(data), seq & 0xFFFF, cmd_id) + data
    return body + struct.pack("<H", crc16(body))


def set_attitude(yaw_deg: float, pitch_deg: float, seq: int = 0) -> bytes:
    yaw = min(max(yaw_deg, A8_MINI_YAW_LIMITS[0]), A8_MINI_YAW_LIMITS[1])
    pitch = min(max(pitch_deg, A8_MINI_PITCH_LIMITS[0]), A8_MINI_PITCH_LIMITS[1])
    return encode(CMD_SET_GIMBAL_ATTITUDE, struct.pack("<hh", round(yaw * 10), round(pitch * 10)), seq)


def take_picture(seq: int = 0) -> bytes:
    return encode(CMD_PHOTO, bytes([PHOTO_TAKE_PICTURE]), seq)


def absolute_zoom(multiple: float, seq: int = 0) -> bytes:
    multiple = min(max(multiple, A8_MINI_ZOOM_LIMITS[0]), A8_MINI_ZOOM_LIMITS[1])
    whole = int(multiple)
    tenths = min(int(round((multiple - whole) * 10)), 9)
    return encode(CMD_ABSOLUTE_ZOOM, bytes([whole, tenths]), seq)


def request_attitude(seq: int = 0) -> bytes:
    return encode(CMD_ACQUIRE_GIMBAL_ATTITUDE, b"", seq)


def decode(packet: bytes):
    """Return (cmd_id, data) for a valid packet, or None (bad header, length or CRC)."""
    if len(packet) < 10 or packet[:2] != HEADER:
        return None
    _, data_len, _, cmd_id = struct.unpack_from("<BHHB", packet, 2)
    end = 8 + data_len
    if len(packet) < end + 2:
        return None
    (crc,) = struct.unpack_from("<H", packet, end)
    if crc != crc16(packet[:end]):
        return None
    return cmd_id, packet[8:end]


def parse_attitude(data: bytes):
    """(yaw, pitch, roll) in degrees from an ACQUIRE_GIMBAL_ATTITUDE reply, or None."""
    if len(data) != 12:
        return None
    yaw, pitch, roll = struct.unpack_from("<hhh", data)
    return yaw / 10.0, pitch / 10.0, roll / 10.0
