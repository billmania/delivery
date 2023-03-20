"""
Utility functions.
"""

from typing import Tuple
from crcmod import mkCrcFun
from pathlib import Path
from collections import namedtuple

#
# Setup to calculate the CRC-16-IBM checksum using the polynomial
# x^^16 + x^^15 + x^^2 + 1.
#
crc16 = mkCrcFun(poly=0x18005, initCrc=0, rev=True, xorOut=0)


class CRCException(Exception):
    pass


Packer = namedtuple('Packer', ['bytes', 'type'])
packing = {
    'B': Packer(1, 'uint8'),
    '?': Packer(1, 'bool'),
    'h': Packer(2, 'int16'),
    'H': Packer(2, 'uint16'),
    'i': Packer(4, 'int32'),
    'l': Packer(4, 'int32'),
    'I': Packer(4, 'uint32'),
    'L': Packer(4, 'uint32'),
    'f': Packer(4, 'float'),
    's': Packer(1, 'bytes')
}
digits = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']


def unpacking(format: str) -> Tuple[int, list]:
    """
    Interpret format as an unpack format string. Return
    the number of bytes that format will require in the
    buffer passed to unpack() and the offset of each attribute
    in the buffer.

    format can contain characters from set(packing.keys()) and unsigned
    integers. If any other characters are found in format, byte_qty
    returns (0, []).
    """

    quantity = 0
    counter = 0
    offset = 0
    offsets = list()

    for entry in format:
        if entry in digits:
            counter = counter * 10 + int(entry)

            continue

        elif entry in packing:
            offsets.append(offset)
            if counter == 0:
                counter = 1

            entry_qty = counter * packing[entry].bytes
            quantity += entry_qty
            offset += entry_qty
            counter = 0

            continue

        return (0, [])

    return (quantity, offsets)


def cksum_is_ok(message: bytes) -> bool:
    """
    Calculate the USBL checksum of the message. Return True
    if the calculated checksum matches the checksum in the message,
    otherwise False.

    The minimum message length is three bytes: one for the CID and two for
    the checksum. The synchronization character is not expected at the front
    of the message and the end-of-message character(s) is not expected at
    the end.
    Also, the message is not expected to be in USBL "data encoding" format.
    """

    if len(message) > 2:
        message_checksum = (message[-1] << 8) + message[-2]
        checksum = crc16(message[:-2])

        return checksum == message_checksum

    raise CRCException("Minimum message is 3 bytes")


def add_cksum(message: bytes) -> bytes:
    """
    Calculate a checksum for the message. Return an updated message
    with the calculated checksum appended.

    The minimum message length is one byte: the CID.
    The synchronization character is not expected at the front of the message
    and the end-of-message character(s) is not expected at the end.
    Also, the message is not expected to be in USBL "data encoding" format.
    """

    if message:
        checksum = crc16(message)
        return message + checksum.to_bytes(2, byteorder='little')

    raise CRCException("No message provided")


def usbl_encode(plain: bytes) -> bytes:
    """
    Encode a collection of bytes to the USBL scheme. The output will
    always have an even number of bytes.

    b'\x0e\xf1' -> b'0EF1'
    """

    encoded = list()
    for byte in plain:
        #
        # Extract the high nibble, convert it to the upper-case ASCII
        # character for the value in hexadecimal and add it to encoded.
        # Do the same for the low nibble.
        #
        high = (byte & 0xf0) >> 4
        low = byte & 0x0f
        encoded.append(ord(f"{high:X}"))
        encoded.append(ord(f"{low:X}"))

    return bytes(encoded)


def usbl_decode(encoded: bytes) -> bytes:
    """
    Decode a collection of bytes from the USBL scheme. The input
    must have an even number of bytes.

    b'0E' -> b'\x0e'
    """

    if (len(encoded) % 2) != 0:
        raise ValueError("encoded message must have an even number of bytes")

    decoded_bytes = list()
    for index in range(0, len(encoded), 2):
        decoded_bytes.append((int(chr(encoded[index]), 16) << 4) +
                             int(chr(encoded[index+1]), 16))

    return bytes(decoded_bytes)


def usbl_parse(msg_in: bytes) -> Tuple[int, bytes, bool]:
    """
    Decode the message, compare the checksum. Extract and return the CID, payload,
    and result of the checksum comparison.

    The SYNC character and the EOM character(s) are expected to have
    already been removed from msg_in.
    """

    message = usbl_decode(msg_in)
    if cksum_is_ok(message):
        return message[0], message[1:-2], True

    return None, None, False


def usbl_assemble(cid: int, payload: bytes = None) -> bytes:
    """
    Assemble an outbound message with the CID and payload,
    calculate and append a checksum, and return the message.
    The new message will not include a SYNC character or the EOM
    character(s).
    """

    if cid is None:
        raise Exception("cid must be provided")

    if payload is None:
        payload = b''

    message = cid.to_bytes(1, byteorder='big') + payload
    complete_msg = usbl_encode(add_cksum(message))

    return complete_msg
