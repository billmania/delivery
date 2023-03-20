"""
USBL datagram support.
"""
from delivery.usbl_utils import usbl_parse, usbl_assemble


class USBLDatagram(object):
    """
    A command or response datagram.

    The developer guide which describes the content of datagrams
    is at:
    https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf
    """

    def __init__(self,
                 dgram_in: bytes = None,
                 cid: int = None,
                 payload: bytes = None):
        """
        For an inbound datagram, dgram_in will be non-None and consist of a
        sync character, a CID, the payload, and the checksum. When cksum_valid
        is True, the extracted cid and payload will be available. When
        cksum_valid is False, the cid and payload will be None.

        For an outbound datagram, the CID and payload will be non-None.
        A checksum will be calculated and a complete datagram will be assembled
        as dgram_out.
        """

        if dgram_in:
            self.dgram_in = dgram_in
            self.cid, self.payload, self.cksum_valid = usbl_parse(
                self.dgram_in[1:])

        else:
            self.cid = cid
            self.payload = payload
            self.cksum_valid = True
            self.dgram_out = usbl_assemble(self.cid,
                                           self.payload)

    def __repr__(self):
        return ("Datagram\n"
                f"CID: {self.cid}\n"
                f"Payload: {self.payload}\n"
                f"Cksum valid: {self.cksum_valid}")
