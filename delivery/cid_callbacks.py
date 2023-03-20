from typing import Callable
from delivery.usbl_messages import CID
from delivery.usbl_datagram import USBLDatagram


class CIDNotFound(Exception):
    pass


class CIDCallbacks(object):
    """
    Associate a callback function with a CID. For use by the function
    which receives datagrams from the device, extracts the CID, and
    processes the datagram.

    Since the Python dict data type is thread-safe, a Lock is not necessary
    between set_callback() and callback().
    """

    def __init__(self):
        self._callbacks = dict()

    def set_callback(self, cid: CID, callback: Callable):
        """
        Define or re-define the callback for a CID.
        """

        self._callbacks[cid] = callback

    def callback(self, datagram: USBLDatagram):
        """
        Execute the callback for datagram. If the datagram's CID doesn't
        have a defined callback, raise CIDNotFound.
        """

        cid = CID(datagram.cid)
        if cid in self._callbacks:
            return self._callbacks[cid](datagram)

        raise CIDNotFound(f"{CID} doesn't have a callback. Use set_callback()")
