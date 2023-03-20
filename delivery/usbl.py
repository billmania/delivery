"""
blueprint subsea USBL and acoustic modem support. No ROS dependencies.
"""
import serial
from delivery.usbl_datagram import USBLDatagram


#
# The sequence of characters at the end of every datagram.
#
EOD = b'\r\n'
#
# Commands are sent to the local USBL device via the serial port.
# Responses come from the local USBL device via the same serial port.
#
CMD_SYNC = b'#'
RESP_SYNC = b'$'

#
# A really long timeout for reading datagrams, so the read() won't
# block forever, just in case. A short timeout for write(), since
# writes should always be quick.
#
READ_TIMEOUT_S = 30.0
WRITE_TIMEOUT_S = 0.1


class NoDatagram(Exception):
    pass


class USBL(object):
    """
    Support for the blueprint subsea X150 and X010 USBL and acoustic modem
    devices.
    """

    def __init__(self, name: str,
                 port_name: str,
                 data_rate: int):
        """
        Setup the serial port for communication with the device.
        """

        self._port_name = port_name
        self._data_rate = data_rate

        self._port_in = None
        self._port_out = None
        self._connect()

    def _connect(self):
        """
        Create an input and an output connection to the device's
        serial port. It's expected one thread will use the write()
        method and a separate thread will use the read().
        """

        self._port_out = serial.Serial(self._port_name,
                                       self._data_rate,
                                       timeout=READ_TIMEOUT_S,
                                       write_timeout=WRITE_TIMEOUT_S)
        self._port_in = serial.Serial(self._port_name,
                                      self._data_rate,
                                      timeout=READ_TIMEOUT_S,
                                      write_timeout=WRITE_TIMEOUT_S)

        self._port_out.reset_output_buffer()
        self._port_in.reset_input_buffer()

    def _disconnect(self):
        """
        Shutdown the serial port connections.
        """

        self._port_in.close()
        self._port_out.close()
        self._port_in = None
        self._port_out = None

    def identify(self) -> dict:
        """
        Retrieve the identification details from the device and return
        them as a dictionary of key-value pairs.
        """

        # TODO: Implement

        return dict()

    def write(self, datagram: USBLDatagram) -> None:
        """
        Write the assembled datagram to the serial port.
        """

        output = CMD_SYNC + datagram.dgram_out + EOD
        self._port_out.write(output)

    def read(self) -> USBLDatagram:
        """
        Read a datagram from the device via the serial port
        and turn it into a USBLDatagram. Otherwise raise
        the NoDatagram Exception.
        This method may block for "a long time".
        """

        raw_datagram = self._port_in.read_until(EOD)
        sync_pos = raw_datagram.find(RESP_SYNC)
        if sync_pos >= 0:
            return USBLDatagram(raw_datagram[sync_pos:-(len(EOD))])

        raise NoDatagram()
