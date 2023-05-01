"""
USBL messages, both Commands and Responses. Each message has one CID. For
request-response commands, the CID is the same for both Command and Response.

In datagrams, all multi-byte numerical values are stored little-endian.

The flow of commands is as follows:

    1. a CID command is chosen and bundled with the necessary attributes
    2. those bits are used to create a USBLMsg
    3. the USBLMsg is then used to create a USBLDatagram
    4. the USBLDatagram is passed to USBL.write() and written
       to the serial port

and the flow for responses:

    1. a USBLDatagram is returned by USBL.read() from the serial port
    2. a USBLMsg is created from the USBLDatagram contents
    3. the CID and attributes are available in the USBLMsg


"""

from abc import ABC, abstractmethod
from enum import Enum, unique
from struct import pack, unpack, error
from delivery.settings import SettingsIndex
from delivery.usbl_utils import unpacking
from delivery.usbl_datagram import USBLDatagram

#
# Indices of attributes unpacked from the SettingsGet payload. See
# SettingsGetResp message.
#
ATTR_STATUS_FLAGS = 0
ATTR_ENV_FLAGS = 10
ATTR_ENV_SALINITY = 12
ATTR_AHRS_FLAGS = 14
ATTR_XCVR_FLAGS = 19
ATTR_BEACON_ID = 20
ATTR_XCVR_RESP_TIME = 22

#
# Various constants specific to USBL.
#
DONT_CARE = 0
PLOAD_DAT = 0x3
MSG_OWAYU = 0x1
SALINITY_FRESHWATER = 0
SALINITY_SEAWATER = 35
POOL_RESPONSE_DELAY = 100
SEA_RESPONSE_DELAY = 10
#
# The maximum number of bytes for a CID.DAT_SEND Command. However, on p 115,
# there is the sentence "Where possible it is recommended (but not required) to
# limit packet lengths to between 16 to 20 bytes."
#
MAX_MSG_LENGTH = 31

RANGE_VALID    = 0b00000001  # noqa: E221
USBL_VALID     = 0b00000010  # noqa: E221
POSITION_VALID = 0b00000100


#
# The state of the transceiver.
#
STATE_STOPPED = 0x3A
STATE_IDLE = 0x3B
STATE_TX = 0x3C
STATE_REQ = 0x3D
STATE_RX = 0x3E
STATE_RESP = 0x3F


@unique
class XCVRFLAGS(Enum):
    """
    The bit flags for the xcvr_flags byte of the settings payload.
    Bits two through four are reserved and kept reset.
    """

    # TODO: Replace the Enum with a simple collection of constants.

    XCVR_DIAG_MSGS       = 0b10000000  # noqa: E221
    XCVR_FIX_MSGS        = 0b01000000  # noqa: E221
    XCVR_USBL_MSGS       = 0b00100000  # noqa: E221
    XCVR_TX_MSGCTRL_ALL  = 0b00000000  # noqa: E221
    XCVR_POSFLT_ENABLE   = 0b00000010  # noqa: E221
    USBL_USE_AHRS        = 0b00000001  # noqa: E221


@unique
class CST_E(Enum):
    """
    Status codes
    """

    CST_OK                = 0x00  # noqa: E221
    CST_FAIL              = 0x01  # noqa: E221
    CST_CMD_PARAM_MISSING = 0x04
    CST_XCVR_RESP_TIMEOUT = 0x34
    CST_XCVR_RESP_ERROR   = 0x35  # noqa: E221
    CST_XCVR_RESP_WRONG   = 0x36  # noqa: E221


@unique
class CID(Enum):
    """
    The Command Identification Codes (CID) for the USBL devices,
    taken from page 40 of USBL Developer Guide, firmware version 2.4.
    """

    SYS_ALIVE = 1
    SYS_INFO = 2
    STATUS = 16
    STATUS_CFG_SET = 18
    SETTINGS_GET = 21
    SETTINGS_SET = 22
    SETTINGS_SAVE = 24

    CAL_ACTION = 32

    XCVR_TX_MSG = 49
    XCVR_RX_ERR = 50
    XCVR_RX_MSG = 51
    XCVR_RX_REQ = 52
    XCVR_RX_RESP = 53

    XCVR_USBL = 56
    XCVR_FIX = 57
    XCVR_STATUS = 58

    PING_SEND = 64
    PING_REQ = 65
    PING_RESP = 66
    PING_ERROR = 67

    DAT_SEND = 96
    DAT_RECEIVE = 97


@unique
class MSG(Enum):
    Cmd = 'C'
    Resp = 'R'


class MsgException(Exception):
    pass


class USBLMsg(ABC):
    """
    Functionality common to all Commands and Responses.
    """

    @abstractmethod
    def __init__(self, cid: CID, msg_type: MSG):
        self.cid = cid
        self.msg_type = msg_type
        #
        # For a Command, _datagram will be formed from the CID and payload
        # provided by the derived class. For a Response, it will be provided
        # from the USBL device.
        #
        self._datagram = None

    def datagram(self) -> USBLDatagram:
        """
        Overload this method to return the USBLDatagram corresponding
        to this USBLMsg, only if the datagram must first be modified.

        The method is always used with Command messages, in order to
        write the command to the serial port. For Response messages,
        the datagram is typically not used after the message has been
        created.
        """

        return self._datagram


class ReceiveMsgResp(USBLMsg):
    """
    CID_DAT_RECEIVE incoming message. Page 117 of the USBL SDK.

    The datagram contains an ACO_FIX, p 47, which has variable length.
    It may contain position, USBL, and range information.
    """

    def __init__(self, datagram: USBLDatagram, my_beacon_id: int):
        super().__init__(CID.DAT_RECEIVE, MSG.Resp)

        start_index = 0
        self._datagram = datagram
        unpack_format = 'BBBB12s'
        buffer_length, _ = unpacking(unpack_format)
        end_index = start_index + buffer_length
        attributes = unpack('<'+unpack_format,
                            self._datagram.payload[start_index:end_index])
        self.dest_id = attributes[0]
        self.my_message = (self.dest_id in [my_beacon_id, 0])
        if not self.my_message:
            #
            # This message is addressed to some other beacon.
            #
            return

        self.src_id = attributes[1]
        self.flags = attributes[2]
        self.position_valid = bool(self.flags & POSITION_VALID)
        self.usbl_valid = bool(self.flags & USBL_VALID)
        self.range_valid = bool(self.flags & RANGE_VALID)
        self.msg_type = attributes[3]
        self.attitude_section = attributes[4]

        if self.range_valid:
            start_index = end_index
            unpack_format = 'IiH'
            buffer_length, _ = unpacking(unpack_format)
            end_index = start_index + buffer_length
            attributes = unpack('<'+unpack_format,
                                self._datagram.payload[start_index:end_index])
            self.range_count = attributes[0]
            self.range_time = attributes[1]
            self.range_dist = attributes[2]

        if self.usbl_valid:
            start_index = end_index
            unpack_format = 'B'
            buffer_length, _ = unpacking(unpack_format)
            end_index = start_index + buffer_length
            attributes = unpack('<'+unpack_format,
                                self._datagram.payload[start_index:end_index])
            self.usbl_channels = attributes[0]

            start_index = end_index
            unpack_format = f"{self.usbl_channels*2}shhh"
            buffer_length, _ = unpacking(unpack_format)
            end_index = start_index + buffer_length
            attributes = unpack('<'+unpack_format,
                                self._datagram.payload[start_index:end_index])
            self.usbl_rssi = attributes[0]
            self.usbl_azimuth_deg = attributes[1] / 10.0
            self.usbl_elevation_deg = attributes[2] / 10.0
            self.usbl_fit_error = attributes[3]

        if self.position_valid:
            start_index = end_index
            unpack_format = 'hhh'
            buffer_length, _ = unpacking(unpack_format)
            end_index = start_index + buffer_length
            attributes = unpack('<'+unpack_format,
                                self._datagram.payload[start_index:end_index])
            self.easting = attributes[0]
            self.northing = attributes[1]
            self.depth = attributes[2]

        start_index = end_index
        unpack_format = '?B'
        buffer_length, _ = unpacking(unpack_format)
        end_index = start_index + buffer_length
        attributes = unpack('<'+unpack_format,
                            self._datagram.payload[start_index:end_index])
        self.ack_flag = attributes[0]
        self.packet_len = attributes[1]

        start_index = end_index
        unpack_format = f"{self.packet_len}s?"
        buffer_length, _ = unpacking(unpack_format)
        end_index = start_index + buffer_length
        attributes = unpack('<'+unpack_format,
                            self._datagram.payload[start_index:end_index])
        self.message = attributes[0]
        self.local_flag = bool(attributes[1])

    def __repr__(self):
        return ("ReceiveMsgResp\n"
                f" Dest ID: {self.dest_id}\n"
                f" Src ID: {self.src_id}\n"
                f" for me?: {self.local_flag}\n"
                f" message: {self.message}\n"
                f"  position: {self.position_valid}"
                f", usbl: {self.usbl_valid}"
                f", range: {self.range_valid}")


class StatusCfgSetCmd(USBLMsg):
    """
    CID_STATUS_CFG_SET command, to enable and disable status messages
    while calibrating the AHRS.
    """

    def __init__(self, enable: bool = False):
        super().__init__(CID.STATUS_CFG_SET, MSG.Cmd)

        #
        # Accelerometer and magnetometer calibration data, attitude
        # values, and environment values.
        # AHRS_RAW_DATA, ACC_CAL, MAG_CAL, ATTITUDE, and ENVIRONMENT
        #
        status_output = 0b00011111
        if enable:
            #
            # Automatic status messages enabled at 5 Hz
            #
            status_mode = 0x3
        else:
            #
            # Status messages disabled.
            #
            status_mode = 0x0

        pack_format = f"<BB"
        payload = pack(pack_format,
                       status_output,
                       status_mode)

        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=payload)


class CalActionCmd(USBLMsg):
    """
    CID_CAL_ACTION command to reset the accelerometer values,
    compute the pitch and roll limits, reset the magnetometer values,
    and compute the hard and soft iron compensation.
    """

    def __init__(self, device: str, action: str):
        """
        device can be one of:
            'ACC'
            'MAG'
        action can be one of:
            'RESET'
            'COMPUTE'
        """

        super().__init__(CID.CAL_ACTION, MSG.Cmd)

        if device == 'ACC':
            if action == 'RESET':
                cal_action = 0x01
            else:
                cal_action = 0x02
        else:
            if action == 'RESET':
                cal_action = 0x04
            else:
                cal_action = 0x05

        pack_format = f"<H"
        payload = pack(pack_format,
                       cal_action)

        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=payload)


class StatusCmd(USBLMsg):
    """
    Request a CID_STATUS response which contains the environment
    and attitude sections.
    """

    def __init__(self):
        super().__init__(CID.STATUS, MSG.Cmd)

        ENV_AND_ATT = 0b00000011
        payload = pack('<B', ENV_AND_ATT)
        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=payload)


class StatusResp(USBLMsg):
    """
    CID_STATUS responses.
    """

    def __init__(self, datagram: USBLDatagram):
        super().__init__(CID.STATUS, MSG.Resp)

        self._datagram = datagram

        AHRS_RAW    = 0b00010000  # noqa: E221
        ACC_CAL     = 0b00001000  # noqa: E221
        MAG_CAL     = 0b00000100  # noqa: E221
        ATTITUDE    = 0b00000010  # noqa: E221
        ENVIRONMENT = 0b00000001

        self.have_ahrs_raw = False
        self.have_acc_cal = False
        self.have_mag_cal = False
        self.have_attitude = False
        self.have_environment = False

        try:
            start_index = 0
            unpack_format = '<BQ'
            end_index = start_index + 9

            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            self.status_flags = attributes[0]
            self.timestamp_sec = round(attributes[1] / 1000.0, 1)

        except error as e:
            raise Exception("StatusResp flags and timestamp:"
                            f" {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

        try:
            if self.status_flags & ENVIRONMENT:
                self.have_environment = True

                start_index = end_index
                unpack_format = '<HhiiH'
                end_index = start_index + 14
                attributes = unpack(
                    unpack_format,
                    self._datagram.payload[start_index:end_index])
                self.supply_v = attributes[0] / 1000.0
                self.temperature_c = attributes[1] / 10.0
                self.pressure_mb = attributes[2]
                self.depth_m = attributes[3] / 10.0
                self.sound_mps = attributes[4] / 10.0

        except error as e:
            raise Exception(f"StatusResp Attitude: {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

        try:
            if self.status_flags & ATTITUDE:
                self.have_attitude = True

                start_index = end_index
                unpack_format = '<hhh'
                end_index = start_index + 6
                attributes = unpack(
                    unpack_format,
                    self._datagram.payload[start_index:end_index])
                self.yaw_deg = attributes[0] / 10.0
                self.pitch_deg = attributes[1] / 10.0
                self.roll_deg = attributes[2] / 10.0

        except error as e:
            raise Exception(f"StatusResp Attitude: {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

        try:
            if self.status_flags & MAG_CAL:
                self.have_mag_cal = True

                start_index = end_index
                unpack_format = '<B?IB'
                end_index = start_index + 7
                attributes = unpack(
                    unpack_format,
                    self._datagram.payload[start_index:end_index])
                self.mag_buffer_percent = attributes[0]
                self.mag_calibration_in_use = attributes[1]
                self.mag_calibration_age_sec = attributes[2]
                self.mag_calibration_fit_percent = attributes[3]

        except error as e:
            raise Exception(f"StatusResp Magnetometer: {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

        try:
            if self.status_flags & ACC_CAL:
                self.have_acc_cal = True

                start_index = end_index
                unpack_format = '<hhhhhh'
                end_index = start_index + 12
                attributes = unpack(
                    unpack_format,
                    self._datagram.payload[start_index:end_index])
                self.acc_min_x = attributes[0]
                self.acc_min_y = attributes[1]
                self.acc_min_z = attributes[2]
                self.acc_max_x = attributes[3]
                self.acc_max_y = attributes[4]
                self.acc_max_z = attributes[5]

        except error as e:
            raise Exception(f"StatusResp Accelerometer {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

        try:
            if self.status_flags & AHRS_RAW:
                self.have_ahrs_raw = True

                start_index = end_index
                unpack_format = '<hhhhhhhhh'
                end_index = start_index + 18
                attributes = unpack(
                    unpack_format,
                    self._datagram.payload[start_index:end_index])
                self.ahrs_raw_acc_x = attributes[0]
                self.ahrs_raw_acc_y = attributes[1]
                self.ahrs_raw_acc_z = attributes[2]
                self.ahrs_raw_mag_x = attributes[3]
                self.ahrs_raw_mag_y = attributes[4]
                self.ahrs_raw_mag_z = attributes[5]
                self.ahrs_raw_gyro_x = attributes[6]
                self.ahrs_raw_gyro_y = attributes[7]
                self.ahrs_raw_gyro_z = attributes[8]

        except error as e:
            raise Exception(f"StatusResp AHRS {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

    def __repr__(self):
        return ("Status\n"
                "\n")


class SendMsgCmd(USBLMsg):
    """
    CID_DAT_SEND command to send data to the remote device.
    """

    def __init__(self, dest_id: int, message: bytes):
        super().__init__(CID.DAT_SEND, MSG.Cmd)

        errors = ''
        if not (0 < dest_id < 16):
            errors += 'dest_id must be between 0 and 16, '
        msg_payload_len = len(message)
        if msg_payload_len > MAX_MSG_LENGTH:
            errors += f"message must be <= {MAX_MSG_LENGTH} bytes, "

        if errors:
            raise MsgException(errors)

        #
        # p 46
        #
        msg_type = MSG_OWAYU
        pack_format = f"<BBB{msg_payload_len}s"
        payload = pack(pack_format,
                       dest_id,
                       msg_type,
                       msg_payload_len,
                       message)

        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=payload)


class SettingsGetCmd(USBLMsg):
    """
    CID_SETTINGS_GET command to retrieve the current configuration.

    See the SettingsIndex class for the definition of the SETTINGS_T
    payload.
    """

    def __init__(self):
        super().__init__(CID.SETTINGS_GET, MSG.Cmd)
        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=None)


class SettingsSaveCmd(USBLMsg):
    """
    CID_SETTINGS_SAVE command to write the current configuration to EEPROM,
    to preserve it across a reboot.
    """

    def __init__(self):
        super().__init__(CID.SETTINGS_SAVE, MSG.Cmd)
        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=None)


class SettingsGetResp(USBLMsg):
    """
    CID_SETTINGS_GET response with the full SETTINGS_T structure.
    """

    def __init__(self, datagram: USBLDatagram):
        super().__init__(CID.SETTINGS_GET, MSG.Resp)

        self._datagram = datagram

        #
        # In the following format string, the attributes for net_mac_addr and
        # ahrs_cal are over-simplified and ignored. They're not really strings.
        #
        unpack_format = '<'

        #
        # For status_flags, ignore bits 3 through 7, bits 0 through 2 are
        # the rate of the CID_STATUS messages.
        #
        # TODO: Replace this with the SettingsIndex class
        #
        unpack_format += 'B'  # status_flags 1 byte 0
        unpack_format += 'B'  # status_output 1 byte 1
        unpack_format += 'B'  # uart_main_baud 1 byte 2
        unpack_format += 'B'  # uart_aux_baud 1 byte 3
        # TODO: Verify the net_mac_addr is not 6 bytes
        unpack_format += '6s'  # net_mac_addr 6 byte 4
        unpack_format += 'I'  # net_ip_addr 4 byte 10
        unpack_format += 'I'  # net_ip_subnet 4 byte 14
        unpack_format += 'I'  # net_ip_gateway 4 byte 18
        unpack_format += 'I'  # net_ip_dns 4 byte 22
        unpack_format += 'H'  # net_tcp_port 2 byte 26

        unpack_format += 'B'  # env_flags 1 byte 28
        unpack_format += 'l'  # env_pressure_ofs 4 byte 29
        unpack_format += 'H'  # env_salinity 2 byte 33
        unpack_format += 'H'  # env_vos 2 byte 35
        unpack_format += 'B'  # ahrs_flags 1 byte 37
        unpack_format += '51s'  # ahrs_cal 51 byte 38
        unpack_format += 'H'  # ahrs_yaw_ofs 2 byte 89
        unpack_format += 'H'  # ahrs_pitch_ofs 2 byte 91
        unpack_format += 'H'  # ahrs_roll_ofs 2 byte 93
        unpack_format += 'B'  # xcvr_flags 1 byte 95

        unpack_format += 'B'  # xcvr_beacon_id 1 byte 96
        unpack_format += 'H'  # xcvr_range_tmo 2 byte 97
        unpack_format += 'H'  # xcvr_resp_time 2 byte 99
        unpack_format += 'H'  # xcvr_yaw 2 byte 101
        unpack_format += 'H'  # xcvr_pitch 2 byte 103
        unpack_format += 'H'  # xcvr_roll 2 byte 105
        unpack_format += 'B'  # xcvr_posflt_vel 1 byte 107
        unpack_format += 'B'  # xcvr_posflt_ang 1 byte 108
        unpack_format += 'B'  # xcvr_posflt_tmo 1 byte 109

        try:
            attributes = unpack(unpack_format, self._datagram.payload)
            self.status_flags = attributes[ATTR_STATUS_FLAGS]
            self.env_flags = attributes[ATTR_ENV_FLAGS]
            self.env_salinity = attributes[ATTR_ENV_SALINITY]
            self.ahrs_flags = attributes[ATTR_AHRS_FLAGS]
            self.xcvr_flags = attributes[ATTR_XCVR_FLAGS]
            self.response_delay = attributes[ATTR_XCVR_RESP_TIME]
            self.beacon_id = attributes[ATTR_BEACON_ID]

        except error as e:
            raise Exception(f"SettingsGetResp exception in unpack: {e}"
                            f", {unpack_format}"
                            f", {len(datagram.payload)}"
                            f", {datagram.payload}")

    def __repr__(self):
        salinity = 'Freshwater' if self.env_salinity == SALINITY_FRESHWATER \
                                else 'Seawater'

        return ("Settings\n"
                f" status flags: {bin(self.status_flags)}\n"
                f" env flags: {bin(self.env_flags)}\n"
                f" ahrs flags: {bin(self.ahrs_flags)}\n"
                f" xcvr flags: {bin(self.xcvr_flags)}\n"
                f" response delay: {self.response_delay}\n"
                f" salinity: {salinity}\n"
                f" beacon ID: {self.beacon_id}")


class SettingsSetResp(USBLMsg):
    def __init__(self, datagram: USBLDatagram):
        super().__init__(CID.SETTINGS_SET, MSG.Resp)

        self._datagram = datagram

        attributes = unpack('<B',
                            self._datagram.payload)

        self.status = attributes[0]


class SettingsSetCmd(USBLMsg):
    """
    This Command causes a write to the device's EEPROM, which
    has a limited lifetime.

    CID_SETTINGS_SET command to configure the device.
    There are a ton of settings parameters defined, but many of them
    aren't touched. This Command depends on first using SettingsGetCmd
    to retrieve all of the current settings. The payload from the
    SettingsGetResp is used to initialize this Command and then
    the settings can be updated.

    After using all of the "set" methods write the SettingsSetCmd
    datagram to the device.
    """

    def __init__(self, current_settings: bytes):
        """
        current_settings is a complete payload of settings. It typically
        comes from the SettingsGetResp datagram.
        """

        super().__init__(CID.SETTINGS_SET, MSG.Cmd)
        if not current_settings:
            raise Exception('current_settings must be provided')

        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=current_settings)
        self._settings = SettingsIndex()

    def _make_datagram(self, attribute: str, value) -> bytes:
        pack_help = self._settings.pack_help(attribute)
        new_payload = self._datagram.payload[:pack_help.start]
        new_payload += pack('<'+pack_help.format, value)
        new_payload += self._datagram.payload[pack_help.end:]

        datagram = USBLDatagram(cid=self.cid.value,
                                payload=new_payload)
        return datagram

    def set_status_messages(self, enabled: bool = False):
        """
        Control the automatic output of status datagrams.
        """

        STATUS_MODE_MANUAL = 0x0
        STATUS_MODE_5HZ = 0x3

        self.status_enabled = enabled
        if self.status_enabled:
            status_flags = STATUS_MODE_5HZ
        else:
            status_flags = STATUS_MODE_MANUAL

        self._datagram = self._make_datagram('status_flags',
                                             status_flags)

    def set_xcvr_response_delay(self, response_delay: int):
        """
        Set the delay before the device will transmit the requested
        response. Longer delays reduce the potential for interference
        in echo-y environment (like swimming pools or shallow water).
        """

        if not (10 <= response_delay <= 1000):
            return

        self.response_delay = response_delay
        self._datagram = self._make_datagram('xcvr_resp_time',
                                             self.response_delay)

    def set_beacon_id(self, beacon_id: int):
        """
        Set the beacon ID to an unsigned integer between 1 and
        15, inclusive. The X150 connected to the Topside must always
        be 15.
        """

        if not (0 < beacon_id < 16):
            return

        self.beacon_id = beacon_id
        self._datagram = self._make_datagram('xcvr_beacon_id',
                                             self.beacon_id)

    def set_salinity(self, salinity_ppt: int):
        """
        Set the salinity value, for use by the automatic
        speed of sound calculation.
        """

        if not (0 <= salinity_ppt <= 100):
            return

        self._datagram = self._make_datagram('env_salinity',
                                             salinity_ppt)

    def set_ahrs_flags(self):
        """
        Disable automatic magnetometer calibration.
        """

        AHRS_FLAGS = 0b00000000

        self.ahrs_flags = AHRS_FLAGS
        self._datagram = self._make_datagram('ahrs_flags',
                                             self.ahrs_flags)

    def set_ahrs_offsets(self,
                         yaw_deg: int = 0,
                         pitch_deg: int = 0,
                         roll_deg: int = 0):
        """
        Apply offsets for the seatrac device to use when it publishes
        the values. blueprintsubsea recommend setting the roll offset
        to 180.0 when the X150 is oriented with the connector up
        and the transducer down.
        """

        self._datagram = self._make_datagram('ahrs_yaw_ofs',
                                             yaw_deg*10)
        self._datagram = self._make_datagram('ahrs_pitch_ofs',
                                             pitch_deg*10)
        self._datagram = self._make_datagram('ahrs_roll_ofs',
                                             roll_deg*10)

    def set_env_flags(self):
        """
        Set the env flags bits in the settings payload, to
        make the device calculate both the pressure offset
        and the speed of sound.
        """

        ENV_FLAGS = 0b00000011

        self.env_flags = ENV_FLAGS
        self._datagram = self._make_datagram('env_flags',
                                             self.env_flags)

    def set_xcvr_flags(self, xcvr_flags: int):
        """
        Set the xcvr_flags bits in the settings payload. Use the constants
        from XCVRFLAGS.
        Currently, bits 2 through 4 are reserved and left reset.
        """

        self.xcvr_flags = xcvr_flags
        self._datagram = self._make_datagram('xcvr_flags',
                                             self.xcvr_flags)


class PingErrorResp(USBLMsg):
    """
    A status message received when a PingSend command was written but the
    PING failed.
    """

    def __init__(self, datagram: USBLDatagram, beacon_id: int):
        super().__init__(CID.PING_ERROR, MSG.Resp)

        self._datagram = datagram

        self.status = None
        self.relevant_beacon_id = None

        start_index = 0
        unpack_format = '<'
        unpack_format += 'B'  # status 1 byte
        unpack_format += 'B'  # beacon_id 1 byte
        end_index = start_index + 2
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])

        self.status = attributes[0]
        self.relevant_beacon_id = attributes[1]

    def __repr__(self):
        representation = ("Ping Error"
                          f" for beacon: {self.relevant_beacon_id}"
                          f" status: {self.status}\n")
        return representation


class PingSendCmd(USBLMsg):
    """
    Ping a remote beacon by its beacon ID. The purpose is two-fold:
        1. Verify the remote beacon is responsive
        2. Determine the position of the remote beacon and the
           range to it.

    The responses to this Command are:
        PING_SEND
        XCVR_USBL
        XCVR_FIX
        PING_RESP
    in that order.
    """

    def __init__(self, beacon_id: int):
        super().__init__(CID.PING_SEND, MSG.Cmd)
        self._beacon_id = beacon_id
        #
        # Request a ping response and use extended depth and USBL
        # information, for a more accurate position.
        #
        MSG_REQX = 0x6
        payload = pack('<BB', self._beacon_id, MSG_REQX)
        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=payload)


class PingSendResp(USBLMsg):
    """
    CID_PING_SEND response with:
        status
        beacon ID
    """

    def __init__(self, datagram: USBLDatagram, beacon_id: int):
        super().__init__(CID.PING_SEND, MSG.Resp)

        self._datagram = datagram
        self.ping_sent = False
        attributes = unpack('<BB', self._datagram.payload)

        self._status = attributes[0]
        self._beacon_id = attributes[1]

        CST_OK = 0x0
        if self._beacon_id == beacon_id and self._status == CST_OK:
            self._ping_sent = True

    def __repr__(self):
        return ("PingSend"
                f" Status: {self._status}"
                f" Sent: {self._ping_sent}"
                f" beacon ID: {self._beacon_id}")


class SysInfoCmd(USBLMsg):
    """
    CID_SYS_INFO command to retrieve seconds of uptime and version
    information.
    """

    def __init__(self):
        super().__init__(CID.SYS_INFO, MSG.Cmd)
        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=None)


class XcvrStatusCmd(USBLMsg):
    """
    CID_XCVR_STATUS command to retrieve the current status. It's used
    to determine if the device is ready to transmit a message.
    """

    def __init__(self):
        super().__init__(CID.XCVR_STATUS, MSG.Cmd)
        self._datagram = USBLDatagram(cid=self.cid.value,
                                      payload=None)


class XcvrStatusResp(USBLMsg):
    """
    CID_XCVR_STATUS response with:
    """

    def __init__(self, datagram: USBLDatagram):
        super().__init__(CID.XCVR_STATUS, MSG.Resp)

        self._datagram = datagram
        status = unpack('<B', self._datagram.payload)[0]

        self.stopped = bool(status == STATE_STOPPED)
        self.idle = bool(status == STATE_IDLE)
        self.transmitting = bool(status == STATE_TX)
        self.requesting = bool(status == STATE_REQ)
        self.receiving = bool(status == STATE_RX)
        self.responding = bool(status == STATE_RESP)

    def __repr__(self):
        return ("XcvrStatus is "
                f" {'Stopped ' if self.stopped else ''}"
                f" {'Idle ' if self.idle else ''}"
                f" {'Transmitting ' if self.transmitting else ''}"
                f" {'Requesting ' if self.requesting else ''}"
                f" {'Receiving ' if self.receiving else ''}"
                f" {'Responding ' if self.responding else ''}"
                "\n"
                )


class SysInfoResp(USBLMsg):
    """
    CID_SYS_INFO response.
    There are at least two issues with
    https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf
    for Beacon firmware version 2.4:
        1. the SYS_INFO response datagram contains the value 795 in the
           hardware part_number attribute, for BOTH the X150 and the X110
        2. the extended_info attribute isn't present when the section
           attribute is equal to 1
    """

    def __init__(self, datagram: USBLDatagram):
        super().__init__(CID.SYS_INFO, MSG.Resp)

        self._datagram = datagram
        unpack_format = '<'
        unpack_format += 'I'  # seconds 0
        unpack_format += 'B'  # section 1

        unpack_format += 'H'  # part_number 2
        unpack_format += 'B'  # part_rev 3
        unpack_format += 'I'  # serial_number 4
        unpack_format += 'H'  # flags_sys 5
        unpack_format += 'H'  # flags_user 6

        unpack_format += '?'  # boot_valid 7
        unpack_format += 'H'  # boot_part_number 8
        unpack_format += 'B'  # boot_version_maj 9
        unpack_format += 'B'  # boot_version_min 10
        unpack_format += 'H'  # boot_version_build 11
        unpack_format += 'I'  # boot_checksum 12

        unpack_format += '?'  # main_valid 13
        unpack_format += 'H'  # main_part_number 14
        unpack_format += 'B'  # main_version_maj 15
        unpack_format += 'B'  # main_version_min 16
        unpack_format += 'H'  # main_version_build 17
        unpack_format += 'I'  # main_checksum 18

        unpack_format += 'B'  # board_rev 19
#        unpack_format += 'B'  # extended_info 20
        try:
            attributes = unpack(unpack_format, self._datagram.payload)

        except Exception as e:
            print(f" Exception {e} in usbl_info unpack")
            print(f" Payload len: {len(self._datagram.payload)}"
                  f" {self._datagram.payload}")

        self.seconds = attributes[0]
        self.section = attributes[1]
        self.hardware = {
            'part_number': 'X150' if attributes[2] == 795 else 'X110',
            'part_rev': attributes[3],
            'serial_number': attributes[4],
            'flags_sys': attributes[5],
            'flags_user': attributes[6]
        }
        self.boot_firmware = {
            'valid': attributes[7],
            'part_number': attributes[8],
            'version': f"{attributes[9]}.{attributes[10]}.{attributes[11]}",
            'cksum': attributes[12]
        }
        self.main_firmware = {
            'valid': attributes[13],
            'part_number': attributes[14],
            'version': f"{attributes[15]}.{attributes[16]}.{attributes[17]}",
            'cksum': attributes[18]
        }
        self.board_rev = attributes[19]
        self.application = 'Main' if self.section == 1 else 'Bootloader'

    def __repr__(self):
        return ("SysInfo\n"
                f" Uptime seconds: {self.seconds}\n"
                f" Application: {self.application}\n"
                f" Hardware: {self.hardware}\n"
                f" Boot firmware: {self.boot_firmware}\n"
                f" Main firmware: {self.main_firmware}\n"
                f" Board revision: {self.board_rev}")


class XcvrUsblResp(USBLMsg):
    """
    CID_XCVR_USBL response.
    """

    def __init__(self, datagram: USBLDatagram, beacon_id: int):
        super().__init__(CID.XCVR_USBL, MSG.Resp)

        self._datagram = datagram

        start_index = 0
        unpack_format = '<'
        unpack_format += 'f'  # xcor_sig_peak 4 bytes
        unpack_format += 'f'  # xcor_threshold 4 bytes
        unpack_format += 'H'  # xcor_cross_point 2 bytes
        unpack_format += 'f'  # xcor_cross_mag 4 bytes
        unpack_format += 'H'  # xcor_detect 2 bytes
        unpack_format += 'H'  # xcor_length 2 bytes
        total_bytes = 18
        end_index = start_index + total_bytes
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        self.xcor_sig_peak = attributes[0]
        self.xcor_threshold = attributes[1]
        self.xcor_cross_point = attributes[2]
        self.xcor_cross_mag = attributes[3]
        self.xcor_detect = attributes[4]
        self.xcor_length = attributes[5]
        start_index = end_index

        unpack_format = '<'
        for count in range(self.xcor_length):
            unpack_format += 'f'  # xcor_data xcor_length*4 bytes
        end_index = start_index + (self.xcor_length * 4)
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.xcor_data = attributes

        unpack_format = '<'
        unpack_format += 'B'  # channels 1 byte

        end_index = start_index + 1
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.channels = attributes[0]

        unpack_format = '<'
        for count in range(self.channels):
            unpack_format += 'h'  # channel_rssi channels*2 byte
        end_index = start_index + (self.channels * 2)
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.channel_rssi = attributes

        unpack_format = '<'
        unpack_format += 'B'  # baselines 1 byte
        end_index = start_index + 1
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.baselines = attributes[0]

        unpack_format = '<'
        for count in range(self.baselines):
            unpack_format += 'f'  # phase_angle baselines*4 bytes
        end_index = start_index + (self.baselines * 4)
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.phase_angle = attributes

        unpack_format = '<'
        unpack_format += 'h'  # signal_azimuth 2 byte
        unpack_format += 'h'  # signal_elevation 2 byte
        unpack_format += 'f'  # signal_fit_error 4 byte
        end_index = start_index + 8
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        self.signal_azimuth = attributes[0]
        self.signal_elevation = attributes[1]
        self.signal_fit_error = attributes[2]

    def __repr__(self):
        return ("XcvrUsbl\n"
                f" xcor_sig_peak: {self.xcor_sig_peak}\n"
                f" xcor_threshold: {self.xcor_threshold}\n"
                f" xcor_cross_point: {self.xcor_cross_point}\n"
                f" xcor_cross_mag: {self.xcor_cross_mag}\n"
                f" xcor_detect: {self.xcor_detect}\n"
                f" xcor_data: ({self.xcor_length}) {self.xcor_data}\n"
                f" channel_rssi: ({self.channels}) {self.channel_rssi}\n"
                f" phase_angle: ({self.baselines}) {self.phase_angle}\n"
                f" signal_azimuth: {self.signal_azimuth}\n"
                f" signal_elevation: {self.signal_elevation}\n"
                f" signal_fit_error: {self.signal_fit_error}\n")


class XcvrFixResp(USBLMsg):
    """
    CID_XCVR_FIX response. Page 107 of
    https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf
    """

    def __init__(self, datagram: USBLDatagram, beacon_id: int):
        super().__init__(CID.XCVR_FIX, MSG.Resp)

        self._datagram = datagram

        start_index = 0
        unpack_format = '<'
        unpack_format += 'B'  # dest_d 1 byte
        unpack_format += 'B'  # src_d 1 byte
        unpack_format += 'B'  # flags 1 byte
        unpack_format += 'B'  # msg_type 1 byte
        unpack_format += 'h'  # attitude_yaw (X150) 2 byte
        unpack_format += 'h'  # attitude_pitch (X150) 2 byte
        unpack_format += 'h'  # attitude_roll (X150) 2 byte
        unpack_format += 'H'  # depth_local (X150) 2 byte
        unpack_format += 'H'  # vos 2 byte
        unpack_format += 'h'  # rssi 2 byte
        end_index = start_index + 16
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.dest_d = attributes[0]
        self.src_d = attributes[1]
        self.flags = attributes[2]
        self.msg_type = attributes[3]

        #
        # The positive Z axis goes through the center of the X150
        # body cylinder, from the transducer to the connector.
        #
        # The positive X axis is perpendicular to the Z axis, from
        # the center of the body cylinder out toward the Front Reference
        # Marking (the vertical bar with a triangle on either side).
        #
        # The positive Y axis is perpendicular to the both the Z and X
        # axes and as described by the right hand thumb (X), index (Y),
        # and middle (Z) fingers.
        #
        # With the way the X150 is mounted on the ROV, with the transducer
        # pointed down, the connector up, and the Front Reference Marking
        # facing forward, X is forward, Y is left, and Z is up, according to:
        # https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf
        #
        # TODO: Determine if it's necessary and how to offset X150 roll, pitch,
        #       and yaw
        #
        # However, after setting roll_offset to 180, only the roll values are
        # correct (port side up is positive). Positive pitch values should
        # indicate bow down but instead indicate bow up. Yaw values are
        # inconsistent.
        # Also, yaw values are NOT compass headings but instead are described
        # as being relative to magnetic North.
        #

        self.attitude_yaw = attributes[4] / 10.0
        self.attitude_pitch = attributes[5] / 10.0
        self.attitude_roll = attributes[6] / 10.0
        self.depth_local = attributes[7] / 10.0
        self.vos = attributes[8]
        self.rssi = attributes[9]

        if self.flags & RANGE_VALID:
            unpack_format = '<'
            unpack_format += 'L'  # range_count 4 byte
            unpack_format += 'l'  # range_time 4 byte
            unpack_format += 'H'  # range_dist (to the X110) 2 byte
            end_index = start_index + 10
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.range_count = attributes[0]
            self.range_time = attributes[1]
            self.range_dist_m = attributes[2] / 10.0
        else:
            self.range_dist = None

        if self.flags & USBL_VALID:
            unpack_format = '<'
            unpack_format += 'B'  # usbl_channels 1 byte
            end_index = start_index + 1
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.usbl_channels = attributes[0]
            unpack_format = '<'
            for count in range(self.usbl_channels):
                unpack_format += 'h'  # usbl_rssi 2 byte
            end_index = start_index + (self.usbl_channels * 2)
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.usbl_rssi = attributes

        unpack_format = '<'
        unpack_format += 'h'  # usbl_azimuth 2 byte
        unpack_format += 'h'  # usbl_elevation 2 byte
        unpack_format += 'h'  # usbl_fit_error 2 byte
        end_index = start_index + 6
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        #
        # valid azimuth angles are [0, 360] relative to the "front" of
        # the X150.
        #
        self.usbl_azimuth_deg = attributes[0] / 10.0
        #
        # valid elevation angles are [-90, 90] relative to the X-Y plane
        # of the X150.
        #
        self.usbl_elevation_deg = attributes[1] / 10.0
        self.usbl_fit_error = attributes[2]

        if self.flags & POSITION_VALID:
            #
            # See page 52 of
            # https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf
            #
            unpack_format = '<'
            unpack_format += 'h'  # position_easting_dm 2 byte
            unpack_format += 'h'  # position_northing_dm 2 byte
            unpack_format += 'h'  # position_depth_dm 2 byte
            end_index = start_index + 6
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            #
            # remote_east_m and remote_north_m are the position of the X110,
            # relative to the X150 and as calculated by the X150.
            #
            self.remote_east_m = attributes[0] / 10
            self.remote_north_m = attributes[1] / 10
            #
            # remote_depth_m is the depth of the X110, as calculated by the
            # X150.
            #
            self.remote_depth_m = attributes[2] / 10
        else:
            self.remote_east_m = 0
            self.remote_north_m = 0
            self.remote_depth_m = 0

    def __repr__(self):
        representation = ("XcvrFix\n"
                          f" dest_d: {self.dest_d}\n"
                          f" src_d: {self.src_d}\n"
                          f" flags: {self.flags}\n"
                          f" msg_type: {self.msg_type}\n"
                          f" attitude_yaw: {self.attitude_yaw}\n"
                          f" attitude_pitch: {self.attitude_pitch}\n"
                          f" attitude_roll: {self.attitude_roll}\n"
                          f" depth_local: {self.depth_local}\n"
                          f" vos: {self.vos}\n"
                          f" rssi: {self.rssi}\n")

        if self.flags & RANGE_VALID:
            representation += (f" range_count: {self.range_count}\n"
                               f" range_time: {self.range_time}\n"
                               f" range_dist: {self.range_dist_m}\n")

        if self.flags & USBL_VALID:
            representation += (f" usbl_rssi: ({self.usbl_channels})"
                               f" {self.usbl_rssi}\n")

        representation += (f" usbl_azimuth: {self.usbl_azimuth_deg}\n"
                           f" usbl_elevation: {self.usbl_elevation_deg}\n"
                           f" usbl_fit_error: {self.usbl_fit_error}\n")

        if self.flags & POSITION_VALID:
            representation += (f" northing_m: {self.position_northing_m}"
                               f" easting_m: {self.position_easting_m}"
                               f" depth_m: {self.position_depth_m}\n")

        return representation


class PingRespResp(USBLMsg):
    """
    CID_PING_RESP response. Has the same information as XCVR_FIX_RESPONSE.
    """

    def __init__(self, datagram: USBLDatagram, beacon_id: int):
        super().__init__(CID.PING_RESP, MSG.Resp)

        self._datagram = datagram

        self.position_easting_m = None
        self.position_northing_m = None
        self.position_depth_m = None
        self.wrong_beacon = False

        start_index = 0
        unpack_format = '<'
        unpack_format += 'B'  # dest_d 1 byte
        unpack_format += 'B'  # src_d 1 byte
        unpack_format += 'B'  # flags 1 byte
        unpack_format += 'B'  # msg_type 1 byte
        unpack_format += 'h'  # attitude_yaw 2 byte
        unpack_format += 'h'  # attitude_pitch 2 byte
        unpack_format += 'h'  # attitude_roll 2 byte
        unpack_format += 'H'  # depth_local 2 byte
        unpack_format += 'H'  # vos 2 byte
        unpack_format += 'h'  # rssi 2 byte
        end_index = start_index + 16
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.dest_d = attributes[0]
        self.src_d = attributes[1]
        if self.src_d != beacon_id:
            self.wrong_beacon = True
            return

        self.flags = attributes[2]
        self.msg_type = attributes[3]
        #
        # Next four attributes are for the X150.
        #
        self.attitude_yaw = attributes[4]
        self.attitude_pitch = attributes[5]
        self.attitude_roll = attributes[6]
        self.depth_local = attributes[7]
        self.vos = attributes[8]
        self.rssi = attributes[9]

        if self.flags & RANGE_VALID:
            unpack_format = '<'
            unpack_format += 'L'  # range_count 4 byte
            unpack_format += 'l'  # range_time 4 byte
            unpack_format += 'H'  # range_dist 2 byte
            end_index = start_index + 10
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.range_count = attributes[0]
            self.range_time = attributes[1]
            self.range_dist = attributes[2]

        if self.flags & USBL_VALID:
            unpack_format = '<'
            unpack_format += 'B'  # usbl_channels 1 byte
            end_index = start_index + 1
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.usbl_channels = attributes[0]
            unpack_format = '<'
            for count in range(self.usbl_channels):
                unpack_format += 'h'  # usbl_rssi 2 byte
            end_index = start_index + (self.usbl_channels * 2)
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.usbl_rssi = attributes

        unpack_format = '<'
        unpack_format += 'h'  # usbl_azimuth 2 byte
        unpack_format += 'h'  # usbl_elevation 2 byte
        unpack_format += 'h'  # usbl_fit_error 2 byte
        end_index = start_index + 6
        attributes = unpack(unpack_format,
                            self._datagram.payload[start_index:end_index])
        start_index = end_index
        self.usbl_azimuth_deg = attributes[0] / 10.0
        self.usbl_elevation_deg = attributes[1] / 10.0
        self.usbl_fit_error = attributes[2]

        if self.flags & POSITION_VALID:
            unpack_format = '<'
            unpack_format += 'h'  # position_easting_dm 2 byte
            unpack_format += 'h'  # position_northing_dm 2 byte
            unpack_format += 'h'  # position_depth_dm 2 byte
            end_index = start_index + 6
            attributes = unpack(unpack_format,
                                self._datagram.payload[start_index:end_index])
            start_index = end_index
            self.position_easting_m = attributes[0] / 10
            self.position_northing_m = attributes[1] / 10
            self.position_depth_m = attributes[2] / 10

    def __repr__(self):
        if self.position_northing_m is not None:
            representation = (f" northing: {self.position_northing_m}"
                              f" easting: {self.position_easting_m}"
                              f" depth: {self.position_depth_m}\n")
        else:
            representation = 'No fix available'

        return representation
