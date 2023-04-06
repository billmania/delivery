"""
Utilities for blueprint subsea scripts.
"""
from delivery.usbl_messages import CID, XCVRFLAGS
from delivery.usbl_messages import SALINITY_SEAWATER, SEA_RESPONSE_DELAY
from delivery.usbl_messages import SettingsGetCmd, SettingsGetResp
from delivery.usbl_messages import SettingsSetCmd, SettingsSaveCmd
from delivery.usbl_messages import SysInfoCmd, SysInfoResp
from delivery.usbl_messages import PingSendCmd, PingSendResp
from delivery.usbl_messages import XcvrUsblResp, XcvrFixResp, PingRespResp
from typing import Tuple
from time import sleep


def usbl_info(usbl) -> SysInfoResp:
    """
    Collect and return the detailed, static information about the device.
    It includes serial numbers and version numbers. It doesn't include
    anything which is software-configurable at run-time.
    """

    sys_info_cmd = SysInfoCmd()
    usbl.write(sys_info_cmd.datagram())
    ATTEMPTS = 5
    for attempt in range(ATTEMPTS):
        try:
            datagram = usbl.read()

        except Exception as e:
            print(f"SysInfoCmd Exception: {e}")
            continue

        if CID(datagram.cid) == CID.SYS_INFO:
            sys_info = SysInfoResp(datagram)
            return sys_info

    raise Exception("Failed to get SysInfo")


def usbl_setup(usbl,
               beacon_id: int = 15,
               yaw_offset: int = 0,
               pitch_offset: int = 0,
               roll_offset: int = 0):
    """
    Setup the device for use, such as the ID number, the status messages,
    and the transceiver flags.
    """

    settings_get_cmd = SettingsGetCmd()
    usbl.write(settings_get_cmd.datagram())

    ATTEMPTS = 5
    for attempt in range(ATTEMPTS):
        try:
            datagram = usbl.read()

        except Exception:
            continue

        if CID(datagram.cid) == CID.SETTINGS_GET:
            settings_get_resp = SettingsGetResp(datagram)
            break

    got_settings = attempt != (ATTEMPTS - 1)

    if got_settings:
        settings_set_cmd = SettingsSetCmd(settings_get_resp.datagram().payload)
    else:
        settings_set_cmd = SettingsSetCmd(None)

    settings_set_cmd.set_status_messages(enabled=False)
    new_xcvr_flags = (XCVRFLAGS.XCVR_FIX_MSGS.value
                      | XCVRFLAGS.XCVR_USBL_MSGS.value
                      | XCVRFLAGS.XCVR_POSFLT_ENABLE.value
                      | XCVRFLAGS.USBL_USE_AHRS.value)
    settings_set_cmd.set_xcvr_flags(new_xcvr_flags)
    settings_set_cmd.set_xcvr_response_delay(SEA_RESPONSE_DELAY)
    settings_set_cmd.set_salinity(SALINITY_SEAWATER)
    settings_set_cmd.set_ahrs_flags()
    settings_set_cmd.set_env_flags()
    settings_set_cmd.set_ahrs_offsets(yaw_deg=yaw_offset,
                                      pitch_deg=pitch_offset,
                                      roll_deg=roll_offset)
    settings_set_cmd.set_beacon_id(beacon_id)

    #
    # If the current settings already match the desired settings,
    # don't send SettingsSetCmd or SettingsSaveCmd, in order to
    # not make an EEPROM write.
    #
    if (settings_set_cmd.datagram().payload
       != settings_get_resp.datagram().payload):

        usbl.write(settings_set_cmd.datagram())
        old_payload = settings_get_resp.datagram().payload
        old_length = len(old_payload)
        new_payload = settings_set_cmd.datagram().payload
        new_length = len(new_payload)

        print(f"Old: {old_length}-> {old_payload}\n"
              f"New: {new_length}-> {new_payload}\n")
        settings_save_cmd = SettingsSaveCmd()
        usbl.write(settings_save_cmd.datagram())

        print('Updated settings in EEPROM')
    else:
        print('Settings already up-to-date')


def usbl_ping(usbl, beacon_id: int = 1) -> Tuple[XcvrUsblResp, XcvrFixResp]:
    """
    Ping a beacon, to determine whether it's operable.
    """

    ping_send_cmd = PingSendCmd(beacon_id)
    usbl.write(ping_send_cmd.datagram())
    #
    # Sending a ping to a remote beacon and receiving a response requires
    # round-trip acoustic communication. It's not a speedy process, so wait
    # a bit before looking for the responses.
    #
    sleep(2.0)

    ATTEMPTS = 1
    responses_received = 0
    while ATTEMPTS:
        try:
            datagram = usbl.read()

        except Exception:
            ATTEMPTS -= 1
            sleep(0.25)
            continue

        responses_received += 1
        if CID(datagram.cid) == CID.PING_SEND:
            PingSendResp(datagram, beacon_id)
        elif CID(datagram.cid) == CID.XCVR_USBL:
            xcvr_usbl = XcvrUsblResp(datagram, beacon_id)
        elif CID(datagram.cid) == CID.XCVR_FIX:
            xcvr_fix = XcvrFixResp(datagram, beacon_id)
        elif CID(datagram.cid) == CID.PING_RESP:
            PingRespResp(datagram, beacon_id)
        else:
            responses_received -= 1

        if responses_received == 4:
            return xcvr_usbl, xcvr_fix

    raise Exception("usbl_ping failed to collect the ping responses")
