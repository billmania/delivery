"""
The callback functions for the USBL CIDs.
"""

from typing import Callable
from delivery.usbl_datagram import USBLDatagram
from delivery.usbl_messages import PingErrorResp
from delivery.usbl_messages import XcvrFixResp
from delivery.usbl_messages import SettingsSetResp
from delivery.usbl_messages import CST_E


class CallbackFuncs(object):

    def __init__(self,
                 beacon_id: int,
                 range_bearing_cb: Callable[[float, float, float], None],
                 status_cb: Callable[[str], None]):
        """
        Args:
            beacon_id: The ID of the X110 being ping-ed.
            range_bearing_cb: A function called when a valid range and bearing
                              update exists.
        """

        try:
            self._beacon_id = int(beacon_id)

        except TypeError:
            raise Exception('Must provide a beacon ID')

        if callable(range_bearing_cb):
            self._range_bearing_cb = range_bearing_cb
        else:
            raise Exception("Must provide a function for "
                            "range and bearing updates")
        if callable(status_cb):
            self._status_cb = status_cb
        else:
            raise Exception("Must provide a function for "
                            "status information")

    def sys_info_cb(self, datagram: USBLDatagram):
        self._status_cb("sys_info callback")

    def ping_send_cb(self, datagram: USBLDatagram):
        pass

    def settings_set_resp_cb(self, datagram: USBLDatagram):
        settings_resp = SettingsSetResp(datagram)
        try:
            status = CST_E(settings_resp.status)

        except ValueError:
            status = f"{settings_resp.status}"

        self._status_cb(f" SettingsSet status is {status}")

    def settings_save_resp_cb(self, datagram: USBLDatagram):
        #
        # The response is the same for the SETTINGS_SET and SETTINGS_SAVE.
        #
        save_resp = SettingsSetResp(datagram)
        try:
            status = CST_E(save_resp.status)

        except ValueError:
            status = f"{save_resp.status}"

        self._status_cb(f" SettingsSave status is {status}")

    def ping_error_cb(self, datagram: USBLDatagram):
        ping_error = PingErrorResp(datagram, self._beacon_id)
        try:
            error = CST_E(ping_error.status)

        except ValueError:
            error = f"{ping_error.status}"

        self._status_cb(f" PingError beacon {ping_error.relevant_beacon_id}"
                        f" status {error}")

    def xcvr_usbl_cb(self, datagram: USBLDatagram):
        pass

    def xcvr_fix_cb(self, datagram: USBLDatagram):
        """
        Called when a XcvrFix datagram has been received from the X150,
        after it received a PING response from the X110.
        """

        xcvr_fix = XcvrFixResp(datagram, self._beacon_id)
        self._range_bearing_cb(xcvr_fix.range_dist_m,
                               xcvr_fix.usbl_azimuth_deg,
                               xcvr_fix.usbl_elevation_deg,
                               xcvr_fix.attitude_yaw,
                               xcvr_fix.attitude_pitch,
                               xcvr_fix.attitude_roll,
                               xcvr_fix.depth_local,
                               xcvr_fix.remote_east_m,
                               xcvr_fix.remote_north_m,
                               xcvr_fix.remote_depth_m
                               )

    def ping_resp_cb(self, datagram: USBLDatagram):
        pass
