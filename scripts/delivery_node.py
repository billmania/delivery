#!/usr/bin/env python3

from threading import Thread
from sys import exc_info
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import diagnostic_msgs
import diagnostic_updater
from delivery.script_utils import usbl_info, usbl_setup
from delivery.cid_callbacks import CIDCallbacks, CIDNotFound
from delivery.callback_funcs import CallbackFuncs
from delivery.usbl import USBL, NoDatagram
from delivery.usbl_messages import CID, PingSendCmd
from usbl_msgs.msg import RangeBearing


class AcommUsblNode(Node):
    """
    Use the X150 to continuously ping the X110 and determine its
    bearing and range. For each successful ping, publish a RangeBearing
    ROS message with the details.
    """

    def __init__(self):
        super().__init__('delivery_node')
        self.get_logger().info('ACOMM USBL node starting')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', Parameter.Type.STRING),
                ('data_rate', Parameter.Type.INTEGER),
                ('name', Parameter.Type.STRING),
                ('x150_id', Parameter.Type.INTEGER),
                ('x110_id', Parameter.Type.INTEGER),
                ('ping_period_s', Parameter.Type.INTEGER)
            ])

        parameters = self.get_parameters(['port',
                                          'data_rate',
                                          'name',
                                          'x150_id',
                                          'x110_id',
                                          'ping_period_s'])
        self._parameters = dict()
        self._parameters['port'] = parameters[0].value
        self._parameters['data_rate'] = parameters[1].value
        self._parameters['name'] = parameters[2].value
        self._parameters['x150_id'] = parameters[3].value
        self._parameters['x110_id'] = parameters[4].value
        self._parameters['ping_period_s'] = parameters[5].value

        self._usbl = None

        self.get_logger().info(
            f"port: {self._parameters['port']},"
            f" data_rate: {self._parameters['data_rate']},"
            f" name: {self._parameters['name']},"
            f" x110_id: {self._parameters['x110_id']}"
        )

        self._pings_sent = 0
        self._pings_response = 0
        self._pings_timeout = 0

    def _ping_diags(self, stat):
        stat.summary(
            diagnostic_msgs.msg.DiagnosticStatus.OK,
            'X150 connected')

        stat.add('pings_sent', f"{self._pings_sent}")
        stat.add('pings_response', f"{self._pings_response}")
        stat.add('pings_timeout', f"{self._pings_timeout}")

        return stat

    def _status_cb(self, status_msg: str):
        """
        A general purpose method for callback functions to use
        for returning status information.
        """
        self.get_logger().info(f"STATUS: {status_msg}")

    def _range_bearing_cb(self,
                          range_m: float,
                          azimuth_deg: float,
                          elevation_deg: float,
                          mag_heading_deg: float = None,
                          pitch_deg: float = None,
                          roll_deg: float = None,
                          depth_m: float = None,
                          remote_east_m: float = None,
                          remote_north_m: float = None,
                          remote_depth_m: float = None
                          ):
        """
        Called when an XcvrFix datagram is received. If the X110 doesn't
        respond to the PING, this method isn't called.
        """

        range_bearing = RangeBearing()
        range_bearing.header.stamp = self.get_clock().now().to_msg()
        range_bearing.orientation_present = False

        if (mag_heading_deg is not None
           and pitch_deg is not None
           and roll_deg is not None
           and depth_m is not None):
            range_bearing.orientation_present = True
            range_bearing.mag_heading_deg = mag_heading_deg
            range_bearing.pitch_deg = pitch_deg
            range_bearing.roll_deg = roll_deg
            range_bearing.depth_m = depth_m

        if range_m is not None:
            range_bearing.range_present = True
            #
            # There is intentional redundancy here.
            # The X150 uses the range, azimuth, and elevation
            # to calculate the remote east and north. remote_depth_m
            # is provided directly by the X110.
            #
            range_bearing.range_m = range_m
            range_bearing.azimuth_deg = azimuth_deg
            range_bearing.elevation_deg = elevation_deg
            range_bearing.remote_east_m = remote_east_m
            range_bearing.remote_north_m = remote_north_m
            range_bearing.remote_depth_m = remote_depth_m
        else:
            range_bearing.range_present = False

        self._range_bearing_pub.publish(range_bearing)

    def setup_device(self):
        """
        Initialize the connections to the device and configure it.
        """

        try:
            self._usbl = USBL('X150',
                              self._parameters['port'],
                              self._parameters['data_rate'])

        except Exception as e:
            self.get_logger().warn("setup_device failed to open USBL"
                                   f" at {self._parameters['port']}")
            raise e

        self.get_logger().info("setup_device waiting for X150 to respond")
        usbl_info(self._usbl)
        usbl_setup(self._usbl, beacon_id=self._parameters['x150_id'])
        self.get_logger().info("setup_device X150 configured")

        self._callbacks = CIDCallbacks()
        self._callback_funcs = CallbackFuncs(
            beacon_id=self._parameters['x110_id'],
            range_bearing_cb=self._range_bearing_cb,
            status_cb=self._status_cb)
        self._setup_callbacks()

        self._datagram_worker = Thread(
            target=self._handle_datagrams_in,
            daemon=True)
        self._datagram_worker.start()

        self._ping_timer = self.create_timer(self._parameters['ping_period_s'],
                                             self._ping)

        self.get_logger().info('ACOMM USBL node started')

    def _setup_callbacks(self):
        """
        Add the callback functions.
        """

        self._callbacks.set_callback(CID.SYS_INFO,
                                     self._callback_funcs.sys_info_cb)
        self._callbacks.set_callback(CID.PING_SEND,
                                     self._callback_funcs.ping_send_cb)
        self._callbacks.set_callback(CID.PING_RESP,
                                     self._callback_funcs.ping_resp_cb)
        self._callbacks.set_callback(CID.PING_ERROR,
                                     self._callback_funcs.ping_error_cb)
        self._callbacks.set_callback(CID.XCVR_FIX,
                                     self._callback_funcs.xcvr_fix_cb)
        self._callbacks.set_callback(CID.XCVR_USBL,
                                     self._callback_funcs.xcvr_usbl_cb)
        self._callbacks.set_callback(CID.SETTINGS_SET,
                                     self._callback_funcs.settings_set_resp_cb)
        self._callbacks.set_callback(
            CID.SETTINGS_SAVE,
            self._callback_funcs.settings_save_resp_cb)

        self._range_bearing_pub = self.create_publisher(RangeBearing,
                                                        'range_bearing', 10)

    def _ping(self) -> None:
        """
        Send one PING to the X110.
        """

        ping_send_cmd = PingSendCmd(self._parameters['x110_id'])
        self._usbl.write(ping_send_cmd.datagram())
        self._pings_sent += 1

    def _handle_datagrams_in(self):
        """
        A work function for reading inbound datagrams from the serial port,
        extracting the CID, and calling the associated callback.
        Expected to be used by a thread.
        """

        while True:
            try:
                datagram = self._usbl.read()

            except NoDatagram:
                #
                # This exception should be very rare.
                #
                self.get_logger().warn("_handle_datagrams_in: "
                                       f" Failed to read a datagram",
                                       throttle_duration_sec=60)
                continue

            except Exception:
                self.get_logger().err("_handle_datagrams_in: "
                                      f"{exc_info()[0]}({exc_info()[1]})"
                                      f" {exc_info()[2]}")
                continue

            try:
                result = self._callbacks.callback(datagram)
                if result:
                    self.get_logger().info(f" CID {CID(datagram.cid).name} "
                                           "callback result"
                                           f" <{result}>")

            except CIDNotFound:
                self.get_logger().warn("_handle_datagrams_in: "
                                       " No callback defined for CID"
                                       f" {hex(datagram.cid)}",
                                       throttle_duration_sec=60)

            except Exception:
                self.get_logger().err("_handle_datagrams_in: "
                                      f"{exc_info()[0]}({exc_info()[1]})"
                                      f" {exc_info()[2]}")

        return


def main(args=None):
    rclpy.init(args=args)
    node = AcommUsblNode()

    while True:
        try:
            node.setup_device()
            break

        except Exception:
            rclpy.spin_once(node, timeout_sec=5.0)

    _diag_updater = diagnostic_updater.Updater(node)
    _diag_updater.setHardwareID('usbl')
    _diag_updater.add('PING stats', node._ping_diags)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
