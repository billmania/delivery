#!/usr/bin/env python3

import sys
from threading import Thread
import TermTk as ttk

from delivery.usbl import USBL, NoDatagram
from delivery.usbl_messages import CID
from delivery.usbl_messages import StatusCfgSetCmd
from delivery.usbl_messages import CalActionCmd
from delivery.usbl_messages import StatusResp


class AHRSCalibrator(object):
    def __init__(self):
        self._show_accel_data = False
        self._show_mag_data = False
        self._setup()
        ttk.TTkLog.info("Calibrator started")

    def _setup(self):
        ttk.TTkLog.use_default_file_logging()

        self._grid = ttk.TTkGridLayout(columnMinHeight=1,
                                       columnMinWidth=1)
        self._root = ttk.TTk(layout=self._grid)

        self._instructions = ttk.TTkLabel(
            text="Press the Accelerometer Start button to begin",
            maxHeight=2,
            border=True)
        self._grid.addWidget(self._instructions,
                             row=0,
                             col=0,
                             colspan=6)

        self._status = ttk.TTkLabel(
            text="",
            maxHeight=2,
            border=True)
        self._grid.addWidget(self._status,
                             row=1,
                             col=0,
                             colspan=6)
        self._status.setText("Idle")

        self._setup_accelerometer(row=3)
        self._setup_magnetometer(row=6)
        self._setup_quit(row=8)
        self._usbl = USBL('X150',
                          sys.argv[1],
                          sys.argv[2])

    @ttk.pyTTkSlot()
    def _start_accel(self):
        self._status.setText('Starting accelerometer calibration')

        enable_acc_calibration = CalActionCmd(device='ACC', action='RESET')
        self._usbl.write(enable_acc_calibration.datagram())
        enable_status = StatusCfgSetCmd(enable=True)
        self._usbl.write(enable_status.datagram())

        self._data_thread = Thread(group=None,
                                   target=self._accel_data,
                                   name='AccelData')
        self._show_mag_data = False
        self._show_accel_data = True
        self._data_thread.start()

    @ttk.pyTTkSlot()
    def _start_mag(self):
        self._status.setText('Starting magnetometer calibration')

        enable_mag_calibration = CalActionCmd(device='MAG', action='RESET')
        self._usbl.write(enable_mag_calibration.datagram())
        enable_status = StatusCfgSetCmd(enable=True)
        self._usbl.write(enable_status.datagram())

        self._data_thread = Thread(group=None,
                                   target=self._mag_data,
                                   name='MagData')
        self._show_accel_data = False
        self._show_mag_data = True
        self._data_thread.start()

    def _accel_data(self):
        self._instructions.setText("X150 Z down, rotate about X"
                                   " completely, both directions."
                                   " Repeat with Y down about Z.")
        self._status.setText("Starting the accel_data worker")

        loop_count = 0

        while self._show_accel_data:
            loop_count += 1

            try:
                datagram = self._usbl.read()

            except NoDatagram:
                ttk.TTkLog.warn(f"{loop_count} No datagram")
                continue

            try:
                if CID(datagram.cid) == CID.STATUS:
                    status = StatusResp(datagram)

                    if status.have_ahrs_raw:
                        self._accel_X.setText(
                            f"{status.ahrs_raw_acc_x:0.3f}")
                        self._accel_Y.setText(
                            f"{status.ahrs_raw_acc_y:0.3f}")
                        self._accel_Z.setText(
                            f"{status.ahrs_raw_acc_z:0.3f}")
                    else:
                        ttk.TTkLog.warn(f"{status.timestamp_sec}: {loop_count}"
                                        " No raw AHRS data")
                else:
                    ttk.TTkLog.warn(f"{loop_count} Ignoring CID"
                                    f" {datagram.cid}")

            except Exception as e:
                ttk.TTkLog.warn(f"{loop_count} Exception {e}")
                ttk.TTkLog.warn(f"{loop_count}  datagram {datagram}")
                continue

        ttk.TTkLog.info("accel_data worker done")
        self._status.setText("")

    def _mag_data(self):
        self._instructions.setText("Completely rotate about X, Y, and Z")
        self._status.setText("Starting the mag_data worker")

        loop_count = 0
        while self._show_mag_data:
            loop_count += 1

            try:
                datagram = self._usbl.read()

            except NoDatagram:
                ttk.TTkLog.warn(f"{loop_count} No datagram")
                continue

            try:
                if CID(datagram.cid) == CID.STATUS:
                    status = StatusResp(datagram)

                    if status.have_ahrs_raw:
                        self._magnet_X.setText(
                            f"{status.ahrs_raw_mag_x:0.3f}")
                        self._magnet_Y.setText(
                            f"{status.ahrs_raw_mag_y:0.3f}")
                        self._magnet_Z.setText(
                            f"{status.ahrs_raw_mag_z:0.3f}")
                        self._status.setText("Buffer percent "
                                             f" {status.mag_buffer_percent}")
                    else:
                        ttk.TTkLog.warn(f"{status.timestamp_sec}: {loop_count}"
                                        " No raw AHRS data")
                else:
                    ttk.TTkLog.warn(f"{loop_count} Ignoring CID"
                                    f" {datagram.cid}")

            except Exception as e:
                ttk.TTkLog.warn(f"{loop_count} Exception {e}")
                ttk.TTkLog.warn(f"{loop_count}  datagram {datagram}")
                continue

        ttk.TTkLog.info("mag_data worker done")

    @ttk.pyTTkSlot()
    def _save_accel(self):
        self._show_accel_data = False

        save_acc_calibration = CalActionCmd(device='ACC', action='COMPUTE')
        self._usbl.write(save_acc_calibration.datagram())
        disable_status = StatusCfgSetCmd(enable=False)
        self._usbl.write(disable_status.datagram())

        self._accel_X.setText("")
        self._accel_Y.setText("")
        self._accel_Z.setText("")

        self._status.setText('Accelerometer calibration completed')

    @ttk.pyTTkSlot()
    def _save_mag(self):
        self._show_mag_data = False

        save_mag_calibration = CalActionCmd(device='MAG', action='COMPUTE')
        self._usbl.write(save_mag_calibration.datagram())
        disable_status = StatusCfgSetCmd(enable=False)
        self._usbl.write(disable_status.datagram())

        self._magnet_X.setText("")
        self._magnet_Y.setText("")
        self._magnet_Z.setText("")

        self._status.setText('Magnetometer calibration completed')

    @ttk.pyTTkSlot()
    def _quit(self):
        self._status.setText('Shutdown')
        self._show_accel_data = False
        self._show_mag_data = False
        sys.exit(0)

    def _setup_accelerometer(self, row: int):
        self._accelerometer_label = ttk.TTkLabel(
            parent=self._root,
            text="Accelerometer")
        self._grid.addWidget(self._accelerometer_label,
                             row=row,
                             col=0,
                             colspan=2)
        self._accelerometer_start = ttk.TTkButton(
            parent=self._root,
            text="Start")
        self._grid.addWidget(self._accelerometer_start,
                             row=row,
                             col=2,
                             colspan=2)
        self._accelerometer_start.clicked.connect(self._start_accel)
        self._accelerometer_save = ttk.TTkButton(
            parent=self._root,
            text="Save")
        self._grid.addWidget(self._accelerometer_save,
                             row=row,
                             col=4,
                             colspan=2)
        self._accelerometer_save.clicked.connect(self._save_accel)

        self._accel_X_label = ttk.TTkLabel(
            parent=self._root,
            text="X")
        self._grid.addWidget(self._accel_X_label,
                             row=row+1,
                             col=0)
        self._accel_X = ttk.TTkLabel(
            parent=self._root,
            text="0.0")
        self._grid.addWidget(self._accel_X,
                             row=row+1,
                             col=1)
        self._accel_Y_label = ttk.TTkLabel(
            parent=self._root,
            text="Y")
        self._grid.addWidget(self._accel_Y_label,
                             row=row+1,
                             col=2)
        self._accel_Y = ttk.TTkLabel(
            parent=self._root,
            text="0.0")
        self._grid.addWidget(self._accel_Y,
                             row=row+1,
                             col=3)
        self._accel_Z_label = ttk.TTkLabel(
            parent=self._root,
            text="Z")
        self._grid.addWidget(self._accel_Z_label,
                             row=row+1,
                             col=4)
        self._accel_Z = ttk.TTkLabel(
            parent=self._root,
            text="0.0")
        self._grid.addWidget(self._accel_Z,
                             row=row+1,
                             col=5)

    def _setup_magnetometer(self, row: int):
        self._magnetometer_label = ttk.TTkLabel(
            parent=self._root,
            text="Magnetometer")
        self._grid.addWidget(self._magnetometer_label,
                             row=row,
                             col=0,
                             colspan=2)
        self._magnetometer_start = ttk.TTkButton(
            parent=self._root,
            text="Start")
        self._grid.addWidget(self._magnetometer_start,
                             row=row,
                             col=2,
                             colspan=2)
        self._magnetometer_start.clicked.connect(self._start_mag)
        self._magnetometer_save = ttk.TTkButton(
            parent=self._root,
            text="Save")
        self._grid.addWidget(self._magnetometer_save,
                             row=row,
                             col=4,
                             colspan=2)
        self._magnetometer_save.clicked.connect(self._save_mag)

        self._magnet_X_label = ttk.TTkLabel(
            parent=self._root,
            text="X")
        self._grid.addWidget(self._magnet_X_label,
                             row=row+1,
                             col=0)
        self._magnet_X = ttk.TTkLabel(
            parent=self._root,
            text="0.0")
        self._grid.addWidget(self._magnet_X,
                             row=row+1,
                             col=1)
        self._magnet_Y_label = ttk.TTkLabel(
            parent=self._root,
            text="Y")
        self._grid.addWidget(self._magnet_Y_label,
                             row=row+1,
                             col=2)
        self._magnet_Y = ttk.TTkLabel(
            parent=self._root,
            text="0.0")
        self._grid.addWidget(self._magnet_Y,
                             row=row+1,
                             col=3)
        self._magnet_Z_label = ttk.TTkLabel(
            parent=self._root,
            text="Z")
        self._grid.addWidget(self._magnet_Z_label,
                             row=row+1,
                             col=4)
        self._magnet_Z = ttk.TTkLabel(
            parent=self._root,
            text="0.0")
        self._grid.addWidget(self._magnet_Z,
                             row=row+1,
                             col=5)

    def _setup_quit(self, row: int):
        self._quit_button = ttk.TTkButton(
            parent=self._root,
            text="Quit")
        self._grid.addWidget(self._quit_button,
                             row=row,
                             col=4,
                             colspan=2)
        self._quit_button.clicked.connect(self._quit)

    def _run(self):
        self._root.mainloop()


if __name__ == '__main__':
    ahrs_calibrator = AHRSCalibrator()

    ahrs_calibrator._run()
