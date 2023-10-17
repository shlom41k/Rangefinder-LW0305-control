import sys
import re
import serial
import serial.tools.list_ports

from PyQt6 import uic, QtCore, QtGui, QtWidgets
from datetime import datetime
from time import sleep
from threading import Thread

# from pympler import muppy
# all_objects = muppy.get_objects()


class MainWindow(QtWidgets.QMainWindow):

    # COM port parameters
    com_baudrate = [19200, 9600, 4800, 2400, 1200, ]

    com_parity = {
        "None": serial.PARITY_NONE,
    }

    com_bytesize = {
        8: serial.EIGHTBITS,
    }

    com_stopbits = {
        1: serial.STOPBITS_ONE,
    }

    # M0305 Device address
    dev_addr = 0x10

    # M0305 Commands
    commands = {
        "cmd_80": 0x80,
        "cmd_81": 0x81,
        "cmd_83": 0x83,
        "cmd_84": 0x84,
        "cmd_8f": 0x8F,
        "cmd_9e": 0x9E,
        "cmd_be": 0xBE,
        "cmd_c0": 0xC0,
        "cmd_c1": 0xC1,
        "cmd_e0": 0xE0,
    }

    # Delay between command for start randing
    # and command for reading measure results
    single_shot_delay = 1.0

    # Delay between request and response
    com_delay = 0.1

    ShotsThread: Thread

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        uic.loadUi("MBR_cntrl.ui", self)

        # Set main window title
        self.setWindowTitle("M0305 rangefinder control pannel")

        # Get all available COM ports
        self.com_ports = None

        # COM port for work with rangefinder
        self.com = None

        # Logging flags
        self.info_log = True
        self.shot_log = True

        # Single shot flag
        self.single_shot = False

        # Shots counter
        self.shot_cnt = 0

        # Series of shots flag
        self.serial_shots = False

        # Default measure export method (1 - Auto, 0 - Query)
        self.export_method = 0x01

        # Set default slave address
        self.x9Ex91_line_ed.setText(str(self.dev_addr))

        # Write COM port settings
        self.clear_com_settings()
        self.set_com_settings()
        self.updete_com_ports_list()

        # Connect events handler with fulctions
        # buttons
        self.com_pb_update.clicked.connect(self.updete_com_ports_list)
        self.com_pb_connect.clicked.connect(self.device_connect)
        self.com_pb_disconnect.clicked.connect(self.device_disconnect)
        self.pb_clear_hist.clicked.connect(self.clear_history)

        self.shot_send_pb_2.clicked.connect(self.cmd_single_shot)
        self.shot_send_pb_3.clicked.connect(self.cmd_single_shot)

        self.ser_shot_start_pb.clicked.connect(self.cmd_start_serial)
        self.ser_shot_stop_pb.clicked.connect(self.cmd_stop_serial)

        self.x81_clr_pb.clicked.connect(self.clear_x81)
        self.x81_clr_pb_2.clicked.connect(self.clear_x81)
        self.x80_send_pb.clicked.connect(self.cmd_80)
        self.x81_send_pb.clicked.connect(self.cmd_81)
        self.x83_send_pb.clicked.connect(self.cmd_83)
        self.x84_send_pb.clicked.connect(self.cmd_84)
        self.x8F_send_pb.clicked.connect(self.cmd_8f)
        self.x9E_send_pb_0.clicked.connect(self.cmd_9e_90)
        self.x9E_send_pb_1.clicked.connect(self.cmd_9e_91)
        self.x9E_send_pb_2.clicked.connect(self.cmd_9e_92)
        self.x9E_send_pb_3.clicked.connect(self.cmd_9e_93)
        self.xBE_send_pb.clicked.connect(self.cmd_be)
        self.xC0_send_pb.clicked.connect(self.cmd_c0)
        self.xC1_send_pb.clicked.connect(self.cmd_c1)
        self.xE0_send_pb.clicked.connect(self.cmd_e0)

        # combo boxes
        self.com_combo_box_portname.currentTextChanged.connect(self.select_com_port)

        # check boxes
        self.info_log_chbox.stateChanged.connect(self.en_dis_info_log)
        self.shot_log_chbox.stateChanged.connect(self.en_dis_measure_log)

        # spin boxes
        self.shot_delay_box.valueChanged.connect(self.set_shot_delay)

        # Enable logging
        self.info_log_chbox.setChecked(True)
        self.shot_log_chbox.setChecked(True)

        # Disable rangefinder control pannel
        self.mbr_tab.setDisabled(True)
        self.measure_group_box.setDisabled(True)
        self.series_group_box.setDisabled(True)

        # Set shot delay box
        self.shot_delay_box.setRange(0.5, 10)
        self.shot_delay_box.setValue(1.0)
        # Current shot label
        self.ser_shot_ed.clear()

        # History list
        self.model = QtGui.QStandardItemModel()
        self.hist_list.setModel(self.model)

    def disp_info(self, msg_type: str, message: str):
        # Send some info into message field
        msg_types = {
            "INFO": "blue",
            "ERROR": "red",
        }
        msg = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | {msg_type}: {message}"

        self.info_lbl.setText(msg)
        self.info_lbl.setStyleSheet(f"color: {msg_types.get(msg_type)}")
        # self.hist_list.addItem(msg)
        # self.hist_list.scrollToBottom()
        self.model.appendRow(QtGui.QStandardItem(msg))
        self.hist_list.scrollToBottom()

    def clear_history(self):
        # self.hist_list.clear()
        self.model.clear()

    def clear_x80(self):
        self.x80_resp_d7_lbl.setText("bit 7")
        self.x80_resp_d7_lbl.setStyleSheet("color: black")
        self.x80_resp_d6_lbl.setText("bit 6")
        self.x80_resp_d6_lbl.setStyleSheet("color: black")
        self.x80_resp_d1_lbl.setText("bit 1")
        self.x80_resp_d1_lbl.setStyleSheet("color: black")
        self.x80_resp_d0_lbl.setText("bit 0")
        self.x80_resp_d0_lbl.setStyleSheet("color: black")

    def clear_x81(self):
        self.x81_resp_d7_lbl.setText("bit 7")
        self.x81_resp_d7_lbl.setStyleSheet("color: black")
        self.x81_resp_d6_lbl.setText("bit 6")
        self.x81_resp_d6_lbl.setStyleSheet("color: black")
        self.x81_resp_d5_lbl.setText("bit 5")
        self.x81_resp_d5_lbl.setStyleSheet("color: black")
        self.x81_resp_d4_lbl.setText("bit 4")
        self.x81_resp_d4_lbl.setStyleSheet("color: black")
        self.x81_resp_d3_lbl.setText("bit 3...0")
        self.x81_resp_d3_lbl.setStyleSheet("color: black")
        self.x81_dist_line_ed.clear()
        self.x81_dist_line_ed_2.clear()
        self.x81_angle_line_ed.clear()
        self.x81_angle_line_ed_2.clear()
        self.x81_speed_line_ed.clear()

    def clear_xbe(self):
        self.xBE_b2_line_ed.clear()
        self.xBE_b1_line_ed.clear()
        self.xBE_b2_line_ed_2.clear()
        self.xBE_b4_line_ed.clear()

    def clear_xc0(self):
        self.xC0_resp_b0_lbl.clear()
        self.xC0_resp_b0_lbl.setStyleSheet("background-color: #d4d4d4")

    def clear_xc1(self):
        self.xC1_resp_b0_lbl.clear()

    def clear_xe0(self):
        self.xE0_resp_d2_lbl.setText("bit 2")
        self.xE0_resp_d2_lbl.setStyleSheet("color: black")
        self.xE0_resp_d1_lbl.setText("bit 1")
        self.xE0_resp_d1_lbl.setStyleSheet("color: black")
        self.xE0_resp_d0_lbl.setText("bit 0")
        self.xE0_resp_d0_lbl.setStyleSheet("color: black")

    def clear_com_settings(self):
        self.com_combo_box_baud.clear()
        self.com_combo_box_bytesize.clear()
        self.com_combo_box_stopbits.clear()
        self.com_combo_box_parity.clear()

    def set_com_settings(self):
        self.com_combo_box_baud.addItems(map(str, self.__class__.com_baudrate))
        self.com_combo_box_bytesize.addItems(map(str, list(self.__class__.com_bytesize.keys())))
        self.com_combo_box_stopbits.addItems(map(str, list(self.__class__.com_stopbits.keys())))
        self.com_combo_box_parity.addItems(map(str, list(self.__class__.com_parity.keys())))

    def disable_com_settings(self):
        self.com_pb_update.setDisabled(True)
        self.com_combo_box_portname.setDisabled(True)
        self.com_combo_box_baud.setDisabled(True)
        self.com_combo_box_bytesize.setDisabled(True)
        self.com_combo_box_stopbits.setDisabled(True)
        self.com_combo_box_parity.setDisabled(True)

    def enable_com_settings(self):
        self.com_pb_update.setEnabled(True)
        self.com_combo_box_portname.setEnabled(True)
        self.com_combo_box_baud.setEnabled(True)
        self.com_combo_box_bytesize.setEnabled(True)
        self.com_combo_box_stopbits.setEnabled(True)
        self.com_combo_box_parity.setEnabled(True)

    def updete_com_ports_list(self):
        # Update list of available COM ports
        self.com_combo_box_portname.clear()
        self.com_ports = self.__class__.get_com_ports()

        if self.com_ports:
            self.com_combo_box_portname.addItems(list(self.com_ports.keys()))
            self.com_lbl_portname_2.setText(self.com_ports.get(self.com_combo_box_portname.currentText()))
            self.com_lbl_portname_2.setStyleSheet("color: blue")
            self.com_pb_connect.setEnabled(True)
            self.com_pb_disconnect.setEnabled(True)
        else:
            self.com_lbl_portname_2.setText("No available COM ports :(")
            self.com_lbl_portname_2.setStyleSheet("color: red")
            self.com_pb_connect.setDisabled(True)
            self.com_pb_disconnect.setDisabled(True)

    def select_com_port(self):
        # Select COM port
        self.com_lbl_portname_2.setText(self.com_ports.get(self.com_combo_box_portname.currentText()))

    def device_connect(self):
        # Create COM object
        try:
            self.com = serial.Serial(
                port=self.com_combo_box_portname.currentText(),
                baudrate=int(self.com_combo_box_baud.currentText()),
                bytesize=self.com_bytesize.get(int(self.com_combo_box_bytesize.currentText())),
                stopbits=self.com_stopbits.get(int(self.com_combo_box_stopbits.currentText())),
                parity=self.com_parity.get(self.com_combo_box_parity.currentText()),
                writeTimeout=2,
                timeout=0,
            )
            self.disp_info("INFO", "COM port created")
        except:
            self.disp_info("ERROR", "Can't create serial port")
            return

        # Connect to COM port
        try:
            if not self.com.isOpen():
                self.com.open()
                sleep(self.com_delay)

            self.com.reset_input_buffer()
            self.com.reset_output_buffer()
            self.com.flushInput()
            self.com.flushOutput()
            sleep(self.com_delay)

            self.disp_info("INFO", "COM port connected")
        except:
            self.disp_info("ERROR", "Can't open serial port")
            return

        # GUI
        self.com_lbl_portname_2.setStyleSheet("color: green")
        self.com_pb_connect.setDisabled(True)
        self.disable_com_settings()
        # Enable rangefinder control panel
        self.mbr_tab.setEnabled(True)
        self.measure_group_box.setEnabled(True)
        self.series_group_box.setEnabled(True)
        # Put down single shot flag
        self.single_shot = False

    def device_disconnect(self):
        # Disconnect via COM port
        try:
            if self.com.isOpen():
                self.com.close()

            self.disp_info("INFO", "COM port closed")
            self.com_lbl_portname_2.setStyleSheet("color: blue")
            self.com_pb_connect.setEnabled(True)
            self.enable_com_settings()
            # Disable rangefinder control panel
            self.mbr_tab.setDisabled(True)
            self.measure_group_box.setDisabled(True)
            self.series_group_box.setDisabled(True)
            # Clear rangefinder control panel
            self.clear_x80()
            self.clear_x81()
            self.clear_xbe()
            self.clear_xc0()
            self.clear_xc1()
            self.clear_xe0()
            # Put down single shot flag
            self.single_shot = False
            # Stop serial measuring
            self.cmd_stop_serial()
        except:
            self.disp_info("ERROR", "Can't close serial port!")
            self.com_lbl_portname_2.setStyleSheet("color: red")

    def com_write(self, dev: int, message: list, crc: int):
        msg = bytearray([dev, *message, crc])
        self.com.write(msg)
        #sleep(0.1)

    def com_read(self):
        # while com.inWaiting():
        return self.com.readall().hex()

    def en_dis_info_log(self):
        # Enable/Disable info displaying
        self.info_log = True if self.info_log_chbox.isChecked() else False

    def en_dis_measure_log(self):
        # Enable/Disable measure results displaying
        self.shot_log = True if self.shot_log_chbox.isChecked() else False

    def set_shot_delay(self):
        # Set shot delay
        self.single_shot_delay = self.shot_delay_box.value()

    def cmd_start_serial(self):
        # Command for start serial measure

        # Disable control panels
        self.ser_shot_spbox.setDisabled(True)
        self.ser_shot_start_pb.setDisabled(True)
        self.mbr_tab.setDisabled(True)
        self.measure_group_box.setDisabled(True)

        # Disable logging
        self.info_log_chbox.setChecked(False)
        # self.shot_log_chbox.setChecked(False)

        # Put up serial shots flag
        self.serial_shots = True

        # Run measure in Thread
        self.ShotsThread = Thread(target=self.serial_shots_measure, daemon=True)
        self.ShotsThread.start()

    def cmd_stop_serial(self):
        # Command for stop serial measure

        # Put down serial shots flag
        self.serial_shots = False

        # Enable control panels
        self.ser_shot_spbox.setEnabled(True)
        self.ser_shot_start_pb.setEnabled(True)
        self.mbr_tab.setEnabled(True)
        self.measure_group_box.setEnabled(True)
        # Clear current shot label
        self.ser_shot_ed.clear()

    def serial_shots_measure(self):
        # Serial measure func
        self.shot_cnt = 0

        # Shots in cycle
        while (self.shot_cnt < self.ser_shot_spbox.value()) and self.serial_shots:
            self.shot_cnt += 1

            self.ser_shot_ed.setText(str(self.shot_cnt))
            self.cmd_single_shot()

        self.cmd_stop_serial()

    def cmd_single_shot(self):
        # Set shot delay
        # self.single_shot_delay = self.shot_delay_box.value()

        # Command for single shot
        cmd = [self.commands.get("cmd_83"), 0x00]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print(hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, cmd, crc)
            if self.info_log:
                self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {cmd}, CRC: {hex(crc)}")

            # Up single_shot flag
            self.single_shot = True

            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")
            return

        # Get ACK from rangefinder (start RANGING)
        try:
            ans = self.com_read()
            # print(ans)

            if self.com_resp_check(ans, 6):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    if self.info_log:
                        self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    # Wait for measure
                    sleep(self.single_shot_delay)

                    # If rangefinder automatically send measure result
                    if self.export_method == 0x01:
                        # Go to 0x81 command (receive part)
                        self.cmd_81()
                    else:
                        self.single_shot = False
                else:
                    self.disp_info("ERROR", "Invalid CRC")
            else:
                self.disp_info("ERROR", "Wrong answer from device")
        except:
            self.disp_info("ERROR", "No answer from device!")

    def cmd_80(self):
        cmd = self.commands.get("cmd_80")

        # Calculate CRC
        crc = self.crc_calc(cmd)
        # print(hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, [cmd], crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {hex(cmd)}, CRC: {hex(crc)}")
            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")

        # Get answer
        try:
            ans = self.com_read()
            # print("Response: ", ans)

            self.x80_resp_line_ed.setText(f"{int(ans, 16):#x}")

            # Response must have 5 bytes [dev_id, 0x20, data(2 bytes), crc]
            if self.com_resp_check(ans, 10):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)
                # print(msg, hex(dev_crc))
                if self.crc_check(msg, dev_crc):
                    self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    msg_byte0 = int(msg[2:4], 16)
                    msg_byte0_str = self.byte_to_string(msg_byte0)

                    # Bit 0
                    if msg_byte0_str[0] == "1":
                        self.x80_resp_d0_lbl.setText("1 - Angle sensor has some error")
                        self.x80_resp_d0_lbl.setStyleSheet("color: red")
                    else:
                        self.x80_resp_d0_lbl.setText("0 - Angle sensor has no errors")
                        self.x80_resp_d0_lbl.setStyleSheet("color: green")
                    # Bit 1
                    if msg_byte0_str[1] == "1":
                        self.x80_resp_d1_lbl.setText("1 - Angle sensor enabled")
                    else:
                        self.x80_resp_d1_lbl.setText("0 - Angle sensor disabled")
                    # Bit 2...5 - reserved
                    # Bit 6
                    if msg_byte0_str[6] == "1":
                        self.x80_resp_d6_lbl.setText("1 - Module has some error")
                        self.x80_resp_d6_lbl.setStyleSheet("color: red")
                    else:
                        self.x80_resp_d6_lbl.setText("0 - Module has no errors")
                        self.x80_resp_d6_lbl.setStyleSheet("color: green")
                    # Bit 7
                    if msg_byte0_str[7] == "1":
                        self.x80_resp_d7_lbl.setText("1 - Busy (randing in process)")
                    else:
                        self.x80_resp_d7_lbl.setText("0 - Randing finished")
                else:
                    self.disp_info("ERROR", "Invalid CRC")
                    self.clear_x80()
            else:
                self.disp_info("ERROR", "Invalid answer")
                self.clear_x80()
        except:
            self.disp_info("ERROR", "No answer from device!")
            self.x80_resp_line_ed.clear()
            self.clear_x80()

    def cmd_81(self):
        # When command 0x81 only
        if not self.single_shot:
            # print("0x81 command only")
            cmd = self.commands.get("cmd_81")

            # Calculate CRC
            crc = self.crc_calc(cmd)
            # print(hex(crc))

            # Send command
            try:
                self.com_write(self.dev_addr, [cmd], crc)
                if self.info_log:
                    self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {hex(cmd)}, CRC: {hex(crc)}")
                sleep(self.com_delay)
            except:
                self.disp_info("ERROR", "Can't send message")

        # Put down single shot flag
        self.single_shot = False

        # Get answer
        try:
            ans = self.com_read()
            # print("Response: ", ans)

            self.x81_resp_line_ed.setText(f"{int(ans, 16):#x}")

            # Response must have 6 or 7 bytes [dev_id, 0x01, data(3 or 4 bytes), crc]
            if self.com_resp_check(ans, 12) or self.com_resp_check(ans, 14):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    if self.info_log:
                        self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    # Message byte0
                    msg_byte0 = int(msg[2:4], 16)
                    msg_byte0_str = self.byte_to_string(msg_byte0)

                    # Bit 3...0 byte 0
                    if msg_byte0_str[0] == "1":
                        self.x81_resp_d3_lbl.setText("1 - Working mode: SPEED")
                        # self.x81_resp_d3_lbl.setStyleSheet("color: red")
                    else:
                        self.x81_resp_d3_lbl.setText("0 - Working mode: RANGING")
                        # self.x81_resp_d3_lbl.setStyleSheet("color: green")
                    # Bit 4
                    if msg_byte0_str[4] == "1":
                        self.x81_resp_d4_lbl.setText("1 - Data unit: YARD")
                        self.x81_b1_unit_lbl.setText("yard")
                    else:
                        self.x81_resp_d4_lbl.setText("0 - Data unit: METER")
                        self.x81_b1_unit_lbl.setText("meter")
                    # Bit 5
                    if msg_byte0_str[5] == "1":
                        self.x81_resp_d5_lbl.setText("1 - Data resolution: 0.1/LSB")
                    else:
                        self.x81_resp_d5_lbl.setText("0 - Data resolution: 0.5/LSB")
                    # Bit 6
                    if msg_byte0_str[6] == "1":
                        self.x81_resp_d6_lbl.setText("1 - Angle is invalid")
                        self.x81_resp_d6_lbl.setStyleSheet("color: red")
                    else:
                        self.x81_resp_d6_lbl.setText("0 - Angle is valid")
                        self.x81_resp_d6_lbl.setStyleSheet("color: green")
                    # Bit 7
                    if msg_byte0_str[7] == "1":
                        self.x81_resp_d7_lbl.setText("1 - Distance is invalid")
                        self.x81_resp_d7_lbl.setStyleSheet("color: red")
                    else:
                        self.x81_resp_d7_lbl.setText("0 - Distance is valid")
                        self.x81_resp_d7_lbl.setStyleSheet("color: green")

                    # When RANGING measure mode:
                    if msg_byte0_str[0] == "0":

                        # When distance is valid:
                        if msg_byte0_str[7] == "0":
                            # msg_byte1 and msg_byte2 is a data
                            mult_fact = 0.1 if msg_byte0_str[5] == "1" else 0.5
                            distance = round(int(msg[4:8], 16) * mult_fact, 3)
                            # print(distance)
                            self.x81_dist_line_ed.setText(f"{distance}")
                            self.x81_dist_line_ed_2.setText(f"{distance}")

                            # Angle when distance is valid
                            angle = int(msg[8:10], 16)

                            # When log enable
                            if self.shot_log:
                                if self.serial_shots:
                                    msg = f"Shot: {self.shot_cnt}, total: {self.ser_shot_spbox.value()}, " \
                                          f"DISTANCE: {distance} {'m' if msg_byte0_str[4] == '0' else 'y'}, " \
                                          f"angle: {angle - 256 if angle >= 128 else angle} deg"
                                else:
                                    msg = f"DISTANCE: {distance} {'m' if msg_byte0_str[4] == '0' else 'y'}, " \
                                          f"angle: {angle - 256 if angle >= 128 else angle} deg"

                                self.disp_info("INFO", msg)
                        else:
                            # Ranging no result
                            if int(msg[4:6], 16) == 0x80:
                                # print("Ranging no result")
                                self.x81_dist_line_ed.setText(f"No result")
                                self.x81_dist_line_ed_2.setText(f"No result")
                            # System error
                            elif int(msg[4:6], 16) == 0x81:
                                self.x81_dist_line_ed.setText(f"System error")
                                self.x81_dist_line_ed_2.setText(f"System error")
                                # print("System error")
                            else:
                                self.x81_dist_line_ed.clear()
                                self.x81_dist_line_ed_2.clear()
                                # print("Unknown code")

                            # Angle when distance is invalid
                            angle = int(msg[6:8], 16)

                            # When log enable
                            if self.shot_log:
                                if self.serial_shots:
                                    msg = f"Shot: {self.shot_cnt}, total: {self.ser_shot_spbox.value()}, " \
                                          f"Distance INVALID, angle: {angle - 256 if angle >= 128 else angle} deg"
                                else:
                                    msg = f"Distance INVALID, angle: {angle - 256 if angle >= 128 else angle} deg"

                                self.disp_info("INFO", msg)

                        # Angle field
                        # print(angle)
                        self.x81_angle_line_ed.setText(f"{angle - 256 if angle >= 128 else angle}")
                        self.x81_angle_line_ed_2.setText(f"{angle - 256 if angle >= 128 else angle}")
                        # Speed field
                        self.x81_speed_line_ed.setText("---")
                    else:
                        # SPEED measure mode

                        # msg_byte1 and msg_byte2 is a data
                        mult_fact = 0.1 if msg_byte0_str[5] == "1" else 0.5
                        speed = round(int(msg[4:8], 16) * mult_fact, 3)
                        # print(speed)
                        self.x81_speed_line_ed.setText(f"{speed}")

                        # When log enable
                        if self.shot_log:
                            if self.serial_shots:
                                msg = f"Shot: {self.shot_cnt}, total: {self.ser_shot_spbox.value()}, SPEED: {speed} km/h"
                            else:
                                msg = f"SPEED: {speed} km/h"

                            self.disp_info("INFO", msg)

                        # Angle field
                        self.x81_angle_line_ed.setText("---")
                        self.x81_angle_line_ed_2.setText("---")
                        # Distance field
                        self.x81_dist_line_ed.setText("---")
                else:
                    self.disp_info("ERROR", "Invalid CRC")
                    self.clear_x81()
            else:
                self.disp_info("ERROR", "Invalid answer")
                self.clear_x81()
        except:
            self.disp_info("ERROR", "No answer from device!")
            self.x81_resp_line_ed.clear()
            self.clear_x81()

    def cmd_83(self):

        # When SPEED mode, delay for measure must be ~3 sec
        self.single_shot_delay = 3 if (self.x83_d3_cb.currentText() == "Speed" and self.shot_delay_box.value() < 3) else self.shot_delay_box.value()

        # Only RANGING (0x00) and SPEED (0x01) modes support
        mode = 0x01 if self.x83_d3_cb.currentText() == "Speed" else 0x00

        # Command parameter (not used D7...D6 command bits)
        param = mode + 0x10 if self.x83_d4_cb.currentText() == "Yard" else mode

        # Command for START ranging
        cmd = [self.commands.get("cmd_83"), param]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print("CRC: ", hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, cmd, crc)
            if self.info_log:
                self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {cmd}, CRC: {hex(crc)}")

            # Up single_shot flag
            self.single_shot = True

            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")
            return

        # Get ACK from rangefinder (start RANGING)
        try:
            ans = self.com_read()
            # print(ans)

            self.x83_resp_line_ed.setText(f"{int(ans, 16):#x}")

            if self.com_resp_check(ans, 6):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    if self.info_log:
                        self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    # Wait for measure
                    sleep(self.single_shot_delay)

                    # If rangefinder automatically send measure result
                    if self.export_method == 0x01:
                        # Go to 0x81 command (receive part)
                        self.cmd_81()
                    else:
                        self.single_shot = False
                else:
                    self.disp_info("ERROR", "Invalid CRC")
            else:
                self.disp_info("ERROR", "Wrong answer from device")
        except:
            self.disp_info("ERROR", "No answer from device!")
            self.x83_resp_line_ed.clear()

    def cmd_84(self):
        # Command for STOP ranging
        cmd = self.commands.get("cmd_84")

        # Calculate CRC
        crc = self.crc_calc(cmd)
        # print(hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, [cmd], crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {hex(cmd)}, CRC: {hex(crc)}")
            sleep(self.com_delay)

            self.x84_resp_line_ed.setText(f"Without response")
        except:
            self.disp_info("ERROR", "Can't send message")
            self.x84_resp_line_ed.clear()

    def cmd_8f(self):
        red_las = 0x00 if self.x8F_b0_cb.currentText() == "OFF" else 0x01
        cmd = [self.commands.get("cmd_8f"), red_las]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print("CRC: ", hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, cmd, crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {cmd}, CRC: {hex(crc)}")
            sleep(self.com_delay)
            self.x8F_resp_line_ed.setText(f"Without response")
        except:
            self.disp_info("ERROR", "Can't send message")

    def cmd_9e_90(self):
        bauds = {
            "1200": 0x00,
            "2400": 0x01,
            "4800": 0x02,
            "9600": 0x03,
            "19200": 0x04,
        }
        baud_param = bauds.get(self.x9Ex90_b4_cb.currentText())
        # Command for set baudrate
        cmd = [self.commands.get("cmd_9e"), 0x90, 0x00, 0x00, baud_param]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print("CRC: ", hex(crc))

        # TBD. Now this command not used
        self.disp_info("INFO", "This command LOCKED")

    def cmd_9e_91(self):
        # Get new slave address
        slv_addr = self.x9Ex91_line_ed.text()

        if slv_addr.isdigit() and (0 < len(slv_addr) <= 3):
            new_addr = int(slv_addr)

            if 0x01 <= new_addr <= 0xFF:
                # Command for set new slave address
                cmd = [self.commands.get("cmd_9e"), 0x91, 0x00, 0x00, new_addr]

                # Calculate CRC
                crc = self.crc_calc(*cmd)
                # print("CRC: ", hex(crc))
            else:
                self.disp_info("ERROR", "Invalid new slave address. Must be in 0x01...0xFF")
        else:
            self.disp_info("ERROR", "Invalid new slave address")

        # TBD. Now this command not used
        self.disp_info("INFO", "This command LOCKED")

    def cmd_9e_92(self):
        # Get export method
        method = 0x01 if self.x9Ex92_b4_cb.currentText() == "Auto" else 0x00

        # Command for set export method
        cmd = [self.commands.get("cmd_9e"), 0x92, 0x00, 0x00, method]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print("CRC: ", hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, cmd, crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {cmd}, CRC: {hex(crc)}")
            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")

        # Get ACK from rangefinder
        try:
            ans = self.com_read()
            # print(ans)

            self.x9E_resp_line_ed.setText(f"{int(ans, 16):#x}")

            if self.com_resp_check(ans, 6):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    # Change export method
                    self.export_method = method
                else:
                    self.disp_info("ERROR", "Invalid CRC")
            else:
                self.disp_info("ERROR", "Wrong answer from device")
        except:
            self.disp_info("ERROR", "No answer from device!")

    def cmd_9e_93(self):

        lsb = 0.025
        uart_vio = {
            "1.8 V": int(round(1.8 / lsb)),
            "2.5 V": int(round(2.5 / lsb)),
            "3.3 V": int(round(3.3 / lsb)),
            "5 V": int(round(5.0 / lsb)),
        }

        # Get UART I/O Voltage
        vio = uart_vio.get(self.x9Ex92_b4_cb_2.currentText())

        # Command for set UART VIO
        cmd = [self.commands.get("cmd_9e"), 0x93, 0x00, 0x00, vio]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print("CRC: ", hex(crc))

        # TBD. Now this command not used
        self.disp_info("INFO", "This command LOCKED")

    def cmd_be(self):
        cmd = [self.commands.get("cmd_be"), 0x91]

        # Calculate CRC
        crc = self.crc_calc(*cmd)
        # print("CRC: ", hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, cmd, crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {cmd}, CRC: {hex(crc)}")
            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")

        # Get answer
        try:
            ans = self.com_read()
            # print("Response: ", ans)

            self.xBE_resp_line_ed.setText(f"{int(ans, 16):#x}")

            # Response must have 6 bytes [dev_id, 0x11, data(3 bytes), crc]
            if self.com_resp_check(ans, 12):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    # Message byte0 - Slave address
                    msg_byte0 = int(msg[2:4], 16)
                    self.xBE_b1_line_ed.setText(hex(msg_byte0))

                    # Message byte1
                    msg_byte1 = int(msg[4:6], 16)
                    msg_byte1_str = self.byte_to_string(msg_byte1)

                    # Bit 4
                    if msg_byte1_str[4] == "1":
                        self.xBE_b2_line_ed_2.setText("1 - Auto")
                    else:
                        self.xBE_b2_line_ed_2.setText("0 - Query")

                    # Bit 3...0
                    if msg_byte1_str[0:4] == "0000":
                        self.xBE_b2_line_ed.setText("1200")
                    elif msg_byte1_str[0:4] == "1000":
                        self.xBE_b2_line_ed.setText("2400")
                    elif msg_byte1_str[0:4] == "0100":
                        self.xBE_b2_line_ed.setText("4800")
                    elif msg_byte1_str[0:4] == "1100":
                        self.xBE_b2_line_ed.setText("9600")
                    elif msg_byte1_str[0:4] == "0010":
                        self.xBE_b2_line_ed.setText("19200")

                    # Message byte2 - UART I/O Voltage
                    msg_byte2 = int(msg[6:8], 16)
                    uart_voltage = round(msg_byte2 * 0.025, 3)
                    self.xBE_b4_line_ed.setText(f"{uart_voltage}")
                else:
                    self.disp_info("ERROR", "Invalid CRC")
                    self.clear_xbe()
            else:
                self.disp_info("ERROR", "Invalid answer")
                self.clear_xbe()
        except:
            self.disp_info("ERROR", "No answer from device!")
            self.xBE_resp_line_ed.clear()
            self.clear_xbe()

    def cmd_c0(self):
        cmd = self.commands.get("cmd_c0")

        # Calculate CRC
        crc = self.crc_calc(cmd)
        # print(hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, [cmd], crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {hex(cmd)}, CRC: {hex(crc)}")
            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")

        # Get answer
        try:
            ans = self.com_read()
            # print(ans)

            self.xC0_resp_line_ed.setText(f"{int(ans, 16):#x}")

            if self.com_resp_check(ans, 6):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    self.xC0_resp_b0_lbl.setText("SUCCESSFUL")
                    self.xC0_resp_b0_lbl.setStyleSheet("background-color: #38f587")
                    self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")
                else:
                    self.xC0_resp_b0_lbl.setText("WRONG")
                    self.xC0_resp_b0_lbl.setStyleSheet("background-color: red")
                    self.disp_info("ERROR", "Invalid CRC")
            else:
                self.xC0_resp_b0_lbl.setText("WRONG")
                self.xC0_resp_b0_lbl.setStyleSheet("background-color: red")

                self.disp_info("ERROR", "Wrong answer from device")
        except:
            self.xC0_resp_b0_lbl.setText("WRONG")
            self.xC0_resp_b0_lbl.setStyleSheet("background-color: red")

            self.disp_info("ERROR", "No answer from device!")
            self.xC0_resp_line_ed.clear()

    def cmd_c1(self):
        cmd = self.commands.get("cmd_c1")

        # Calculate CRC
        crc = self.crc_calc(cmd)
        # print(hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, [cmd], crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {hex(cmd)}, CRC: {hex(crc)}")
            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")

        # Get answer
        try:
            ans = self.com_read()
            # print(ans)

            self.xC1_resp_line_ed.setText(f"{int(ans, 16):#x}")

            # Response must have 5 bytes [dev_id, 0x61, data(2 bytes), crc]
            if self.com_resp_check(ans, 10):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)
                # print(msg, hex(dev_crc))

                if self.crc_check(msg, dev_crc):
                    self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    msg_byte0 = int(msg[2:4], 16)
                    msg_byte1 = int(msg[4:6], 16)
                    # print(f"Firmvare version: {msg_byte0}.{msg_byte1}")
                    self.xC1_resp_b0_lbl.setText(f"{msg_byte0}.{msg_byte1}")
                else:
                    self.disp_info("ERROR", "Invalid CRC")
                    self.clear_xc1()
            else:
                self.disp_info("ERROR", "Invalid answer")
                self.clear_xc1()

        except:
            self.disp_info("ERROR", "No answer from device!")
            self.xC1_resp_line_ed.clear()
            self.clear_xc1()

    def cmd_e0(self):
        cmd = self.commands.get("cmd_e0")

        # Calculate CRC
        crc = self.crc_calc(cmd)
        # print(hex(crc))

        # Send command
        try:
            self.com_write(self.dev_addr, [cmd], crc)
            self.disp_info("INFO", f"To device --> address: {hex(self.dev_addr)}, command: {hex(cmd)}, CRC: {hex(crc)}")
            sleep(self.com_delay)
        except:
            self.disp_info("ERROR", "Can't send message")

        # Get answer
        try:
            ans = self.com_read()
            # print(ans)

            self.xE0_resp_line_ed.setText(f"{int(ans, 16):#x}")

            # Response must have 4 bytes [dev_id, 0xE0, data(1 byte), crc]
            if self.com_resp_check(ans, 8):
                msg, dev_crc = ans[2:-2], int(ans[-2:], 16)

                if self.crc_check(msg, dev_crc):
                    self.disp_info("INFO", f"From device <-- address: {hex(self.dev_addr)}, data: 0x{msg}, CRC: {hex(dev_crc)}")

                    msg_byte0 = int(msg[2:4], 16)
                    msg_byte0_str = self.byte_to_string(msg_byte0)
                    # print(msg_byte0_str)

                    # Bit 0
                    if msg_byte0_str[0] == "1":
                        self.xE0_resp_d0_lbl.setText("1 - CRC error")
                        self.xE0_resp_d0_lbl.setStyleSheet("color: red")
                    else:
                        self.xE0_resp_d0_lbl.setText("0 - No CRC errors")
                        self.xE0_resp_d0_lbl.setStyleSheet("color: black")
                    # Bit 1
                    if msg_byte0_str[1] == "1":
                        self.xE0_resp_d1_lbl.setText("1 - Unrecognizable command")
                        self.xE0_resp_d1_lbl.setStyleSheet("color: red")
                    else:
                        self.xE0_resp_d1_lbl.setText("0 - No command errors")
                        self.xE0_resp_d1_lbl.setStyleSheet("color: black")
                    # Bit 2
                    if msg_byte0_str[2] == "1":
                        self.xE0_resp_d2_lbl.setText("1 - Communication error")
                        self.xE0_resp_d2_lbl.setStyleSheet("color: red")
                    else:
                        self.xE0_resp_d2_lbl.setText("0 - No communication errors")
                        self.xE0_resp_d2_lbl.setStyleSheet("color: black")
                else:
                    self.disp_info("ERROR", "Invalid CRC")
                    self.clear_xe0()
            else:
                self.disp_info("ERROR", "Invalid answer")
                self.clear_xe0()
        except:

            self.disp_info("ERROR", "No answer from device!")
            self.xE0_resp_line_ed.clear()
            self.clear_xe0()

    @staticmethod
    def get_com_ports() -> dict:
        # Get all COM-ports
        ports = dict()

        for port, desc, _ in sorted(serial.tools.list_ports.comports()):
            ports[port] = desc

        return ports

    @staticmethod
    def crc_calc(*args) -> int:
        accum = int(f"0x{hex(sum(args))[-2:]}", 16)
        # print(0xFF - accum + 0x01)
        return 0xFF - accum + 0x01

    @staticmethod
    def crc_check(msg: str, crc_in: int) -> bool:
        msg_data = [int(b, 16) for b in re.findall(r'\w\w', msg)]
        my_crc = MainWindow.crc_calc(*msg_data)
        return True if my_crc == crc_in else False

    @staticmethod
    def com_resp_check(response: str, min_len: int):
        return True if len(response) >= min_len else False

    @staticmethod
    def byte_to_string(b: int) -> str:
        s = format(b, "b")
        return (("0" * (8 - len(s))) + s)[::-1]


if __name__ == "__main__":

    main_app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(main_app.exec())
