"""Script to send and receive and display CAN messages from
    the ODDI dual motor test setup.

For more information about the Open Dynamic Robot Initiative:
https://open-dynamic-robot-initiative.github.io/

For more information about the dual motor torque controller:
https://github.com/open-dynamic-robot-initiative/mw_dual_motor_torque_ctrl

Authors:
    David Widjaja
    Dawson Horvath
    Keith Gordon
    Nick Ioannidis

[Insert MIT license here, go do whatever you like lmao]
"""

import struct

import tkinter as tk
import can

def value_to_bytes(val):
    """
    Convert a float value to bytes
    """

    # First, get the float and multiply it by 2**24, then round to integer.
    int_val = int(val * (2.0 ** 24))

    bytes = struct.pack('i', int_val)

    return bytes[::-1]

def bytes_to_value(data):        
    """Convert bytes recevied via CAN to float value."""        
    assert len(data) == 4     
    # convert bytes to signed int (note: bytes in data are in reversed order)        
    qval = struct.unpack("<i", data[::-1])[0]        
    # convert q-value to human-readable float        
    fval = float(qval) / 2.0**24        
    return fval

class VirtualTestBed:

    def __init__(self):

        self.bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=500000)

        self.send_periodic_status(self.bus)
        self.send_periodic_current(self.bus)
        self.send_periodic_encoder(self.bus)
        self.send_periodic_adc(self.bus)
        self.send_periodic_velocity(self.bus)

    def send_periodic_status(self, bus):
        """
        Sends randomized, periodic status messages

        ID: 0x010
        DLC: 1
        Data:
            bit 0: system enable flag
            bit 1: motor 1 enable flag
            bit 2: motor 1 ready flag
            bit 3: motor 2 enable flag
            bit 4: motor 2 ready flag
            bit 5-7: error code
                0: no error
                1: encoder error
                2: can receiver timeout
                3: unused.
                4: position converter error
                5: position rollover error
                6: unused.
                7: other error.
        """

        print("sending periodic status...")

        id = 0x010

        system_enable = 1
        motor_1_enable = 1
        motor_1_ready = 0
        motor_2_enable = 1
        motor_2_ready = 0
        error_code = 4

        data = [(system_enable << 0) 
                | (motor_1_enable << 1)
                | (motor_1_ready << 2)
                | (motor_2_enable << 3)
                | (motor_2_ready << 4) 
                | (error_code << 5) & 0xFF]

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send_periodic(msg, 0.20)

    def send_periodic_current(self, bus):
        """
        Sends randomized, periodic current messages

        ID: 0x020
        DLC: 8
        Data:
            Bytes 0-3: motor 1 current
            Bytes 4-7: motor 2 current
        """

        print("sending periodic current...")

        id = 0x020

        motor_1_curr = 5
        motor_2_curr = -3
        data = list(value_to_bytes(motor_1_curr) + value_to_bytes(motor_2_curr))

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send_periodic(msg, 0.20)

    def send_periodic_encoder(self, bus):
        """
        Sends randomized, periodic encoder messages

        ID: 0x030
        DLC: 8
        Data:
            Bytes 0-3: motor 1 encoder position
            Bytes 4-7: motor 2 encoder position
        """

        print("sending periodic encoder...")

        id = 0x030

        motor_1_enc = 10
        motor_2_enc = -10
        data = list(value_to_bytes(motor_1_enc) + value_to_bytes(motor_2_enc))

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send_periodic(msg, 0.20)

    def send_periodic_velocity(self, bus):
        """
        Sends randomized, periodic velocity messages

        ID: 0x040
        DLC: 8
        Data:
            Bytes 0-3: motor 1 velocity
            Bytes 4-7: motor 2 velocity
        """

        print("sending periodic velocity...")

        id = 0x040

        motor_1_vel = 2
        motor_2_vel = -1
        data = list(value_to_bytes(motor_1_vel) + value_to_bytes(motor_2_vel))

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send_periodic(msg, 0.20)

    def send_periodic_adc(self, bus):
        """
        Sends randomized, periodic adc

        ID: 0x050
        DLC: 8
        Data:
            Bytes 0-3: motor 1 adc
            Bytes 4-7: motor 2 adc
        """

        print("sending periodic adc...")

        id = 0x050

        motor_1_adc = 10
        motor_2_adc = -10
        data = list(value_to_bytes(motor_1_adc) + value_to_bytes(motor_2_adc))

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send_periodic(msg, 0.20)

class CanSender:

    def __init__(self):

        self.bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=500000)

        self.motor_1_curr = 0
        self.motor_2_curr = 0

        pass

    def send_current_command(self, current_mtr1, current_mtr2):

        id = 0x005

        self.motor_1_curr = current_mtr1
        self.motor_2_curr = current_mtr2

        data = list(value_to_bytes(self.motor_1_curr) + value_to_bytes(self.motor_2_curr))

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send(msg)

        print("current command sent")

    def send_single_command(self, command_code, en):
        """
        Sends single command message
        """

        id = 0x000

        data = [command_code, 0, 0, 0, en, 0, 0, 0]

        msg = can.Message(arbitration_id=id, data=data)
        self.bus.send(msg)

        print("command {} sent.".format(command_code))

class StatusBox(tk.LabelFrame):

    def __init__(self, parent, *args, **kwargs):
        tk.LabelFrame.__init__(self, parent, *args, **kwargs)

        self.grid(row=0, column=1, padx=15, pady=15)
        self.grid_rowconfigure([0, 1], weight=1, minsize= 25)
        self.grid_columnconfigure([0, 1], weight=1, minsize=50)

        # initialize status labels
        self.lbl_sys_enable_desc = tk.Label(master=self, text="System Enable:")
        self.lbl_error_code_desc = tk.Label(master=self, text="Error Code:")
        self.lbl_sys_enable = tk.Label(master=self, text="{}".format(0))
        self.lbl_error_code = tk.Label(master=self, text="{}".format(0))

        # grid status labels
        self.lbl_sys_enable_desc.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.lbl_error_code_desc.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.lbl_sys_enable.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.lbl_error_code.grid(row=1, column=1, sticky="w", padx=10, pady=10)

class MTR1Box(tk.LabelFrame):

    def __init__(self, parent, *args, **kwargs):
        tk.LabelFrame.__init__(self, parent, *args, **kwargs)

        self.grid(row=0, column=0, padx=15, pady=15)
        self.grid_rowconfigure([0, 1, 2, 3, 4, 5], weight=1, minsize=25)
        self.grid_columnconfigure([0, 1], weight=1, minsize=50)

        # initialize motor 1 labels
        self.lbl_mtr1_en_desc = tk.Label(master=self, text="MTR1 Enable:")
        self.lbl_mtr1_rdy_desc = tk.Label(master=self, text="MTR1 Ready:")
        self.lbl_mtr1_curr_desc = tk.Label(master=self, text="MTR1 Current(A):")
        self.lbl_mtr1_enc_desc = tk.Label(master=self, text="MTR1 Encoder Position(revs):")
        self.lbl_mtr1_vel_desc = tk.Label(master=self, text="MTR1 Velocity(rpm):")
        self.lbl_mtr1_adc_desc = tk.Label(master=self, text="MTR1 ADC:")
        self.lbl_mtr1_en = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr1_rdy = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr1_curr = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr1_enc = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr1_vel = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr1_adc = tk.Label(master=self, text="{}".format(0))

        # grid motor 1 labels
        self.lbl_mtr1_en_desc.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_rdy_desc.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_curr_desc.grid(row=2, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_enc_desc.grid(row=3, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_vel_desc.grid(row=4, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_adc_desc.grid(row=5, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_en.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_rdy.grid(row=1, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_curr.grid(row=2, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_enc.grid(row=3, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_vel.grid(row=4, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr1_adc.grid(row=5, column=1, sticky="w", padx=10, pady=10)

class MTR2Box(tk.LabelFrame):

    def __init__(self, parent, *args, **kwargs):
        tk.LabelFrame.__init__(self, parent, *args, **kwargs)

        self.grid(row=0, column=2, padx=15, pady=15)
        self.grid_rowconfigure([0, 1, 2, 3, 4, 5], weight=1, minsize=25)
        self.grid_columnconfigure([0, 1], weight=1, minsize=50)

        # initialize motor 2 labels
        self.lbl_mtr2_en_desc = tk.Label(master=self, text="MTR2 Enable:")
        self.lbl_mtr2_rdy_desc = tk.Label(master=self, text="MTR1 Ready:")
        self.lbl_mtr2_curr_desc = tk.Label(master=self, text="MTR2 Current(A):")
        self.lbl_mtr2_enc_desc = tk.Label(master=self, text="MTR2 Encoder Position(revs):")
        self.lbl_mtr2_vel_desc = tk.Label(master=self, text="MTR2 Velocity(rpm):")
        self.lbl_mtr2_adc_desc = tk.Label(master=self, text="MTR2 ADC:")
        self.lbl_mtr2_en = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr2_rdy = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr2_curr = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr2_enc = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr2_vel = tk.Label(master=self, text="{}".format(0))
        self.lbl_mtr2_adc = tk.Label(master=self, text="{}".format(0))

        # grid motor 2 labels
        self.lbl_mtr2_en_desc.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_rdy_desc.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_curr_desc.grid(row=2, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_enc_desc.grid(row=3, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_vel_desc.grid(row=4, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_adc_desc.grid(row=5, column=0, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_en.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_rdy.grid(row=1, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_curr.grid(row=2, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_enc.grid(row=3, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_vel.grid(row=4, column=1, sticky="w", padx=10, pady=10)
        self.lbl_mtr2_adc.grid(row=5, column=1, sticky="w", padx=10, pady=10)

class CmdInputBox(tk.LabelFrame):

    def __init__(self, parent, *args, **kwargs):
        tk.LabelFrame.__init__(self, parent, *args, **kwargs)

        self.can_sender = CanSender()

        self.grid(row=1, column=0, columnspan=2, padx=15, pady=15)
        self.grid_rowconfigure([0, 1, 2, 3, 4, 5, 6, 7, 8], weight=1, minsize=25)
        self.grid_columnconfigure([0, 1, 2], weight=1, minsize=50)

        # command code labels
        self.lbl_cmd_sys_en_desc = tk.Label(master=self, text="System Enable:")
        self.lbl_cmd_mtr1_en_desc = tk.Label(master=self, text="Motor 1 Enable:")
        self.lbl_cmd_mtr1_vspring_desc = tk.Label(master=self, text="Motor 1 Enable Virtual Spring Mode:")
        self.lbl_cmd_mtr2_en_desc = tk.Label(master=self, text="Motor 2 Enable:")
        self.lbl_cmd_mtr2_vspring_desc = tk.Label(master=self, text="Motor 2 Enable Virtual Spring Mode:")
        self.lbl_cmd_send_curr_desc = tk.Label(master=self, text="Send Current CAN Msg:")
        self.lbl_cmd_send_pos_desc = tk.Label(master=self, text="Send Encoder Position CAN Msg:")
        self.lbl_cmd_send_vel_desc = tk.Label(master=self, text="Send Velocity CAN Msg:")
        self.lbl_cmd_send_adc_desc = tk.Label(master=self, text="Send ADC CAN Msg:")
        self.lbl_cmd_sys_en = tk.Entry(master=self)
        self.lbl_cmd_mtr1_en = tk.Entry(master=self)
        self.lbl_cmd_mtr1_vspring = tk.Entry(master=self)
        self.lbl_cmd_mtr2_en = tk.Entry(master=self)
        self.lbl_cmd_mtr2_vspring = tk.Entry(master=self)
        self.lbl_cmd_send_curr = tk.Entry(master=self)
        self.lbl_cmd_send_pos = tk.Entry(master=self)
        self.lbl_cmd_send_vel = tk.Entry(master=self)
        self.lbl_cmd_send_adc = tk.Entry(master=self)
        self.lbl_cmd_sys_en_button = tk.Button(master=self, text="Send", command=self.send_sys_en)
        self.lbl_cmd_mtr1_en_button = tk.Button(master=self, text="Send", command = self.send_mtr1_en)
        self.lbl_cmd_mtr1_vspring_button = tk.Button(master=self, text="Send", command=self.send_mtr1_vspring_en)
        self.lbl_cmd_mtr2_en_button = tk.Button(master=self, text="Send", command=self.send_mtr2_en)
        self.lbl_cmd_mtr2_vspring_button = tk.Button(master=self, text="Send", command=self.send_mtr2_vspring_en)
        self.lbl_cmd_send_curr_button = tk.Button(master=self, text="Send", command=self.send_curr)
        self.lbl_cmd_send_pos_button = tk.Button(master=self, text="Send", command=self.send_pos)
        self.lbl_cmd_send_vel_button = tk.Button(master=self, text="Send", command=self.send_vel)
        self.lbl_cmd_send_adc_button = tk.Button(master=self, text="Send", command=self.send_adc)

        # set initial values
        self.lbl_cmd_sys_en.insert(0, "0")
        self.lbl_cmd_mtr1_en.insert(0, "0")
        self.lbl_cmd_mtr1_vspring.insert(0, "0")
        self.lbl_cmd_mtr2_en.insert(0, "0")
        self.lbl_cmd_mtr2_vspring.insert(0, "0")
        self.lbl_cmd_send_curr.insert(0, "0")
        self.lbl_cmd_send_pos.insert(0, "0")
        self.lbl_cmd_send_vel.insert(0, "0")
        self.lbl_cmd_send_adc.insert(0, "0")

        # command code grid
        self.lbl_cmd_sys_en_desc.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr1_en_desc.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr1_vspring_desc.grid(row=2, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr2_en_desc.grid(row=3, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr2_vspring_desc.grid(row=4, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_curr_desc.grid(row=5, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_pos_desc.grid(row=6, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_vel_desc.grid(row=7, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_adc_desc.grid(row=8, column=0, sticky="w", padx=10, pady=10)
        self.lbl_cmd_sys_en.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr1_en.grid(row=1, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr1_vspring.grid(row=2, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr2_en.grid(row=3, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr2_vspring.grid(row=4, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_curr.grid(row=5, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_pos.grid(row=6, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_vel.grid(row=7, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_adc.grid(row=8, column=1, sticky="w", padx=10, pady=10)
        self.lbl_cmd_sys_en_button.grid(row=0, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr1_en_button.grid(row=1, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr1_vspring_button.grid(row=2, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr2_en_button.grid(row=3, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_mtr2_vspring_button.grid(row=4, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_curr_button.grid(row=5, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_pos_button.grid(row=6, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_vel_button.grid(row=7, column=2, sticky="w", padx=10, pady=10)
        self.lbl_cmd_send_adc_button.grid(row=8, column=2, sticky="w", padx=10, pady=10)

    def send_sys_en(self):
        if int(self.lbl_cmd_sys_en.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x01, en)

    def send_mtr1_en(self):
        if int(self.lbl_cmd_mtr1_en.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x02, en)

    def send_mtr1_vspring_en(self):
        if int(self.lbl_cmd_mtr1_vspring.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x03, en)

    def send_mtr2_en(self):
        if int(self.lbl_cmd_mtr2_en.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x05, en)

    def send_mtr2_vspring_en(self):
        if int(self.lbl_cmd_mtr2_vspring.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x06, en)

    def send_curr(self):
        if int(self.lbl_cmd_send_curr.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x0C, en)

    def send_pos(self):
        if int(self.lbl_cmd_send_pos.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x0D, en)

    def send_vel(self):
        if int(self.lbl_cmd_send_vel.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x0E, en)

    def send_adc(self):
        if int(self.lbl_cmd_send_adc.get()) == 1:
            en = True
        else:
            en = False
        self.can_sender.send_single_command(0x0F, en)

class CurrInputBox(tk.LabelFrame):

    def __init__(self, parent, *args, **kwargs):
        tk.LabelFrame.__init__(self, parent, *args, **kwargs)

        self.can_sender = CanSender()

        self.grid(row=1, column=2, padx=15, pady=15)
        self.grid_rowconfigure([0, 1, 2], weight=1, minsize=25)
        self.grid_columnconfigure([0, 1], weight=1, minsize=50)

        self.lbl_send_curr_mtr1_desc = tk.Label(master=self, text="Update Motor 1 Current:")
        self.lbl_send_curr_mtr2_desc = tk.Label(master=self, text="Update Motor 2 Current: ")
        self.lbl_send_curr_mtr1 = tk.Entry(master=self)
        self.lbl_send_curr_mtr2 = tk.Entry(master=self)
        self.lbl_update_curr_button = tk.Button(master=self, text="Update CAN Msg", command = self.send_current_command)

        self.lbl_send_curr_mtr1.insert(0, "0")
        self.lbl_send_curr_mtr2.insert(0, "0")

        self.lbl_send_curr_mtr1_desc.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.lbl_send_curr_mtr2_desc.grid(row=1, column=0, sticky="w", padx=10, pady=10)
        self.lbl_send_curr_mtr1.grid(row=0, column=1, sticky="w", padx=10, pady=10)
        self.lbl_send_curr_mtr2.grid(row=1, column=1, sticky="w", padx=10, pady=10)
        self.lbl_update_curr_button.grid(row=2, column=0, columnspan=2, sticky="w", padx=10, pady=10)
    
    def send_current_command(self):
        self.can_sender.send_current_command(float(self.lbl_send_curr_mtr1.get()), float(self.lbl_send_curr_mtr2.get()))

class MainApp(tk.Frame):

    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.parent = parent

        self.rowconfigure([0, 1], weight=1, minsize=100)
        self.columnconfigure([0, 1, 2], weight=1, minsize=100)

        # create frames for different definitions
        self.status = StatusBox(self, relief=tk.RAISED, borderwidth=2, text="STATUS")
        self.mtr1 = MTR1Box(self, relief=tk.RAISED, borderwidth=2, text="MOTOR 1")
        self.mtr2 = MTR2Box(self, relief=tk.RAISED, borderwidth=2, text="MOTOR 2")
        self.cmd_input = CmdInputBox(self, relief=tk.RAISED, borderwidth=2, text="SEND CMD CAN MSG")
        self.curr_input = CurrInputBox(self, relief=tk.RAISED, borderwidth=2, text="CHANGE CURRENT CAN MSG")

    def check_msg(self, bus):

        msg = bus.recv(timeout=None)
        if msg is not None:

            id = msg.arbitration_id

            # status
            if id == 0x010:
                
                data = int(msg.data[0])

                system_enable = data & 1
                motor_1_enable = (data >> 1) & 1
                motor_1_ready = (data >> 2) & 1
                motor_2_enable = (data >> 3) & 1
                motor_2_ready = (data >> 4) & 1
                error_code = (data >> 5) & 0x7

                self.status.lbl_sys_enable["text"] = str(system_enable)
                self.mtr1.lbl_mtr1_en["text"] = str(motor_1_enable)
                self.mtr1.lbl_mtr1_rdy["text"] = str(motor_1_ready)
                self.mtr2.lbl_mtr2_en["text"] = str(motor_2_enable)
                self.mtr2.lbl_mtr2_rdy["text"] = str(motor_2_ready)
                self.status.lbl_error_code["text"] = str(error_code)

            # current
            elif id == 0x020:
                current_1 = bytes_to_value(msg.data[0:4])
                current_2 = bytes_to_value(msg.data[4:8])

                self.mtr1.lbl_mtr1_curr["text"] = "{}".format(current_1)
                self.mtr2.lbl_mtr2_curr["text"] = "{}".format(current_2)

            # encoder pos
            elif id == 0x030:
                enc_1 = bytes_to_value(msg.data[0:4])
                enc_2 = bytes_to_value(msg.data[4:8])

                self.mtr1.lbl_mtr1_enc["text"] = str(enc_1)
                self.mtr2.lbl_mtr2_enc["text"] = str(enc_2)

            # velocity
            elif id == 0x040:
                vel_1 = bytes_to_value(msg.data[0:4])
                vel_2 = bytes_to_value(msg.data[4:8])

                self.mtr1.lbl_mtr1_vel["text"] = str(vel_1)
                self.mtr2.lbl_mtr2_vel["text"] = str(vel_2)

            # adc
            elif id == 0x050:
                adc_1 = bytes_to_value(msg.data[0:4])
                adc_2 = bytes_to_value(msg.data[4:8])

                self.mtr1.lbl_mtr1_adc["text"] = str(adc_1)
                self.mtr2.lbl_mtr2_adc["text"] = str(adc_2)

        self.parent.after(10, self.check_msg, bus)

# create the virtual test bed
virtual_test_bed = VirtualTestBed()

root = tk.Tk()
root.title("Dual Motor Test Bed CAN Input/Output")
root.resizable(width=False, height=False)
app = MainApp(root)
app.pack(side="top", fill="both", expand=True)

receiving_bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=500000)

root.after(10, app.check_msg, receiving_bus)
root.mainloop()
