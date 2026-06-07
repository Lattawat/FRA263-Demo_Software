import platform
import time as pytime
from pymodbus.client import ModbusSerialClient as ModbusClient


# Last holding register in the READ map (0x26–0x31 status); routine reads 0x00 through 0x31.
MAX_ADDRESS = 0x31

HB_HI = 18537       # HI 
HB_YA = 22881       # YA

class Binary():
    """
    Binary Class
    """
    def decimal_to_binary(self, decimal_num):
        """
        This function converts base 10 to base 2
        """
        binary_num = ""
        while decimal_num > 0:
            binary_num = str(decimal_num % 2) + binary_num
            decimal_num = decimal_num // 2
        # Fill to 16 digits with 0
        if len(binary_num) < 16:
            binary_num = "0"*(16-len(binary_num)) + binary_num
        return binary_num
        
    def binary_to_decimal(self, binary_num):
        """
        This function converts base 2 to base 10
        """
        decimal_num = 0
        for i in range(len(binary_num)):
            decimal_num += int(binary_num[i]) * (2 ** (len(binary_num)-i-1))
        return decimal_num
    
    def binary_crop(self, digit, binary_num):
        """
        This function crops the last n digits of the binary number
        """
        return binary_num[len(binary_num)-digit:]

    def binary_twos_complement(self, number):
        """
        This functions converts the (negative) number to its 16-bit two's complement representation
        """
        if number < 0:
            number = (1 << 16) + number  # Adding 2^16 to the negative number
        return number
    
    def binary_reverse_twos_complement(self, number):
        """
        This functions converts the 16-bit two's complement number back to its original signed representation 
        """
        if number & (1 << 15):  # Check if the most significant bit is 1
            number = number - (1 << 16)  # Subtract 2^16 from the number
        return number


class Protocol(Binary):
    def __init__(self):
        self.port = None
        self.client = None

        # Modbus Client
        self.usb_connect = False
        self.slave_address = 21  # Modbus slave address
        self.register = None

        # Routine
        self.routine_normal = True

        # Heartbeat — READ 0x00 (reply HI via WRITE 0x00)
        self.hb_val = None

        # Set operating mode — WRITE 0x01
        self.base_system_status_register = 0b0000

        # Local/UI gripper state 
        self.gripper_status = "0"
        self.gripper_moving_status = "0"

        # Lead/reed sensors — READ 0x26
        self.gripper_actual_reed1 = "0"
        self.gripper_actual_reed2 = "0"
        self.gripper_actual_reed3 = "0"

        # Gripper enable checkbox (AUTO) — WRITE 0x04
        self.gripper_checkbox = "0"

        # Current robot task — READ 0x27
        self.moving_status = "Idle"
        self.moving_status_previous = "Idle"

        # Position, velocity, acceleration — READ 0x28, 0x29, 0x30
        self.theta_actual_pos = 0.0
        self.theta_actual_speed = 0.0
        self.theta_actual_accel = 0.0

        # Robot emergency state — READ 0x31
        self.emergency_stop_status = "0"

        # Soft stop — WRITE 0x25
        self.stop_process = "0"

    def _write_register_debug(self, address: int, value: int, label: str = "") -> bool:
        if not self.client:
            print(f"[ERROR] No Modbus client. Cannot write {label}")
            return False

        wr = self.client.write_register(
            address=address,
            value=value,
            slave=self.slave_address
        )

        ok = not (wr is None or (hasattr(wr, "isError") and wr.isError()))

        # Reverse two's complement for readable signed value
        signed_val = self.binary_reverse_twos_complement(value)

        print(
            f"[WRITE] {label:<20} | "
            f"Slave:{self.slave_address} | "
            f"Addr:{address} (0x{address:02X}) | "
            f"Raw:{value} | "
            f"Signed:{signed_val} | "
            f"Hex:0x{value:04X} | "
            f"Status:{'OK' if ok else 'ERROR'}"
        )

        return ok
    
    # ===== Connection Function ====== 
    def connect_rtu(self, com_port: str, slave: int = 21) -> bool:
            """Create + connect Modbus RTU client."""
            self.slave_address = slave
            self.port = com_port
            # close existing client if any
            self.disconnect()
            # print(com_port, slave)

            self.client = ModbusClient(
                port=com_port,
                baudrate=230400,
                parity="E",
                stopbits=1,
                bytesize=8,
                timeout=0.01,
                retries=0, 
            )
            ok = self.client.connect()
            self.usb_connect = bool(ok)
            return bool(ok)

    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.usb_connect = False

    def is_connected(self) -> bool:
        return bool(self.client) and bool(getattr(self.client, "connected", False))
    # ================================

    # === Routine Function ===
    def routine(self):
        if not self.client:
            self.routine_normal = False
            return False
        
        t0 = pytime.perf_counter()
        # Read Heartbeat
        rr = self.client.read_holding_registers(address=0x00, count=MAX_ADDRESS+1, slave=self.slave_address)
        if rr is None or rr.isError() or not hasattr(rr, "registers"):
            self.routine_normal = False
            return False 
        t1 = pytime.perf_counter()

        # print(f"Heartbeat read: {(t1-t0)*1000:.1f} ms")

        self.register = rr

        # Heartbeat        
        self.hb_val = rr.registers[0]
        
        # READ: 0x26 reeds, 0x27 task, 0x28–0x30 motion, 0x31 emergency
        self.read_gripper_actual_status()   # 0x26 lead/reed sensors
        self.read_theta_moving_status()     # 0x27 current robot task
        self.read_theta_actual_status()     # 0x28–0x30 position, velocity, acceleration
        self.read_emergency_stop_status()   # 0x31 robot emergency state
        self.routine_normal = True
        return True

    # def routine(self):
    #     if not self.client:
    #         self.routine_normal = False
    #         return False
        
    #     t0 = pytime.perf_counter()
    #     # Read heartbeat only
    #     rr_hb = self.client.read_holding_registers(
    #         address=0x00,
    #         count=1,
    #         slave=self.slave_address
    #     )
    #     t1 = pytime.perf_counter()



    #     # Read status block 0x26–0x31
    #     rr_status = self.client.read_holding_registers(
    #         address=0x00,
    #         # count=0x31 - 0x26 + 1,
    #         count=MAX_ADDRESS + 1,
    #         slave=self.slave_address
    #     )
    #     t2 = pytime.perf_counter()

    #     print(f"hb read: {(t1-t0)*1000:.1f} ms, status read: {(t2-t1)*1000:.1f} ms")
    #     # print(status[0], status[1], status[2], status[3], status[10], status[11])

    #     # Status index
    #     # 0x26 : [0]
    #     # 0x27 : [1]
    #     # 0x28 : [2]
    #     # 0x29 : [3]
    #     # 0x30 : [10]
    #     # 0x31 : [11]

    #     if (
    #         rr_hb is None or rr_hb.isError() or not hasattr(rr_hb, "registers") or
    #         rr_status is None or rr_status.isError() or not hasattr(rr_status, "registers")
    #     ):
    #         self.routine_normal = False
    #         return False

    #     self.hb_val = rr_hb.registers[0]

    #     status = rr_status.registers

    #     self.gripper_actual_reed1 = bool(status[0] & 0b0001)  # 0x26
    #     self.gripper_actual_reed2 = bool(status[0] & 0b0010)  # 0x26
    #     self.gripper_actual_reed3 = bool(status[0] & 0b0100)  # 0x26

    #     moving = status[1]  # 0x27

    #     # print(f"Moving status raw: {moving:016b}")
    #     self.moving_status_previous = self.moving_status

    #     if moving & 0b000001:
    #         self.moving_status = "Homing"
    #     elif moving & 0b000010:
    #         self.moving_status = "Go Pick"
    #     elif moving & 0b000100:
    #         self.moving_status = "Go Place"
    #     elif moving & 0b001000:
    #         self.moving_status = "Go Point"
    #     else:
    #         self.moving_status = "Idle"

    #     self.theta_actual_pos = self.binary_reverse_twos_complement(status[2]) / 10.0  # 0x28
    #     self.theta_actual_speed = self.binary_reverse_twos_complement(status[3]) / 10.0  # 0x29
    #     self.theta_actual_accel = self.binary_reverse_twos_complement(status[10]) / 10.0  # 0x30

    #     # print(f"Position: {self.theta_actual_pos} deg, Speed: {self.theta_actual_speed} deg/s, Accel: {self.theta_actual_accel} deg/s²")
    #     # print(f"Emergency stop raw: {status[11]:016b}")
    #     self.emergency_stop_status = bool(status[11] & 0b0001)  # 0x31

    #     self.routine_normal = True
    #     return True
    
#### STATUS + Connection
    # === Heartbeat Functions (0x00) ===
    def write_heartbeat_hi(self) -> bool:
        wr = self.client.write_register(address=0x00, value=HB_HI, slave=self.slave_address)
        if wr is None or (hasattr(wr, "isError") and wr.isError()):
            return False
        return True
    def read_heartbeat_register(self) -> int | None:
        """Read holding register 0x00 only (lightweight, for 5 Hz heartbeat loop)."""
        if not self.client:
            return None
        rr = self.client.read_holding_registers(
            address=0x00, count=1, slave=self.slave_address
        )
        if rr is None or rr.isError() or not hasattr(rr, "registers"):
            return None
        self.hb_val = rr.registers[0]
        return self.hb_val

    def heartbeat_tick(self) -> tuple[bool, bool]:
        """
        One heartbeat cycle at HB rate (e.g. 5 Hz):
          1. Read reg 0x00
          2. If value is YA (22881), write HI (18537) back

        Returns (saw_ya, wrote_hi).
        """
        hb = self.read_heartbeat_register()
        if hb is None:
            return False, False
        if hb == HB_YA:
            return True, self.write_heartbeat_hi()
        return False, False

    def heartbeat_from_routine(self):
        """Legacy helper when hb_val was already read by routine()."""
        hb = getattr(self, "hb_val", None)
        if hb is None:
            return False, None

        if hb == HB_YA:
            ok = self.write_heartbeat_hi()   # FC06 only
            return bool(ok), hb

        return False, hb
    
    # === Set operating mode (0x01) ===
    def write_base_system_status(self, command):
        if command == 'go_home':
            self.base_system_status_register = 0b0001   
        elif command == 'Jog':
            self.base_system_status_register = 0b0010
        elif command == 'Auto':
            self.base_system_status_register = 0b0100
        elif command == 'set_home':
            self.base_system_status_register = 0b1000
        elif command == 'Test':
            self.base_system_status_register = 0b10000
        self._write_register_debug(0x01, self.base_system_status_register, f"BaseSystem {command}")


#### [Manual] Gripper + Command
    # === Gripper open | close | up | down command (0x02) ===
    def write_gripper_command(self, command):
        if command == 'Up':
            self.gripper_command_register = 0b0000   
        elif command == 'Down':
            self.gripper_command_register = 0b0001
        elif command == 'Open':
            self.gripper_command_register = 0b0010
        elif command == 'Close':
            self.gripper_command_register = 0b0100
        self._write_register_debug(0x02, self.gripper_command_register, f"Gripper {command}")

    # === Gripper pick | place command (0x03) ===
    def write_gripper_movement(self, command):
        if command == 'Pick':
            self.gripper_movement_register = 0b0001   
        elif command == 'Place':
            self.gripper_movement_register = 0b0010
        self._write_register_debug(0x03, self.gripper_movement_register, f"GripperMove {command}")
    
    # === Gripper Enable Checkbox (AUTO mode) (0x04) ===
    def write_gripper_checkbox(self, command):
        if command == 'Disable':
            self.gripper_checkbox_register = 0b0000
        elif command == 'Enable':
            self.gripper_checkbox_register = 0b0001
        self._write_register_debug(0x04, self.gripper_checkbox_register, f"GripperCheckbox {command}")
    
    # === Position incrementing command (Jog: degree) (0x05) ===
    def write_jog(self, value=None):
        self.jog_degree = self.binary_twos_complement(value)
        self._write_register_debug(0x05, self.jog_degree, "JOG")


#### [Test] Performance + Precision
    # === Set test mode (Performance/Precision) (0x06) ===
    def write_test_mode(self, mode=None):
        if mode == "Performance":
            self.test_mode = 1
        elif mode == "Precision" :
            self.test_mode = 0 
        self._write_register_debug(0x06, self.test_mode, f"TestMode {mode}")

    # === Set desire velocity for performance test (0x07) ===
    def write_test_speed(self, value=None):
        self.test_speed = self.binary_twos_complement(value)
        self._write_register_debug(0x07, self.test_speed, "TestSpeed")

    # === Set desire acceleration for performance test (0x08) ===
    def write_test_accel(self, value=None):
        self.test_accel = self.binary_twos_complement(value)
        self._write_register_debug(0x08, self.test_accel, "TestAccel")

    # === Set initial position for precision test (0x09) ===
    def write_test_init_pos(self, init_pos=None):
        self.test_init_pos = self.binary_twos_complement(init_pos)
        self._write_register_debug(0x09, self.test_init_pos, "TestInitPos")

    # === Set final position for precision test (0x10) ===
    def write_test_target_pos(self, target_pos=None):
        self.test_target_pos = self.binary_twos_complement(target_pos)
        self._write_register_debug(0x10, self.test_target_pos, "TestTargetPos")

    # === Set the number of repetition in precision test (sign = unit) (0x11) ===
    def write_test_repeat(self, repeat=None):
        self.test_repeat_w_unit = self.binary_twos_complement(repeat)
        self._write_register_debug(0x11, self.test_repeat_w_unit, "TestRepeat")


#### [Auto] Pick-Place + P2P
    # Desire pick and place for hole 1-5 with direction 
    # (+1 => go to index 1 in counter 
    # clockwise direction, -5 => go to index 5 in clockwise direction) 
    # (0x12-0x21)
    def write_pick_place_hole(self, address, value):
        signed_value = self.binary_twos_complement(value)
        self._write_register_debug(address, signed_value, f"Pick Place")
           
    def write_n_pair(self, value):
        self._write_register_debug(0x22, value, f"N_pare {value}")

    # === Choose unit for point to point action (0x23) ===
    def write_p2p_unit(self, unit=None):
        if unit == 'degree':
            self.p2p_unit = 0b0000 
        elif unit == 'index':
            self.p2p_unit = 0b0001
        self._write_register_debug(0x23, self.p2p_unit, f"P2P Unit {unit}")

    # === Set the desire position based on the unit with direction (0x24) ===
    def write_p2p_value(self, value=None):
        self.p2p_value = self.binary_twos_complement(value)
        self._write_register_debug(0x24, self.p2p_value, "P2P Value")


    # === Soft stop command (0x25) ===
    def write_stop_process(self, command):
        if command == 'Normal':
            self.stop_process_register = 0b0000   
        elif command == 'Stop':
            self.stop_process_register = 0b0001
        self._write_register_debug(0x25, self.stop_process_register, f"StopProcess {command}")



#### READ STATUS
    # === Leed sensors status (0x26) ===
    def read_gripper_actual_status(self):
        # Reed switch status
        gripper_actual_status_binary = self.binary_crop(4, self.decimal_to_binary(self.register.registers[0x26]))[::-1]
        self.gripper_actual_reed1 = (gripper_actual_status_binary[0] == '1')  # READ 0x26
        self.gripper_actual_reed2 = (gripper_actual_status_binary[1] == '1')  # READ 0x26
        self.gripper_actual_reed3 = (gripper_actual_status_binary[2] == '1')  # READ 0x26


    # === Current robot task  (0x27) ===
    def read_theta_moving_status(self):
        self.moving_status_previous = self.moving_status
        moving_status_binary = self.binary_crop(6, self.decimal_to_binary(self.register.registers[0x27]))[::-1]
        
        if moving_status_binary[0] == '1':
            self.moving_status = "Homing"
        elif moving_status_binary[1] == '1':
            self.moving_status = "Go Pick"
        elif moving_status_binary[2] == '1':
            self.moving_status = "Go Place"
        elif moving_status_binary[3] == '1':
            self.moving_status = "Go Point"
        else:
            self.moving_status = "Idle"

    # === Current robot position w.r.t current home pos,velo,accel (0x28 - 0x30) ===
    def read_theta_actual_status(self):
        self.theta_actual_pos = self.binary_reverse_twos_complement(self.register.registers[0x28]) / 10.0
        self.theta_actual_speed = self.binary_reverse_twos_complement(self.register.registers[0x29]) / 10.0
        self.theta_actual_accel = self.binary_reverse_twos_complement(self.register.registers[0x30]) / 10.0


    # === Robot emergency state (0x31) ===
    def read_emergency_stop_status(self):
        emergency_stop_binary = self.binary_crop(4, self.decimal_to_binary(self.register.registers[0x31]))[::-1]
        self.emergency_stop_status = (emergency_stop_binary[0] == '1')  # READ 0x31

