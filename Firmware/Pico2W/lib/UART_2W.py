
from machine import Pin as _Pin
from machine import UART as _UART
import struct as _struct
import micropython as _micropython
from time import ticks_ms as _ticks_ms, ticks_diff as _ticks_diff


# ============================================================
# MicroPython optimization
# ============================================================

_native = _micropython.native


# ============================================================
# Frame types
# ============================================================

TYPE_COMMAND = 0x10
TYPE_TELEMETRY = 0x20
TYPE_KICK_ACK = 0x21
TYPE_KICK_RESULT = 0x22


# KICK command
KICK_NONE = 0
KICK_FIRE = 1
KICK_CANCEL = 2


# KICK ACK status
ACK_ACCEPTED = 0
ACK_DUPLICATE = 1
ACK_REJECT_BUSY = 2
ACK_REJECT_FAULT = 3
ACK_REJECT_MODE = 4
ACK_REJECT_INVALID = 5


# KICK result status
RESULT_SUCCESS = 0
RESULT_CANCELLED = 1
RESULT_FAILED = 2
RESULT_TIMEOUT = 3


# KICK ID reserved value
KICK_ID_NONE = 0xFFFF


# ============================================================
# Constants
# ============================================================

_SYNC = b"\xA5\x5A"

# _FRAME_LENGTHS = (
#     0, 0, 0, 0, 0, 0, 0, 0,
#     0, 0, 0, 0, 0, 0, 0, 0,
#     23, 0, 0, 0, 0, 0, 0, 0,
#     0, 0, 0, 0, 0, 0, 0, 0,
#     21, 9, 9
# )

_FRAME_LENGTHS = (
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    25, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    23, 9, 9
)

_COMMAND_STRUCT = "<hhhHBHHBHB"
_TELEMETRY_STRUCT = "<BBHHHHBHH"
_KICK_STRUCT = "<HB"

_COMMAND_SIZE = _struct.calcsize(_COMMAND_STRUCT)
_TELEMETRY_SIZE = _struct.calcsize(_TELEMETRY_STRUCT)
_KICK_SIZE = _struct.calcsize(_KICK_STRUCT)


# ============================================================
# CRC16-CCITT LUT
# ============================================================

def _make_crc_table():
    table = [0] * 256

    for i in range(256):
        crc = i << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

        table[i] = crc

    return tuple(table)


_CRC16_TABLE = _make_crc_table()


@_native
def _crc16(data, start, end):
    crc = 0xFFFF
    table = _CRC16_TABLE

    for i in range(start, end):
        crc = (
            ((crc << 8) & 0xFFFF)
            ^ table[((crc >> 8) ^ data[i]) & 0xFF]
        )

    return crc


# ============================================================
# UART Base
# ============================================================

class _UARTBase:

    def __init__(self):
        self.uart = _UART(
            1,
            baudrate=1_000_000,
            bits=8,
            parity=None,
            stop=1,
            tx=_Pin(8),
            rx=_Pin(9),
            rxbuf=128,
            txbuf=128,
            timeout=0,
            timeout_char=0
        )

        self._rx = bytearray()
        self._tx_seq = 0

        self._last_seq = None

        self.seq_lost = 0
        self.seq_duplicate = 0
        self.seq_out_of_order = 0

        self._last_rx = 0
        self._has_rx = False

    def _send_frame(self, frame_type, payload):
        seq = self._tx_seq

        body = bytes((frame_type, seq)) + payload
        crc = _crc16(body, 0, len(body))

        frame = _SYNC + body + _struct.pack("<H", crc)

        result = self.uart.write(frame)

        self._tx_seq = (seq + 1) & 0xFF

        return result

    def _read_packets(self):
        uart_read = self.uart.read
        rx = self._rx

        data = uart_read()

        if data:
            rx.extend(data)

        packets = []

        find = rx.find
        ticks = _ticks_ms
        diff = _ticks_diff

        while True:

            pos = find(_SYNC)

            if pos < 0:
                if rx and rx[-1] == 0xA5:
                    rx[:] = b"\xA5"
                else:
                    rx.clear()

                break

            if pos:
                del rx[:pos]

            if len(rx) < 4:
                break

            frame_type = rx[2]

            if frame_type >= len(_FRAME_LENGTHS):
                del rx[0]
                continue

            frame_length = _FRAME_LENGTHS[frame_type]

            if not frame_length:
                del rx[0]
                continue

            if len(rx) < frame_length:
                break

            received_crc = (
                rx[frame_length - 2]
                | (rx[frame_length - 1] << 8)
            )

            calculated_crc = _crc16(
                rx,
                2,
                frame_length - 2
            )

            if received_crc != calculated_crc:
                del rx[0]
                continue

            seq = rx[3]

            payload = bytes(rx[4:frame_length - 2])

            now = ticks()

            if self._has_rx and diff(now, self._last_rx) > 100:
                self._last_seq = None

            last_seq = self._last_seq

            if last_seq is not None:

                delta = (seq - last_seq) & 0xFF

                if delta == 0:
                    self.seq_duplicate += 1

                elif delta < 128:
                    self.seq_lost += delta - 1
                    self._last_seq = seq

                else:
                    self.seq_out_of_order += 1

            else:
                self._last_seq = seq

            self._last_rx = now
            self._has_rx = True

            packets.append((frame_type, seq, payload))

            del rx[:frame_length]

        return packets

    def link_alive(self, timeout_ms=50):
        if not self._has_rx:
            return False

        return _ticks_diff(
            _ticks_ms(),
            self._last_rx
        ) <= timeout_ms


# ============================================================
# Pico 2 W UART
# ============================================================

class Pico2WUART(_UARTBase):

    def __init__(self):
        super().__init__()

        self.kick_result_ack_id = KICK_ID_NONE

    def send_command(
        self,
        vx=0,
        vy=0,
        omega=0,
        dribbler=0,
        kick_command=KICK_NONE,
        kick_id=0,
        control_mode=1,
        host_error_status=0,
        command_timeout_ms=30
    ):

        payload = _struct.pack(
            _COMMAND_STRUCT,
            vx,
            vy,
            omega,
            dribbler,
            kick_command,
            kick_id,
            self.kick_result_ack_id,
            control_mode,
            host_error_status,
            command_timeout_ms
        )

        return self._send_frame(TYPE_COMMAND, payload)

    def read_messages(self):
        messages = []

        for frame_type, seq, payload in self._read_packets():

            if frame_type == TYPE_TELEMETRY:

                (
                    kick_status,
                    motor_fault,
                    ultrasonic_0,
                    ultrasonic_1,
                    ultrasonic_2,
                    ultrasonic_3,
                    ultrasonic_status,
                    error_status,
                    system_status
                ) = _struct.unpack(
                    _TELEMETRY_STRUCT,
                    payload
                )

                messages.append({
                    "type": "telemetry",
                    "seq": seq,
                    "kick_status": kick_status,
                    "motor_fault": motor_fault,
                    "ultrasonic": (
                        ultrasonic_0,
                        ultrasonic_1,
                        ultrasonic_2,
                        ultrasonic_3
                    ),
                    "ultrasonic_status": ultrasonic_status,
                    "error_status": error_status,
                    "system_status": system_status
                })

            elif frame_type == TYPE_KICK_ACK:

                kick_id, status = _struct.unpack(
                    _KICK_STRUCT,
                    payload
                )

                messages.append({
                    "type": "kick_ack",
                    "seq": seq,
                    "kick_id": kick_id,
                    "status": status
                })

            elif frame_type == TYPE_KICK_RESULT:

                kick_id, status = _struct.unpack(
                    _KICK_STRUCT,
                    payload
                )

                self.kick_result_ack_id = kick_id

                messages.append({
                    "type": "kick_result",
                    "seq": seq,
                    "kick_id": kick_id,
                    "status": status
                })

        return messages


# ============================================================
# CRC self-test
# ============================================================

assert _crc16(b"123456789", 0, 9) == 0x29B1
assert _COMMAND_SIZE == 19
assert _TELEMETRY_SIZE == 17
assert _KICK_SIZE == 3
