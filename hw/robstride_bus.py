"""
python-can Bus implementation for the RobStride CAN2USB debugger.

The CAN2USB debugger uses a CH340 USB-serial chip and speaks
an AT-framed binary protocol over serial:

TX/RX frame format:
  'A' 'T' [4-byte wire ID] [1-byte data len] [data bytes] '\r' '\n'

Wire ID encoding:
  wire_id = (29-bit CAN arbitration ID << 3) | 0x04  (big-endian)

Confirmed working at 921600 baud on Windows (COM5).

Usage with robstride.Client:
    from robstride_bus import RobStrideBus
    from robstride import Client

    bus = RobStrideBus("COM5")
    client = Client(bus)
    client.enable(127)
"""

import struct
import time
import logging
from typing import Optional

import serial
from can import BusABC, Message

logger = logging.getLogger(__name__)

HEADER = b'AT'
TERMINATOR = b'\r\n'
MIN_FRAME_LEN = 9  # AT(2) + wire_id(4) + len(1) + terminator(2), zero data


class RobStrideBus(BusABC):
    """python-can Bus for the RobStride CAN2USB debugger (AT serial protocol)."""

    def __init__(self, channel: str, ttyBaudrate: int = 921600, **kwargs):
        super().__init__(channel=channel, **kwargs)
        self._ser = serial.Serial(
            port=channel,
            baudrate=ttyBaudrate,
            timeout=0.01,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._ser.dtr = True
        self._ser.rts = True
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._rxbuf = bytearray()
        self.channel_info = f"RobStride CAN2USB on {channel} @ {ttyBaudrate}"
        time.sleep(0.05)

    @staticmethod
    def _encode_arb_id(arb_id: int) -> bytes:
        wire_id = (arb_id << 3) | 0x04
        return struct.pack('>I', wire_id)

    @staticmethod
    def _decode_wire_id(data: bytes) -> int:
        wire_id = struct.unpack('>I', data)[0]
        return wire_id >> 3

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        frame = HEADER
        frame += self._encode_arb_id(msg.arbitration_id)
        frame += bytes([msg.dlc])
        frame += bytes(msg.data[:msg.dlc])
        frame += TERMINATOR

        if timeout is not None and timeout != self._ser.write_timeout:
            self._ser.write_timeout = timeout

        self._ser.write(frame)
        self._ser.flush()

    def _recv_internal(self, timeout: Optional[float]):
        deadline = time.time() + (timeout if timeout is not None else 0)

        while True:
            # Read available bytes into buffer
            if self._ser.in_waiting > 0:
                self._rxbuf += self._ser.read(self._ser.in_waiting)

            # Try to extract a frame from the buffer
            msg = self._try_parse_frame()
            if msg is not None:
                return msg, False

            # Check timeout
            if timeout is not None and time.time() >= deadline:
                return None, False

            # Brief sleep to avoid busy-spinning
            remaining = deadline - time.time() if timeout is not None else 0.1
            if remaining > 0:
                self._ser.timeout = min(remaining, 0.05)
                chunk = self._ser.read(max(1, self._ser.in_waiting))
                if chunk:
                    self._rxbuf += chunk
            else:
                return None, False

    def _try_parse_frame(self) -> Optional[Message]:
        while True:
            # Find 'AT' header
            idx = self._rxbuf.find(HEADER)
            if idx < 0:
                # No header found; keep last byte in case it's 'A' of next frame
                if len(self._rxbuf) > 1:
                    del self._rxbuf[:-1]
                return None

            # Discard garbage before header
            if idx > 0:
                logger.debug("Discarding %d bytes before AT header", idx)
                del self._rxbuf[:idx]

            # Need at least MIN_FRAME_LEN bytes
            if len(self._rxbuf) < MIN_FRAME_LEN:
                return None

            data_len = self._rxbuf[6]
            frame_len = 7 + data_len + 2  # header(2) + wire_id(4) + len(1) + data + term(2)

            if len(self._rxbuf) < frame_len:
                return None

            # Verify terminator
            if self._rxbuf[frame_len - 2:frame_len] != TERMINATOR:
                # Bad frame, skip this 'AT' and search for next
                logger.debug("Bad terminator, skipping frame")
                del self._rxbuf[:2]
                continue

            # Extract the frame
            frame = bytes(self._rxbuf[:frame_len])
            del self._rxbuf[:frame_len]

            arb_id = self._decode_wire_id(frame[2:6])
            data = frame[7:7 + data_len]

            return Message(
                arbitration_id=arb_id,
                is_extended_id=True,
                timestamp=time.time(),
                dlc=data_len,
                data=data,
            )

    def shutdown(self) -> None:
        super().shutdown()
        if self._ser.is_open:
            self._ser.close()
