"""python-can compatible transport for the official RobStride CH340 adapter."""

from __future__ import annotations

import time

import can
import serial


class RobstrideSerialBus:
    """Wrap RobStride's AT-framed serial protocol with send/recv methods."""

    def __init__(
        self,
        channel: str,
        bitrate: int = 1_000_000,
        baudrate: int = 921_600,
        timeout: float = 0.1,
        **_kwargs,
    ):
        if bitrate != 1_000_000:
            raise ValueError(
                "The official RobStride adapter transport currently supports "
                "the motor's default 1 Mbit/s CAN configuration only"
            )
        self.channel = channel
        self.timeout = timeout
        self.serial = serial.Serial(channel, baudrate, timeout=timeout)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        self._buffer = bytearray()

    def send(self, frame: can.Message, timeout: float | None = None) -> None:
        if not frame.is_extended_id:
            raise ValueError("RobStride motors require 29-bit extended CAN IDs")
        if len(frame.data) > 8:
            raise ValueError("Classic CAN payloads cannot exceed 8 bytes")

        # The adapter stores CAN flags in the low three bits. Bit 2 selects an
        # extended frame; the 29-bit arbitration ID occupies the upper bits.
        encoded_id = (int(frame.arbitration_id) << 3) | 0x04
        packet = (
            b"AT"
            + encoded_id.to_bytes(4, "big")
            + bytes([len(frame.data)])
            + bytes(frame.data)
            + b"\r\n"
        )

        original_timeout = self.serial.write_timeout
        try:
            self.serial.write_timeout = timeout
            self.serial.write(packet)
            self.serial.flush()
        finally:
            self.serial.write_timeout = original_timeout

    def recv(self, timeout: float | None = None) -> can.Message | None:
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            message = self._extract_message()
            if message is not None:
                return message

            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.serial.timeout = min(self.timeout, remaining)
            else:
                self.serial.timeout = self.timeout

            # pyserial waits for the requested byte count or the timeout.
            # Read one blocking byte, then immediately drain what is already
            # buffered so a short CAN packet does not incur the full timeout.
            chunk = self.serial.read(1)
            if chunk:
                self._buffer.extend(chunk)
                waiting = self.serial.in_waiting
                if waiting:
                    self._buffer.extend(self.serial.read(waiting))
            elif deadline is not None and time.monotonic() >= deadline:
                return None

    def _extract_message(self) -> can.Message | None:
        while True:
            start = self._buffer.find(b"AT")
            if start < 0:
                if self._buffer[-1:] == b"A":
                    self._buffer[:] = b"A"
                else:
                    self._buffer.clear()
                return None
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 9:
                return None

            data_length = self._buffer[6]
            total_length = 9 + data_length
            if data_length > 8:
                del self._buffer[0]
                continue
            if len(self._buffer) < total_length:
                return None
            if self._buffer[total_length - 2 : total_length] != b"\r\n":
                del self._buffer[0]
                continue

            encoded_id = int.from_bytes(self._buffer[2:6], "big")
            arbitration_id = (encoded_id >> 3) & 0x1FFF_FFFF
            data = bytes(self._buffer[7 : 7 + data_length])
            del self._buffer[:total_length]
            return can.Message(
                arbitration_id=arbitration_id,
                is_extended_id=True,
                data=data,
            )

    def shutdown(self) -> None:
        self.serial.close()
