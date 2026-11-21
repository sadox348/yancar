#!/usr/bin/env python3
"""Pure-Python codec for the STM32F407 AGV UART protocol."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


class ProtocolError(ValueError):
    """Raised when a serial frame does not match the documented protocol."""


@dataclass(frozen=True)
class StatFrame:
    enabled: int
    left_target: int
    left_actual: int
    left_pwm: int
    right_target: int
    right_actual: int
    right_pwm: int
    left_feedforward: int
    right_feedforward: int


@dataclass(frozen=True)
class ParsedFrame:
    kind: str
    fields: Tuple[str, ...]
    stat: Optional[StatFrame] = None


def _clamp(value: int, limit: int) -> int:
    if limit <= 0:
        raise ValueError("limit must be positive")
    return max(-limit, min(limit, value))


def encode_velocity_command(linear: float, angular: float,
                            linear_scale: float, angular_scale: float,
                            max_command: int) -> str:
    """Convert a Twist-like command to the integer frame expected by STM32."""
    values = (linear, angular, linear_scale, angular_scale)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("command values and scales must be finite")
    linear_int = _clamp(int(round(linear * linear_scale)), max_command)
    angular_int = _clamp(int(round(angular * angular_scale)), max_command)
    return f"V,{linear_int},{angular_int}\r\n"


def parse_frame(raw: str) -> ParsedFrame:
    line = raw.strip()
    if not line:
        raise ProtocolError("empty frame")

    parts = tuple(part.strip() for part in line.split(","))
    kind = parts[0].upper()

    if kind == "STAT":
        if len(parts) != 10:
            raise ProtocolError(f"STAT requires 9 fields, got {len(parts) - 1}")
        try:
            values = [int(value) for value in parts[1:]]
        except ValueError as exc:
            raise ProtocolError("STAT fields must be integers") from exc
        return ParsedFrame(kind, parts[1:], StatFrame(*values))

    if kind in {"ACK", "EVT", "ERR", "VER"}:
        return ParsedFrame(kind, parts[1:])

    raise ProtocolError(f"unknown frame type: {parts[0]}")

