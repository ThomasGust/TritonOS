"""Simple GPIO leak input.

Navigator leak detector routing can be setup-dependent. In some installations
the leak detector output is routed to a Raspberry Pi GPIO line.

This helper reads a single GPIO line using libgpiod.
"""

from __future__ import annotations

from dataclasses import dataclass

try:
    import gpiod  # type: ignore
except Exception:  # pragma: no cover
    gpiod = None


@dataclass
class LeakGPIO:
    chip: str = "/dev/gpiochip0"
    line: int = 0
    invert: bool = False

    def __post_init__(self) -> None:
        if gpiod is None:
            raise RuntimeError("python gpiod not installed")
        self._chip = gpiod.Chip(self.chip)
        self._line = self._chip.get_line(int(self.line))
        self._line.request(consumer="triton_leak", type=gpiod.LINE_REQ_DIR_IN)

    def read(self) -> bool:
        v = bool(self._line.get_value())
        return (not v) if self.invert else v

    def close(self) -> None:
        try:
            self._line.release()
        except Exception:
            pass
        try:
            self._chip.close()
        except Exception:
            pass
