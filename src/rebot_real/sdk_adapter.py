from __future__ import annotations

import importlib.util
from pathlib import Path
import re
import socket
import sys
import time
from types import ModuleType
from typing import Any, Callable


class UdpStateSubscriber:
    """Receive the SDK public UDP JointState without opening a TCP control session."""

    def __init__(
        self,
        sdk_root: str | Path,
        *,
        bind_ip: str = "0.0.0.0",
        udp_port: int = 5001,
        timeout_s: float = 0.2,
        decoder: Callable[[bytes], Any] | None = None,
    ) -> None:
        self.sdk_root = Path(sdk_root).expanduser().resolve()
        self.bind_ip = bind_ip
        self.udp_port = int(udp_port)
        self.timeout_s = float(timeout_s)
        if not (0 < self.udp_port <= 65535):
            raise ValueError("udp_port must be in [1, 65535]")
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self._decoder = decoder or _load_protocol_module(self.sdk_root).decode_joint_state
        self._socket: socket.socket | None = None

    def open(self) -> None:
        if self._socket is not None:
            return
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Deliberately do not set SO_REUSEADDR: one state recorder owns the port.
            udp_socket.bind((self.bind_ip, self.udp_port))
            udp_socket.settimeout(self.timeout_s)
        except Exception:
            udp_socket.close()
            raise
        self._socket = udp_socket

    def receive(self) -> tuple[int, Any] | None:
        if self._socket is None:
            raise RuntimeError("UDP state subscriber is not open")
        try:
            payload, _ = self._socket.recvfrom(65536)
        except socket.timeout:
            return None
        # Host timestamp is taken immediately after recvfrom returns, before JSON decode.
        timestamp_host_rx_ns = time.monotonic_ns()
        return timestamp_host_rx_ns, self._decoder(payload)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def __enter__(self) -> "UdpStateSubscriber":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def _load_protocol_module(sdk_root: Path) -> ModuleType:
    """Load only the SDK protocol decoder; never import/construct ArmClient."""

    protocol_path = sdk_root / "upper" / "python" / "wlsea_arm_sdk" / "protocol.py"
    if not protocol_path.is_file():
        raise FileNotFoundError(f"SDK protocol.py not found: {protocol_path}")
    spec = importlib.util.spec_from_file_location("_rebot_sdk_protocol", protocol_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load SDK protocol module: {protocol_path}")
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves annotation helpers through sys.modules while the
    # SDK protocol module is executing, so register this private module first.
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(spec.name, None)
        raise
    if not hasattr(module, "decode_joint_state"):
        raise RuntimeError("SDK protocol module has no decode_joint_state")
    return module


def detect_sdk_version(sdk_root: str | Path) -> str:
    """Read the handoff package version without importing the full SDK package."""

    init_path = (
        Path(sdk_root).expanduser().resolve()
        / "upper"
        / "python"
        / "wlsea_arm_sdk"
        / "__init__.py"
    )
    if not init_path.is_file():
        return "unknown"
    match = re.search(
        r'^__version__\s*=\s*["\']([^"\']+)["\']',
        init_path.read_text(encoding="utf-8"),
        flags=re.MULTILINE,
    )
    return match.group(1) if match else "unknown"
