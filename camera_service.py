#!/usr/bin/env python3
"""Optional local camera runtime for the dashboard.

The workcell model owns persistent camera configuration and tagged part poses. This
service only owns live device capture and JPEG frame access, so the dashboard
still works when OpenCV or a physical camera is unavailable.
"""

from __future__ import annotations

import json
import platform
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

try:  # pragma: no cover - depends on local machine packages/devices
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


class CameraService:
    _MAC_CAMERA_DISCOVERY = r'''
import AVFoundation
import Foundation

let devices = AVCaptureDevice.devices(for: .video) + AVCaptureDevice.devices(for: .muxed)
let rows: [[String: Any]] = devices.enumerated().map { index, device in
    return [
        "id": index,
        "label": device.localizedName,
        "uniqueId": device.uniqueID,
        "deviceType": device.deviceType.rawValue
    ]
}
let data = try! JSONSerialization.data(withJSONObject: rows, options: [])
print(String(data: data, encoding: .utf8)!)
'''

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.lock = threading.Lock()
        self.config = dict(config or {})
        self.capture = None
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.running = False
        self.last_error: Optional[str] = None
        self.last_frame_at: Optional[float] = None
        self.last_frame_size: Optional[Dict[str, int]] = None
        self.frame_count = 0
        self._jpeg: Optional[bytes] = None
        self._device_cache: List[Dict[str, Any]] = []
        self._device_cache_at = 0.0

    def configure(self, config: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            merged = dict(self.config)
            merged.update(config or {})
            self.config = merged
        return self.status()

    def list_devices(self, max_index: int = 6, probe_opencv: bool = False) -> List[Dict[str, Any]]:
        # Opening sequential indices to discover cameras activates Continuity
        # Camera on macOS. Enumerate metadata only and expose external devices;
        # neither the iPhone nor the built-in FaceTime camera is ever opened.
        if platform.system() == "Darwin":
            if not probe_opencv and self._device_cache:
                return [dict(device) for device in self._device_cache]
            if not probe_opencv:
                return []
            devices = self._list_macos_external_devices()
            self._device_cache = devices
            self._device_cache_at = time.time()
            return [dict(device) for device in devices]

        devices: List[Dict[str, Any]] = []
        if not probe_opencv:
            index = self._resolve_source_locked()
            return [{"id": index, "label": f"Camera {index}", "source": "opencv"}]
        if cv2 is None:
            return devices
        for index in range(max_index):
            cap = cv2.VideoCapture(index)
            opened = bool(cap and cap.isOpened())
            if opened:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                devices.append({
                    "id": index,
                    "label": f"Camera {index}",
                    "width": width,
                    "height": height,
                    "source": "opencv",
                })
            if cap:
                cap.release()
        return devices

    @staticmethod
    def _is_external_macos_camera(device: Dict[str, Any]) -> bool:
        label = str(device.get("label") or "").casefold()
        device_type = str(device.get("deviceType") or "").casefold()
        blocked_terms = (
            "continuity", "iphone", "ipad", "facetime", "built-in",
            "builtin", "desk view",
        )
        if any(term in label or term in device_type for term in blocked_terms):
            return False
        return "external" in device_type

    def _list_macos_external_devices(self) -> List[Dict[str, Any]]:
        try:
            completed = subprocess.run(
                ["/usr/bin/swift", "-e", self._MAC_CAMERA_DISCOVERY],
                capture_output=True,
                text=True,
                # The first Swift/AVFoundation metadata query can spend more
                # than ten seconds warming Apple's compiler/module cache. It
                # does not open a video device, so allow it to finish instead
                # of falsely reporting that no external camera exists.
                timeout=30,
                check=False,
            )
            if completed.returncode != 0:
                self.last_error = "External camera discovery failed."
                return []
            rows = json.loads(completed.stdout.strip() or "[]")
        except (OSError, subprocess.SubprocessError, json.JSONDecodeError):
            self.last_error = "External camera discovery failed."
            return []
        devices = []
        # OpenCV's macOS AVFoundation backend does not use Apple's discovery
        # order. It combines video/muxed devices and sorts them by uniqueID
        # before applying the integer camera index. Mirror that source code
        # exactly so selecting Lenovo cannot resolve to FaceTime at open time.
        ordered_rows = sorted(
            (row for row in rows if isinstance(row, dict)),
            key=lambda row: str(row.get("uniqueId") or ""),
        ) if isinstance(rows, list) else []
        for opencv_index, row in enumerate(ordered_rows):
            if not isinstance(row, dict) or not self._is_external_macos_camera(row):
                continue
            devices.append({
                "id": opencv_index,
                "label": str(row.get("label") or f"External Camera {opencv_index}"),
                "uniqueId": str(row.get("uniqueId") or ""),
                "deviceType": str(row.get("deviceType") or "external"),
                "source": "avfoundation_external",
                "external": True,
                "indexOrder": "opencv_unique_id_sorted",
            })
        self.last_error = None if devices else "No external USB camera found."
        return devices

    def start(self) -> Dict[str, Any]:
        if cv2 is None:
            self.last_error = "OpenCV is not installed; camera capture is unavailable."
            return self.status(ok=False)
        with self.lock:
            if self.running:
                return self.status_locked()
            source = self._resolve_source_locked()
            if source is None:
                return self.status_locked(ok=False)
            if platform.system() == "Darwin":
                allowed = self.list_devices(probe_opencv=True)
                requested_unique_id = str(self.config.get("deviceUniqueId") or "")
                selected = next(
                    (device for device in allowed if requested_unique_id and device.get("uniqueId") == requested_unique_id),
                    None,
                )
                if requested_unique_id and selected is None:
                    self.last_error = (
                        "The selected external camera is not connected. Refusing to open "
                        "a different camera; reconnect it and click Find Cameras."
                    )
                    return self.status_locked(ok=False)
                if selected is None:
                    selected = next((device for device in allowed if int(device["id"]) == source), None)
                if selected is None:
                    if allowed:
                        selected = allowed[0]
                    else:
                        self.last_error = (
                            "No external USB camera was found. The Mac's built-in camera "
                            "and Continuity/iPhone cameras are intentionally blocked."
                        )
                        return self.status_locked(ok=False)
                source = int(selected["id"])
                self.config["deviceId"] = source
                self.config["deviceUniqueId"] = str(selected.get("uniqueId") or "")
                self.config["deviceLabel"] = str(selected.get("label") or "External Camera")
                cap = cv2.VideoCapture(source, cv2.CAP_AVFOUNDATION)
            else:
                cap = cv2.VideoCapture(source)
            if not cap or not cap.isOpened():
                self.last_error = f"Could not open camera device {source!r}."
                if cap:
                    cap.release()
                return self.status_locked(ok=False)
            width = int(self.config.get("width") or 0)
            height = int(self.config.get("height") or 0)
            if width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.capture = cap
            self.stop_event.clear()
            self.running = True
            self.last_error = None
            self.thread = threading.Thread(target=self._capture_loop, name="camera-capture", daemon=True)
            self.thread.start()
            return self.status_locked()

    def _resolve_source_locked(self) -> Optional[Any]:
        source = self.config.get("deviceId", 0)
        try:
            return int(source)
        except (TypeError, ValueError):
            return 0

    def stop(self) -> Dict[str, Any]:
        self.stop_event.set()
        thread = self.thread
        if thread and thread.is_alive():
            thread.join(timeout=1.5)
        with self.lock:
            self._release_locked()
            return self.status_locked()

    def _release_locked(self) -> None:
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:
                pass
        self.capture = None
        self.thread = None
        self.running = False
        # A stopped or failed camera must never leave a calibration/localization
        # endpoint consuming the last successful frame as though it were live.
        self._jpeg = None
        self.last_frame_at = None
        self.last_frame_size = None

    def _capture_loop(self) -> None:  # pragma: no cover - hardware path
        while not self.stop_event.is_set():
            with self.lock:
                cap = self.capture
                quality = int(self.config.get("jpegQuality") or 82)
            if cap is None:
                break
            ok, frame = cap.read()
            if not ok:
                with self.lock:
                    self.last_error = "Camera read failed."
                time.sleep(0.1)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if not ok:
                with self.lock:
                    self.last_error = "JPEG encode failed."
                continue
            height, width = frame.shape[:2]
            with self.lock:
                self._jpeg = bytes(encoded)
                self.last_frame_at = time.time()
                self.last_frame_size = {"width": int(width), "height": int(height)}
                self.frame_count += 1
                self.last_error = None
            time.sleep(0.03)
        with self.lock:
            self._release_locked()

    def get_jpeg(self) -> Optional[bytes]:
        with self.lock:
            if not self.running or self._jpeg is None or self.last_frame_at is None:
                return None
            stale_after = max(0.25, float(self.config.get("staleAfterS") or 3.0))
            if time.time() - self.last_frame_at > stale_after:
                return None
            return self._jpeg

    def is_running(self) -> bool:
        with self.lock:
            return self.running

    def status_locked(self, ok: bool = True) -> Dict[str, Any]:
        return {
            "ok": bool(ok),
            "available": cv2 is not None,
            "running": self.running,
            "config": dict(self.config),
            "lastError": self.last_error,
            "lastFrameAt": self.last_frame_at,
            "lastFrameSize": self.last_frame_size,
            "frameCount": self.frame_count,
            "devices": self.list_devices(probe_opencv=False),
            "devicePolicy": "external_only" if platform.system() == "Darwin" else "opencv",
            "streamUrl": "/api/camera/stream",
            "frameUrl": "/api/camera/frame",
        }

    def status(self, ok: bool = True) -> Dict[str, Any]:
        with self.lock:
            return self.status_locked(ok=ok)
