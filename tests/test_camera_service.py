import json
import unittest
from unittest.mock import MagicMock, patch

import camera_service
from camera_service import CameraService


MAC_CAMERAS = [
    {
        "id": 0,
        "label": "FaceTime HD Camera",
        "uniqueId": "3F45E80A-0176-46F7-B185-BB9E2C0E82E3",
        "deviceType": "AVCaptureDeviceTypeBuiltInWideAngleCamera",
    },
    {
        "id": 1,
        "label": "Jack's iPhone Camera",
        "uniqueId": "E5A41A6A-3FF4-4B98-A6C8-CDFA00000001",
        "deviceType": "AVCaptureDeviceTypeContinuityCamera",
    },
    {
        "id": 2,
        "label": "Lenovo FHDWC310",
        "uniqueId": "0x113000017ef485b",
        "deviceType": "AVCaptureDeviceTypeExternal",
    },
]


class CameraDevicePolicyTests(unittest.TestCase):
    @patch("camera_service.platform.system", return_value="Darwin")
    @patch("camera_service.subprocess.run")
    def test_macos_discovery_never_opens_and_only_returns_external_usb(self, run, _system):
        run.return_value = MagicMock(returncode=0, stdout=json.dumps(MAC_CAMERAS), stderr="")
        service = CameraService({"deviceId": 0})
        with patch.object(camera_service.cv2, "VideoCapture") as video_capture:
            devices = service.list_devices(probe_opencv=True)
        video_capture.assert_not_called()
        self.assertEqual(run.call_args.kwargs["timeout"], 30)
        # OpenCV sorts by unique ID, so Lenovo's 0x... ID is camera index 0
        # even though Apple's unsorted discovery listed it third here.
        self.assertEqual([device["id"] for device in devices], [0])
        self.assertEqual(devices[0]["label"], "Lenovo FHDWC310")
        self.assertTrue(devices[0]["external"])
        self.assertEqual(devices[0]["indexOrder"], "opencv_unique_id_sorted")

    @patch("camera_service.platform.system", return_value="Darwin")
    @patch("camera_service.threading.Thread")
    def test_start_replaces_unsafe_saved_index_with_external_camera(self, thread, _system):
        service = CameraService({"deviceId": 1, "deviceUniqueId": "usb-1"})
        service.list_devices = MagicMock(return_value=[{
            "id": 0, "label": "Lenovo FHDWC310", "uniqueId": "usb-1", "external": True,
        }])
        capture = MagicMock()
        capture.isOpened.return_value = True
        with patch.object(camera_service.cv2, "VideoCapture", return_value=capture) as video_capture:
            status = service.start()
        video_capture.assert_called_once_with(0, camera_service.cv2.CAP_AVFOUNDATION)
        thread.return_value.start.assert_called_once()
        self.assertTrue(status["running"])
        self.assertEqual(status["config"]["deviceId"], 0)
        self.assertEqual(status["config"]["deviceUniqueId"], "usb-1")

    @patch("camera_service.platform.system", return_value="Darwin")
    def test_start_never_substitutes_another_camera_for_saved_unique_id(self, _system):
        service = CameraService({"deviceId": 0, "deviceUniqueId": "lenovo-id"})
        service.list_devices = MagicMock(return_value=[{
            "id": 0, "label": "Other External Camera", "uniqueId": "other-id", "external": True,
        }])
        with patch.object(camera_service.cv2, "VideoCapture") as video_capture:
            status = service.start()
        video_capture.assert_not_called()
        self.assertFalse(status["ok"])
        self.assertIn("Refusing to open a different camera", status["lastError"])

    @patch("camera_service.platform.system", return_value="Darwin")
    def test_start_refuses_builtin_and_phone_when_no_external_camera_exists(self, _system):
        service = CameraService({"deviceId": 0})
        service.list_devices = MagicMock(return_value=[])
        with patch.object(camera_service.cv2, "VideoCapture") as video_capture:
            status = service.start()
        video_capture.assert_not_called()
        self.assertFalse(status["ok"])
        self.assertFalse(status["running"])
        self.assertIn("intentionally blocked", status["lastError"])

    def test_phone_and_builtin_names_are_blocked_even_if_mislabeled_external(self):
        for label in ("Continuity Camera", "Jack's iPhone", "FaceTime HD Camera", "Built-in Camera"):
            self.assertFalse(CameraService._is_external_macos_camera({
                "label": label,
                "deviceType": "AVCaptureDeviceTypeExternal",
            }))


if __name__ == "__main__":
    unittest.main()
