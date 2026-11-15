"""Pruebas de integración sin acceder a cámaras, COM ni motores reales."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import MGApp as app


class CameraTests(unittest.TestCase):
    def camera(self, opened=True, readable=True):
        cap = MagicMock()
        cap.isOpened.return_value = opened
        cap.read.return_value = (readable, object() if readable else None)
        return cap

    def test_windows_falls_back_when_directshow_has_no_frames(self):
        first, second = self.camera(readable=False), self.camera()
        with patch.object(app.sys, 'platform', 'win32'), \
                patch.object(app.cv2, 'VideoCapture', side_effect=[first, second]) as factory:
            self.assertIs(app.open_video_source(2), second)
        self.assertEqual(factory.call_args_list[0].args, (2, app.cv2.CAP_DSHOW))
        self.assertEqual(factory.call_args_list[1].args, (2, app.cv2.CAP_MSMF))
        first.release.assert_called_once()
        second.release.assert_not_called()

    def test_windows_releases_all_failed_candidates(self):
        caps = [self.camera(opened=False) for _ in range(3)]
        with patch.object(app.sys, 'platform', 'win32'), \
                patch.object(app.cv2, 'VideoCapture', side_effect=caps):
            with self.assertRaises(RuntimeError):
                app.open_video_source(7)
        for cap in caps:
            cap.release.assert_called_once()

    def test_driver_exception_releases_camera_and_tries_next_backend(self):
        first, second = self.camera(), self.camera()
        first.read.side_effect = app.cv2.error('driver failure')
        with patch.object(app.sys, 'platform', 'win32'), \
                patch.object(app.cv2, 'VideoCapture', side_effect=[first, second]):
            self.assertIs(app.open_video_source(0), second)
        first.release.assert_called_once()

    def test_macos_keeps_its_native_backend(self):
        cap = self.camera()
        with patch.object(app.sys, 'platform', 'darwin'), \
                patch.object(app.cv2, 'VideoCapture', return_value=cap) as factory:
            self.assertIs(app.open_video_source(1), cap)
        self.assertEqual(factory.call_args.args, (1, app.cv2.CAP_AVFOUNDATION))

    def test_invalid_urls_never_open_a_camera(self):
        with patch.object(app.cv2, 'VideoCapture') as factory:
            for url in ('', '192.168.1.20', 'file:///tmp/movie.mp4', 'http://'):
                with self.subTest(url=url), self.assertRaises(ValueError):
                    app.open_video_source(source='iphone', url=url)
            factory.assert_not_called()

    def test_network_uses_ffmpeg_timeouts_and_preserves_stream_size(self):
        cap = self.camera()
        with patch.object(app.cv2, 'VideoCapture', return_value=cap) as factory:
            self.assertIs(app.open_video_source(source='iphone',
                          url=' http://192.168.1.20:4747/video '), cap)
        self.assertEqual(factory.call_args.args, (
            'http://192.168.1.20:4747/video', app.cv2.CAP_FFMPEG,
            [app.cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
             app.cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000]))
        cap.set.assert_called_once_with(app.cv2.CAP_PROP_BUFFERSIZE, 1)

    def test_failed_network_connection_is_released(self):
        cap = self.camera(opened=False)
        with patch.object(app.cv2, 'VideoCapture', return_value=cap):
            with self.assertRaises(RuntimeError):
                app.open_video_source(source='iphone', url='rtsp://192.168.1.20/live')
        cap.release.assert_called_once()


class SerialTests(unittest.TestCase):
    def sender(self):
        return SimpleNamespace(
            simulacion=False, paused=False, connected=True, calibrating=False,
            _camera_opening=False, _closing=False, last_trama='<1,1|2,2|3,3>\n',
            last_serial_time=1.0, serial_conn=MagicMock())

    def test_windows_com_ports_are_included_and_sorted(self):
        ports = [SimpleNamespace(device=p) for p in ('COM10', 'COM3', 'COM1')]
        with patch.object(app.list_ports, 'comports', return_value=ports):
            self.assertEqual(app.App._get_serial_ports(None), ['COM1', 'COM3', 'COM10'])

    def test_no_fake_ports_when_nothing_is_connected(self):
        with patch.object(app.list_ports, 'comports', return_value=[]):
            self.assertEqual(app.App._get_serial_ports(None), [])

    def test_unchanged_commands_refresh_before_slave_watchdog(self):
        sender = self.sender()
        with patch.object(app.time, 'monotonic', return_value=1.11):
            app.App.send_serial_if_needed(sender, sender.last_trama)
        sender.serial_conn.write.assert_called_once_with(sender.last_trama.encode())
        self.assertLess(app.SERIAL_REFRESH_SECONDS, 0.250)
        self.assertEqual(sender.last_serial_time, 1.11)

    def test_changed_commands_are_limited_to_20_hz(self):
        sender = self.sender()
        with patch.object(app.time, 'monotonic', return_value=1.02):
            app.App.send_serial_if_needed(sender, '<4,4|5,5|6,6>\n')
        sender.serial_conn.write.assert_not_called()
        with patch.object(app.time, 'monotonic', return_value=1.06):
            app.App.send_serial_if_needed(sender, '<4,4|5,5|6,6>\n')
        sender.serial_conn.write.assert_called_once_with(b'<4,4|5,5|6,6>\n')

    def test_no_motion_packets_during_pause_setup_or_simulation(self):
        for flag in ('paused', 'calibrating', '_camera_opening', '_closing', 'simulacion'):
            sender = self.sender()
            setattr(sender, flag, True)
            with self.subTest(flag=flag), patch.object(app.time, 'monotonic', return_value=2.0):
                app.App.send_serial_if_needed(sender, sender.last_trama)
                sender.serial_conn.write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
