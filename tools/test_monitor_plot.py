"""Tests for scripts/ground/monitor_plot.py (the --listen --plot window)."""

import contextlib
import csv
import importlib
import io
import socket
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import matplotlib

matplotlib.use("Agg")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from ground import monitor_plot  # noqa: E402

GOOD_PACKET = ",".join(["1"] * 35).encode()


class FakeSocket:
    def __init__(self, packets):
        self.packets = list(packets)
        self.sent = []
        self.closed = False

    def recvfrom(self, _size):
        if not self.packets:
            raise socket.timeout()
        return self.packets.pop(0), ("192.168.4.1", 4210)

    def bind(self, _addr):
        pass

    def settimeout(self, _t):
        pass

    def sendto(self, data, addr):
        self.sent.append((data, addr))

    def close(self):
        self.closed = True


class MonitorPlotTest(unittest.TestCase):
    def _monitor(self, packets, tmp):
        sock = FakeSocket(packets)
        csv_file = open(Path(tmp) / "log.csv", "w", newline="", encoding="utf-8")
        self.addCleanup(csv_file.close)
        monitor = monitor_plot.Monitor(sock, csv_file)
        self.addCleanup(monitor_plot.plt_close, monitor.fig)
        return monitor, sock, csv_file

    def test_import_has_no_side_effects(self):
        with mock.patch.object(socket, "socket") as s, \
                mock.patch.object(Path, "mkdir") as mkdir, \
                mock.patch.object(Path, "open") as popen:
            importlib.reload(monitor_plot)
        s.assert_not_called()
        mkdir.assert_not_called()
        popen.assert_not_called()

    def test_log_dir_comes_from_ground_paths(self):
        from ground import paths

        self.assertEqual(monitor_plot.LOG_DIR, paths.LOG_DIR)

    def test_update_consumes_sample(self):
        with tempfile.TemporaryDirectory() as tmp:
            monitor, sock, csv_file = self._monitor([GOOD_PACKET], tmp)
            monitor.update(0)
            csv_file.flush()
            self.assertEqual(len(monitor.roll_data), 1)
            self.assertEqual(monitor.packet_count, 1)
            self.assertEqual(monitor.bad_packet_count, 0)
            with open(Path(tmp) / "log.csv", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(len(rows), 2)  # header + 1 sample
            # connect keepalive goes out, and only connect
            self.assertEqual([d for d, _ in sock.sent], [b"connect"])

    def test_update_counts_bad_packets_and_skips_gains(self):
        with tempfile.TemporaryDirectory() as tmp:
            monitor, _sock, _f = self._monitor(
                [b"garbage", b"\xff\xfe", b"GAINS,1,2,3", GOOD_PACKET], tmp)
            with contextlib.redirect_stdout(io.StringIO()):
                monitor.update(0)
            self.assertEqual(monitor.bad_packet_count, 2)
            self.assertEqual(monitor.packet_count, 1)

    def test_four_panels(self):
        with tempfile.TemporaryDirectory() as tmp:
            monitor, _s, _f = self._monitor([], tmp)
            self.assertEqual(len(monitor.axes), 4)

    def test_main_runs_headless_and_closes_everything(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = FakeSocket([GOOD_PACKET])
            with mock.patch.object(monitor_plot, "LOG_DIR", Path(tmp)), \
                    mock.patch.object(monitor_plot.socket, "socket",
                                      return_value=fake), \
                    mock.patch.object(monitor_plot, "show_figure") as show, \
                    contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(monitor_plot.main([]), 0)
            show.assert_called_once()
            monitor_plot.plt_close("all")
            self.assertTrue(fake.closed)
            self.assertEqual(len(list(Path(tmp).glob("flight_log_*.csv"))), 1)
            self.assertIn("잘못된 패킷", out.getvalue())


if __name__ == "__main__":
    unittest.main()
