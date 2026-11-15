"""Verifica que los objetos mantienen su memoria de forma independiente."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import MGApp as app


class ObjectStateTests(unittest.TestCase):
    def test_goalkeeper_pid_is_independent(self):
        first = app.GoalkeeperStrategy()
        second = app.GoalkeeperStrategy()
        robot = {'pos': (70, 325), 'angle': 0}
        args = (robot, (90, 350), (0, 325), (750, 325), 750, 650)
        initial = first(*args)
        first(*args)
        self.assertEqual(second(*args), initial)
        self.assertIsNot(first._st, second._st)
        self.assertEqual(first._st['zona_frames'], 2)
        self.assertEqual(second._st['zona_frames'], 1)

    def test_attacker_pid_is_independent(self):
        first = app.AttackerStrategy()
        second = app.AttackerStrategy()
        robot = {'pos': (400, 325), 'angle': 0}
        args = (robot, (500, 350), (0, 325), (750, 325), 750, 650,
                [], app.RobotPredictor(), 750, 650, 35, 80, 30, 90, 35)
        initial = first(*args)
        first(*args)
        self.assertEqual(second(*args), initial)
        self.assertIsNot(first._st, second._st)

    def test_perspective_cache_belongs_to_image_processor(self):
        first = app.ImageProcessing()
        second = app.ImageProcessing()
        frame = np.zeros((40, 40, 3), dtype=np.uint8)
        points = [(0, 0), (39, 0), (39, 39), (0, 39)]
        warped, width, height = first.warp_image(frame, points, (20, 20))
        self.assertEqual(warped.shape, (20, 20, 3))
        self.assertEqual((width, height), (20, 20))
        self.assertEqual(len(first._warp_cache), 1)
        self.assertEqual(second._warp_cache, {})


if __name__ == '__main__':
    unittest.main()
