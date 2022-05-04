import unittest

from gz_mujoco.gz_mujoco import execute


class TestExample(unittest.TestCase):

    def test_stub(self):
        self.assertEqual(execute(), None)


if __name__ == '__main__':
    unittest.main()
