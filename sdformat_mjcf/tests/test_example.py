import unittest

from sdformat_mjcf.sdformat_mjcf import execute


class TestExample(unittest.TestCase):

    def test_stub(self):
        self.assertEqual(execute(), None)


if __name__ == '__main__':
    unittest.main()
