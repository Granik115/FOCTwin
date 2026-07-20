import unittest

from foctwin.serial_device import SerialLineFramer


class SerialLineFramerTests(unittest.TestCase):
    def test_fragmented_usb_reads_are_joined_before_dispatch(self):
        framer = SerialLineFramer()

        self.assertEqual(framer.feed(b"1\t2"), [])
        self.assertEqual(framer.feed(b"\t3\r"), [])
        self.assertEqual(framer.feed(b"\nStatus: 1\npar"), ["1\t2\t3", "Status: 1"])
        self.assertEqual(framer.feed(b"tial\n"), ["partial"])

    def test_multiple_records_in_one_read_are_preserved(self):
        framer = SerialLineFramer()

        self.assertEqual(framer.feed(b"first\nsecond\r\n"), ["first", "second"])

    def test_unterminated_overflow_is_discarded(self):
        framer = SerialLineFramer(max_pending_bytes=4)

        self.assertEqual(framer.feed(b"12345"), [])
        self.assertEqual(framer.overflow_count, 1)
        self.assertEqual(framer.feed(b"ok\n"), ["ok"])
