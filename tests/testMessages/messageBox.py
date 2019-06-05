import unittest
from src.messages.messageBox import MessageBox


class TestMessageBox(unittest.TestCase):
    def setUp(self):
        self.message_box = MessageBox()

        for i in range(10):
            self.message_box.put(i)
 
    def test_init(self):
        self.assertIsNotNone(self.message_box)

    def test_peek_first(self):
        # Should be 0 then 1 because we put range(10) in the setup
        self.assertEqual(self.message_box.peek_first(), 0)
        _ = self.message_box.get()
        self.assertEqual(self.message_box.peek_first(), 1)

    def test_peek_last(self):
        # Should be 9 and still 9 because we put range(10) in the setup
        self.assertEqual(self.message_box.peek_last(), 9)
        _ = self.message_box.get()
        self.assertEqual(self.message_box.peek_last(), 9)


if __name__ == "__main__":
    unittest.main()
