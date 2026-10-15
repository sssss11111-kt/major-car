import importlib.util
import io
from pathlib import Path
import unittest
module_path=Path(__file__).resolve().parents[1]/'tools/capture.py'
spec=importlib.util.spec_from_file_location('capture',module_path)
capture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
class CaptureTests(unittest.TestCase):
    def test_ascii_status_before_image_and_embedded_header(self):
        payload=bytes([30,210])*11280
        payload=payload[:20]+b'\x00\xff\x01\x01'+payload[24:]
        stream=io.BytesIO(b'CAMERA OK\r\n\x00\xff\x01\x01'+payload+b'next')
        self.assertEqual(capture.read_frame(stream),payload)
        self.assertEqual(stream.read(),b'next')
    def test_truncated_payload_is_not_saved_as_complete(self):
        with self.assertRaises(TimeoutError):
            capture.read_frame(io.BytesIO(b'\x00\xff\x01\x01\x04'))
if __name__=='__main__': unittest.main()
