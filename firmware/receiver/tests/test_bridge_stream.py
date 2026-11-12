import ctypes as c
from pathlib import Path
import random
import subprocess
import tempfile
import unittest


class BridgeStream(c.Structure):
    _fields_ = [('bytes', c.c_uint8 * 512), ('length', c.c_size_t), ('sent', c.c_size_t)]


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        source = Path(__file__).resolve().parents[1] / 'main'
        library = Path(cls.directory.name) / 'bridge.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-fsanitize=undefined', '-fno-sanitize-recover=all', '-fPIC', '-shared', str(source / 'bridge_stream.c'), '-o', str(library)], check=True)
        cls.library = c.CDLL(str(library))
        cls.callback = c.CFUNCTYPE(c.c_int, c.c_void_p, c.c_size_t)
        cls.library.sz_bridge_pump.argtypes = [c.POINTER(BridgeStream), cls.callback, cls.callback]
        cls.library.sz_bridge_pump.restype = c.c_int

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_fragmented_bidirectional_binary_and_cli_streams(self):
        rng = random.Random(527)
        payloads = [bytes(range(256)) * 257 + b'\x00$M<\x00\x01\x01\xff#dump all\r\nsave\r\n', b'\x00$M>\x03\x01\x00\x01\x2e' + bytes(reversed(range(256))) * 130]
        streams = [BridgeStream(), BridgeStream()]
        read_offsets, output = [0, 0], [bytearray(), bytearray()]
        readers, writers = [], []
        for index in range(2):
            def read(destination, size, index=index):
                count = min(size, rng.randrange(0, 150), len(payloads[index]) - read_offsets[index])
                c.memmove(destination, payloads[index][read_offsets[index]:read_offsets[index] + count], count)
                read_offsets[index] += count
                return count
            def write(source, size, index=index):
                count = rng.randrange(size + 1)
                output[index].extend(c.string_at(source, count))
                return count
            readers.append(self.callback(read))
            writers.append(self.callback(write))
        for _ in range(20000):
            for index in range(2):
                self.assertGreaterEqual(self.library.sz_bridge_pump(c.byref(streams[index]), readers[index], writers[index]), 0)
            if all(bytes(output[index]) == payloads[index] for index in range(2)):
                break
        else:
            self.fail('Fragmented transfer did not complete')
        self.assertEqual([bytes(value) for value in output], payloads)

    def test_backpressure_retains_pending_bytes_without_reading_ahead(self):
        stream = BridgeStream()
        reads = []
        @self.callback
        def read(destination, size):
            reads.append(1)
            c.memmove(destination, b'hello', 5)
            return 5
        @self.callback
        def blocked(source, size):
            return 0
        for _ in range(100):
            self.assertEqual(self.library.sz_bridge_pump(c.byref(stream), read, blocked), 0)
        self.assertEqual(len(reads), 1)
        self.assertEqual(bytes(stream.bytes[:stream.length]), b'hello')
        self.assertEqual(stream.sent, 0)

    def test_driver_error_propagates_without_transmitting_garbage(self):
        stream = BridgeStream()
        @self.callback
        def error(destination, size):
            return -1
        @self.callback
        def unused(source, size):
            self.fail('No write should follow a failed read')
        self.assertEqual(self.library.sz_bridge_pump(c.byref(stream), error, unused), -1)

    def test_impossible_driver_counts_are_rejected(self):
        stream = BridgeStream()
        @self.callback
        def too_many(destination, size):
            return size + 1
        @self.callback
        def unused(source, size):
            return 0
        self.assertEqual(self.library.sz_bridge_pump(c.byref(stream), too_many, unused), -1)


if __name__ == '__main__':
    unittest.main()
