"""Trusted local pickle input -> JSONL; runs outside the control interpreter."""
import argparse
import json
from pathlib import Path
import pickle
import socket
import struct
import sys


def read_exact(connection, length):
    result = bytearray()
    while len(result) < length:
        block = connection.recv(length-len(result))
        if not block:
            raise EOFError('Recording stream ended without its close marker')
        result.extend(block)
    return bytes(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fd', type=int, required=True)
    parser.add_argument('--path', type=Path, required=True)
    args = parser.parse_args()
    count = 0
    try:
        with socket.socket(fileno=args.fd) as connection, args.path.open('x', buffering=1) as output:
            print(json.dumps({'ready':True}), flush=True)
            while True:
                length = struct.unpack('!I', read_exact(connection, 4))[0]
                if length == 0:
                    break
                if length > 8*1024*1024:
                    raise ValueError('Oversized recording item')
                event = pickle.loads(read_exact(connection, length))
                output.write(json.dumps(event, allow_nan=False)+'\n')
                count += 1
            output.flush()
        print(json.dumps({'closed':True, 'count':count}), flush=True)
    except Exception as error:
        print(json.dumps({'error':f'{type(error).__name__}: {error}', 'count':count}), flush=True)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
