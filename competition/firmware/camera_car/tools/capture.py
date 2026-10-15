"""Save firmware grayscale frames as lossless PGM files (pyserial for COM mode)."""
import argparse
from datetime import datetime
from pathlib import Path
import time
HEADER=b'\x00\xff\x01\x01'
WIDTH,HEIGHT=188,120
def read_frame(stream):
    window=bytearray()
    while bytes(window)!=HEADER:
        byte=stream.read(1)
        if not byte:
            raise TimeoutError('No complete image header received')
        window+=byte
        if len(window)>len(HEADER): del window[0]
    data=bytearray()
    while len(data)<WIDTH*HEIGHT:
        block=stream.read(WIDTH*HEIGHT-len(data))
        if not block:
            raise TimeoutError(f'Incomplete frame: {len(data)}/{WIDTH*HEIGHT} bytes')
        data+=block
    return bytes(data)
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True,help='e.g. COM5')
    parser.add_argument('--count',type=int,default=1)
    parser.add_argument('--label',choices=['normal','bright','dark'],default='normal')
    parser.add_argument('--out',type=Path,default=Path(__file__).resolve().parents[1]/'data')
    parser.add_argument('--binary',action='store_true')
    args=parser.parse_args()
    if args.count<1: parser.error('--count must be positive')
    import serial
    folder=args.out/args.label; folder.mkdir(parents=True,exist_ok=True)
    with serial.Serial(args.port,115200,timeout=10) as link:
        time.sleep(0.2)
        link.write(b's') # stop before image requests
        time.sleep(0.1)
        link.reset_input_buffer()
        for i in range(args.count):
            link.write(b'b' if args.binary else b'g')
            frame=read_frame(link)
            name=datetime.now().strftime('%Y%m%d-%H%M%S-%f')
            path=folder/f'{name}-{i+1:02}.pgm'
            path.write_bytes(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii')+frame)
            print(path)
if __name__=='__main__': main()
