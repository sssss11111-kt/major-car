"""Render generated fixtures and actual C results; Python standard library only."""
from pathlib import Path
from base64 import b64encode
import csv
import html
import json
import struct
import zlib

def png_rgb(width,height,pixels):
    def chunk(kind,data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    scanlines=b''.join(b'\0'+pixels[y*width*3:(y+1)*width*3] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',width,height,8,2,0,0,0))
            +chunk(b'IDAT',zlib.compress(scanlines))+chunk(b'IEND',b''))

def render(folder):
    records=[json.loads(line) for line in (folder/'results.jsonl').read_text().splitlines()]
    cards=[]
    montage=bytearray(188*4*120*6*3)
    for index,item in enumerate(records):
        name=item['name']
        raw=(folder/f'{name}-gray.pgm').read_bytes()
        header=b'P5\n188 120\n255\n'
        if not raw.startswith(header) or len(raw)!=len(header)+188*120:
            raise ValueError(f'Invalid fixture PGM: {name}')
        rgb=bytearray(component for value in raw[len(header):] for component in (value,value,value))
        for y,(left,right,center,truth_left,truth_right) in enumerate(item['rows']):
            for x,color in [(left,(0,160,255)),(right,(0,160,255)),(center,(255,0,0))]:
                if 0<=x<188:
                    start=(y*188+x)*3
                    rgb[start:start+3]=bytes(color)
        (folder/f'{name}.png').write_bytes(png_rgb(188,120,rgb))
        if index<24:
            for y in range(120):
                source=y*188*3
                target=((index//4*120+y)*752+index%4*188)*3
                montage[target:target+188*3]=rgb[source:source+188*3]
        classification=item['expectation']
        status='FAIL' if item['failures'] else ('OBSERVE' if classification=='characterize' else 'PASS')
        cards.append(f'''<article><h2>{html.escape(name)}</h2><img src="{html.escape(name)}.png" width="376" height="240">
<p>{status} · {classification} · valid={item['valid']} · paired={item['paired']}/{item['scanned']}</p>
<p>threshold={item['threshold']} · error={item['error']:.2f} · max row center error={item['max_center_error']:.2f}</p></article>''')
    (folder/'overview.png').write_bytes(png_rgb(752,720,montage))
    document='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>图像算法测试报告</title>
<style>body{font:16px system-ui;background:#f5f6f8;color:#182230;margin:24px}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(390px,1fr));gap:16px}article{background:white;border-radius:12px;padding:16px}h2{font-size:17px}img{image-rendering:pixelated;max-width:100%;height:auto}p{line-height:1.5}</style>
<h1>合成图像测试报告</h1><p>蓝色：识别到的边线；红色：中线。24 个代表场景，另有 128 个固定种子的噪声变体。
OBSERVE 仅记录算法表现，不计为准确识别通过。合成图像不能替代真实镜头、光照和实车验收。</p><main>'''+''.join(cards)+'</main></html>'
    (folder/'report.html').write_text(document,encoding='utf-8')
    print(f'Visual report: {folder / "report.html"}')

def render_special(folder):
    cards = []
    with (folder / 'results.tsv').open(encoding='utf-8', newline='') as report:
        records = list(csv.DictReader(report, delimiter='\t'))
    for record in records:
        scene = record['scene']
        images = []
        for suffix, channels, label in [('gray.pgm', 1, '原始灰度'),
                                        ('binary.pgm', 1, '二值图'),
                                        ('overlay.ppm', 3, '扫描叠加')]:
            data = (folder / f'{scene}-{suffix}').read_bytes()
            header = b'P5\n188 120\n255\n' if channels == 1 else b'P6\n188 120\n255\n'
            if not data.startswith(header) or len(data) != len(header) + 188 * 120 * channels:
                raise ValueError(f'Invalid special-track image: {scene}-{suffix}')
            pixels = data[len(header):]
            if channels == 1:
                pixels = bytes(c for v in pixels for c in (v, v, v))
            png = b64encode(png_rgb(188, 120, pixels)).decode('ascii')
            images.append(f'<figure><img src="data:image/png;base64,{png}" width="188" height="120" alt="{label}"><figcaption>{label}</figcaption></figure>')
        stats = ' · '.join(f'{html.escape(k)}={html.escape(v)}' for k, v in record.items()
                           if k not in ('scene', 'verdict'))
        cards.append(f'<article><h2>{html.escape(scene)} · {html.escape(record["verdict"])}</h2>'
                     f'<p>{stats}</p><section>{"".join(images)}</section></article>')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>特殊赛道扫描报告</title>
<style>
body{font:16px system-ui,sans-serif;color:#122824;background:#eef4f1;margin:24px auto;padding:0 16px;max-width:1100px}
article{background:#fff;border:1px solid #c5d7d0;border-radius:9px;padding:12px 16px;margin:16px 0}
h2{font-size:17px;margin:0}p{line-height:1.6}section{display:flex;flex-wrap:wrap;gap:12px}
figure{margin:0}img{image-rendering:pixelated;display:block}figcaption{color:#3a5650}
</style>
<h1>特殊赛道扫描报告</h1>
<p>合成画面上的主机结果。绿色标出扫描区间，蓝色标出普通边界，红色标出普通中线。
类型编号依次为普通、十字岛、环岛、未知。状态编号依次为空闲、候选、十字岛、环岛、未知、冷却、故障。</p>
<p>PASS 表示预期识别或空闲行为通过，REJECT 表示通过普通或干扰场景拒识，
UNKNOWN 表示证据不足，OBSERVE 表示仅记录可见环岛缺少接近时序的表现。</p>
''' + ''.join(cards)
    (folder / 'report.html').write_text(page, encoding='utf-8')
    print(f'Special-track report: {folder / "report.html"}')

if __name__=='__main__':
    render(Path(__file__).resolve().parent/'out/vision')
    special = Path(__file__).resolve().parent/'out/special'
    if (special / 'results.tsv').exists():
        render_special(special)
