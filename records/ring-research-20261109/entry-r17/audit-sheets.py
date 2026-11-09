from pathlib import Path
import json,zipfile,io
from PIL import Image,ImageDraw
p=Path('records/ring-research-20261109/entry-r17');r=json.loads((p/'results.json').read_text(encoding='utf-8'))
print('invalid',[(f['index'],f['r17']['failure'],f['r17']['phase']) for f in r['frames'] if not f['r17']['excluded'] and not f['r17']['valid']])
with zipfile.ZipFile(r['capture_zip']) as z:
 fs=[f for f in r['frames'] if not f['r17']['excluded']]
 ms=[json.loads(l) for l in z.read('manual-02/frames.jsonl').decode().splitlines()]
 for start in range(0,len(fs),35):
  sheet=Image.new('RGB',(5*376,7*264),'#ececec');d=ImageDraw.Draw(sheet)
  for j,f in enumerate(fs[start:start+35]):
   im=Image.open(io.BytesIO(z.read('manual-02/'+ms[f['index']-1]['file'].replace('\\','/')))).convert('RGB');draw=ImageDraw.Draw(im);m=f['r17']
   pts=[tuple(v[:2]) for v in m['path']]
   if pts:draw.line(pts, fill='orange',width=1)
   elif m['valid']:
    for y in range(18,106):
     x=m['lines'][120+y]
     if x:draw.point((x,y),fill='orange')
   x=(j%5)*376;y=(j//5)*264
   sheet.paste(im.resize((376,240)),(x,y));d.text((x+4,y+242),f"{f['index']} valid{m['valid']} p{m['phase']} f{m['failure']} n{len(pts)} cmd{m['command']:.1f}",fill='black')
  sheet.save(p/f'audit-{start//35+1}.png')
