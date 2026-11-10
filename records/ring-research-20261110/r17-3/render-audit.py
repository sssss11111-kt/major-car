"""Build offline per-frame review and contact sheets from verified audit data."""
from pathlib import Path
import io
import json
import re
import zipfile
from PIL import Image, ImageDraw

work = Path(__file__).resolve().parent
report = json.loads((work/'results.json').read_text(encoding='utf-8'))
audit = json.loads((work/'frame-audit.json').read_text(encoding='utf-8'))
old_page = (work/'comparison.html').read_text(encoding='utf-8')
display = json.loads(re.search(r'<script id="data" type="application/json">(.*?)</script>',
                               old_page, re.S).group(1))
for frame, facts in zip(display['frames'], audit['frames']):
    frame['audit'] = facts

with zipfile.ZipFile(report['capture_zip']) as archive:
    metadata = [json.loads(line) for line in archive.read('manual-02/frames.jsonl').decode().splitlines()]

    def overlay(index, key='r173'):
        data = archive.read('manual-02/'+metadata[index-1]['file'].replace('\\', '/'))
        image = Image.open(io.BytesIO(data)).convert('RGB')
        draw = ImageDraw.Draw(image)
        result = report['frames'][index-1][key]
        if result['path'] and result['valid']:
            draw.line([tuple(point[:2]) for point in result['path']], fill='#ff8900')
        elif key == 'r173' and result['local_active']:
            facts = audit['frames'][index-1]
            lines = facts['local_rows']
            for row in range(18, 106):
                center = lines[120+row]
                if center:
                    draw.point((center,row),fill='#ff8900' if result['valid'] else '#4593cf')
        elif result['valid']:
            for row in range(18,106):
                center = result['lines'][120+row]
                if center:
                    draw.point((center,row), fill='#ff8900')
        return image

    included = [frame for frame in audit['frames'] if frame['included']]
    for start in range(0, len(included), 35):
        sheet = Image.new('RGB', (1880,1960), '#f0f2f4')
        draw = ImageDraw.Draw(sheet)
        for position, facts in enumerate(included[start:start+35]):
            x, y = position%5*376, position//5*280
            sheet.paste(overlay(facts['frame']).resize((376,240)), (x,y))
            label = f"F{facts['frame']} valid={int(facts['valid'])} local={int(facts['local'])} near={facts['near_rows'] if facts['local'] else '-'}"
            color = '#b52121' if not facts['valid'] else '#222222'
            draw.text((x+4,y+242), label, fill=color)
            draw.text((x+4,y+258), f"e={facts['error']:.0f} pre={facts['preview']:.0f} servo={facts['command']:.2f}", fill=color)
        sheet.save(work/f'audit-atlas-{start//35+1:02}.png')

    focus = [59,60,61,62,63,139,140,141,150,155,156,157,158,218,219,220,221,222,223,224,225]
    sheet = Image.new('RGB',(1880,6*280),'white')
    draw = ImageDraw.Draw(sheet)
    for position, index in enumerate(focus):
        facts = audit['frames'][index-1]
        x,y = position%5*376, position//5*280
        sheet.paste(overlay(index).resize((376,240)),(x,y))
        draw.text((x+4,y+242),f"F{index} valid={int(facts['valid'])} near={facts['near_rows']} bottom={facts['near_bottom']}",fill='#222')
        draw.text((x+4,y+258),f"e={facts['error']:.0f} pre={facts['preview']:.0f} servo={facts['command']:.2f}",fill='#222')
    sheet.save(work/'疑点帧.png')

template = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>R17-3 逐帧识别检查</title>
<style>body{font:16px/1.6 system-ui,"Microsoft YaHei";margin:24px;background:#f2f4f7;color:#172438}
main{max-width:1550px;margin:auto}.panels{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}
section,.info{background:white;border:1px solid #d7dde5;padding:12px}canvas{width:100%;image-rendering:pixelated}
button,input{font:inherit}button{margin:4px;padding:6px 12px}#seek{width:80%}.bad{color:#ad2222}
table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:7px;border-bottom:1px solid #dde3ea}
@media(max-width:1000px){.panels{grid-template-columns:repeat(2,1fr)}}@media(max-width:600px){.panels{grid-template-columns:1fr}}
</style><main><h1>R17-3 逐帧识别检查</h1>
<p>312 帧原图均核对 CRC 和 SHA-256。沿用此前范围，纳入 1～85、111～264 共 239 帧，第 111 帧复位。算法判定有效 232 帧，拒绝 7 帧。</p>
<p>记录没有同步编码器。软件阶段和模拟舵机反映离线复算，局部接管单独显示。第 156 帧首次失效，之后用于诊断。</p>
<div><button id="prev">上一帧</button><button id="next">下一帧</button><button id="play">播放</button>
<input id="number" type="number" min="1" max="312" value="1"> 帧 <button id="nextBad">下个拒绝帧</button><button id="nextFlag">下个疑点帧</button></div>
<input id="seek" type="range" min="1" max="312" value="1"><strong id="position"></strong>
<div id="jumps"></div><div class="panels">
<section><h3>原始灰度</h3><canvas id="raw" width="188" height="120"></canvas></section>
<section><h3>当前阈值二值图</h3><canvas id="binary" width="188" height="120"></canvas></section>
<section><h3>R17 输出</h3><canvas id="old" width="188" height="120"></canvas></section>
<section><h3>R17-3 输出</h3><canvas id="new" width="188" height="120"></canvas></section></div>
<p>橙色为有效路线或逐行中心。拒绝帧用蓝色显示残留的原始观测点。逐行点之间的空缺保留显示。</p>
<div class="info"><p id="finding"></p><table id="facts"></table></div>
<p>“有效”表示算法通过了当前检查。路线正确性和真实车辆转弯效果需要结合现场记录判断。</p>
</main><script id="auditData" type="application/json">__DATA__</script><script>
const data=JSON.parse(document.getElementById('auditData').textContent),el=id=>document.getElementById(id);
let current=1,timer=null,token=0;window.auditReady=false;
function drawOutput(ctx,m,a,isNew){
 if(m.path.length&&m.valid){ctx.strokeStyle='#ff8900';ctx.beginPath();m.path.forEach((p,i)=>i?ctx.lineTo(p[0]+.5,p[1]+.5):ctx.moveTo(p[0]+.5,p[1]+.5));ctx.stroke();return}
 const lines=isNew&&a.local?a.local_rows:m.lines;if(!m.valid&&!(isNew&&a.local))return;
 ctx.fillStyle=m.valid?'#ff8900':'#4593cf';for(let y=18;y<=105;y++){const x=lines[120+y];if(x)ctx.fillRect(x,y,1,1)}
}
async function render(n){current=Math.max(1,Math.min(312,n));const id=++token,f=data.frames[current-1],a=f.audit;
 el('number').value=current;el('seek').value=current;el('position').textContent=`第 ${current}/312 帧 · ${a.included?'纳入分析':'此前排除段'}`;
 await new Promise((resolve,reject)=>{const image=new Image();image.onload=()=>{if(id===token){for(const name of ['raw','binary','old','new']){const ctx=el(name).getContext('2d');ctx.drawImage(image,0,0);if(name==='binary'){const p=ctx.getImageData(0,0,188,120);for(let y=0;y<120;y++)for(let x=0;x<188;x++){const k=(y*188+x)*4,v=p.data[k]>a.thresholds[y]?255:0;p.data[k]=p.data[k+1]=p.data[k+2]=v}ctx.putImageData(p,0,0)}else if(name==='old'||name==='new')drawOutput(ctx,f[name],a,name==='new')}}resolve()};image.onerror=reject;image.src=f.gray});
 if(id!==token)return;el('finding').textContent=a.flags.join('。')||(a.local?'当前局部环道观测通过检查':'入口或普通循迹阶段');el('finding').className=a.valid?'':'bad';
 const entries=[['算法判定',a.valid?'有效':'拒绝'],['局部接管',a.local?'已接管':'尚未接管'],['软件阶段 / 状态 / 失败码',`${a.phase} / ${a.status} / ${a.failure}`],['局部近端观测 / 单边观测 / 最近观测行',`${a.near_rows} / ${a.single_near} / ${a.near_bottom}`],['局部原始误差 / 控制近端误差',`${a.raw_local_error===null?'未启用':a.raw_local_error.toFixed(2)} / ${a.error}`],['预瞄误差 / 预瞄有效',`${a.preview} / ${a.preview_valid}`],['模拟舵机 / 命令限幅 / 变化速率限制',`${a.command.toFixed(3)} / ${a.limited} / ${a.slew_limited}`],['岛体下端 / 暂存年龄 / 接管确认',`${a.island_bottom} / ${a.island_hold_age} / ${a.confirm}`]];
 el('facts').replaceChildren(...entries.map(([k,v])=>{const tr=document.createElement('tr');for(const x of [k,v]){const td=document.createElement('td');td.textContent=x;tr.append(td)}return tr}));window.auditReady=true;window.renderedFrame=current;
}
el('prev').onclick=()=>render(current-1);el('next').onclick=()=>render(current+1);el('number').onchange=e=>render(+e.target.value);el('seek').oninput=e=>render(+e.target.value);
el('nextBad').onclick=()=>render(data.frames.find(f=>f.index>current&&f.audit.included&&!f.audit.valid)?.index||156);
el('nextFlag').onclick=()=>render(data.frames.find(f=>f.index>current&&f.audit.included&&f.audit.flags.some(x=>!x.includes('单边')))?.index||61);
el('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;el('play').textContent='播放'}else{el('play').textContent='暂停';timer=setInterval(()=>render(current===312?1:current+1),400)}};
for(const n of [58,61,62,139,141,150,155,156,157,158,187,218,219,223,224]){const b=document.createElement('button');b.textContent=n;b.onclick=()=>render(n);el('jumps').append(b)}
window.auditSeek=render;render(1);
</script></html>'''
payload=json.dumps(display,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
(work/'逐帧回放.html').write_text(template.replace('__DATA__',payload),encoding='utf-8')
print('Rendered 239 scoped frames in 7 contact sheets and 312-frame offline viewer')
