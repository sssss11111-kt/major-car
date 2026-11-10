"""Standalone, original-numbered 120..247 playback and diagnostic overlays."""
from pathlib import Path
import base64
import json
from io import BytesIO
from PIL import Image, ImageDraw, ImageFont

work = Path(__file__).resolve().parent
result = json.loads((work/'results.json').read_text(encoding='utf-8'))
frames = result['frames']
assert [f['index'] for f in frames] == list(range(120,248))
font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 18)
sheet = Image.new('RGB', (188*3*3, (120*3+65)*6), 'white')
draw = ImageDraw.Draw(sheet)
for row, index in enumerate([129,138,143,203,219,224]):
    frame = frames[index-120]
    source = Image.open(BytesIO(base64.b64decode(frame['image'].split(',')[1]))).convert('RGB')
    for column, version in enumerate(['gray','r17','r174']):
        overlay=source.copy()
        pen=ImageDraw.Draw(overlay)
        if version!='gray':
            current=frame['r17'] if version=='r17' else frame
            if current['path']:
                pen.line([(p[0],p[1]) for p in current['path']],fill=(240,125,0),width=1)
            else:
                lines=current['lines']
                for y in range(18,106):
                    x=lines[120+y]
                    if x: pen.point((x,y),fill=(240,125,0) if current['valid'] else (20,125,230))
            if version=='r174':
                for x in range(0,188,4): pen.point((x,105),fill=(40,140,240))
        sheet.paste(overlay.resize((564,360)),(column*564,row*425+55))
        label=f'{index}  '+dict(gray='原图',r17='R17',r174='R17-4')[version]
        if version=='r174':label+=f"  阶段{frame['phase']}  {'普通弯道' if frame['local'] else '入口规划'}"
        draw.text((column*564+10,row*425+10),label,font=font,fill=(25,35,50))
sheet.save(work/'关键帧对比.png')

payload=[]
for frame, scenario in zip(frames,result['exit_scenario']):
    payload.append({**frame,'exit':{k:scenario[k] for k in ['phase','valid','local','error','preview','command','path','lines']}})

html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>R17-4 环岛逐帧回放 120～247</title>
<style>body{font-family:system-ui,"Microsoft Yahei",sans-serif;background:#f1f4f8;color:#1c2a3b;margin:24px auto;padding:0 24px;max-width:1700px}h1{font-size:28px}p{line-height:1.7}.controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:15px 0}button,input{font:inherit;padding:9px;border:1px solid #a8b5c5;border-radius:5px;background:white}button{cursor:pointer}input[type=range]{width:100%;padding:0}.panels{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}section{padding:12px;background:white;border:1px solid #d5dfeb;border-radius:8px}canvas{width:100%;height:auto;image-rendering:pixelated}h2{font-size:17px}table{background:white;border-collapse:collapse;width:100%;margin:20px 0}td{padding:10px;border-bottom:1px solid #dde4ec}td:first-child{width:40%}.bad{color:#b32232}.note{font-size:14px;color:#506176}#frame{font-weight:bold}.banner{background:#e2ecfc;padding:12px;border-radius:6px}@media(max-width:1000px){.panels{grid-template-columns:repeat(2,1fr)}}
</style><h1>R17-4 环岛逐帧回放</h1>
<p>仅保留原编号 <b>120～247</b>，共 <b>128</b> 帧。原始图像 CRC 与 SHA-256 已核对。121 帧有效，156、157、219～223 帧拒绝。</p>
<p class="banner">采集没有同步轮速。图像复算的阶段及舵机命令属于软件输出。右侧 EXIT 场景在第 203 帧注入 EXIT / 270°，用于检查已确认出环后的选路。</p>
<div class="controls"><button id="prev">上一帧</button><button id="next">下一帧</button><button id="play">播放</button><label>原帧号 <input id="jump" type="number" min="120" max="247" value="120"></label><button id="reject">下个拒绝帧</button><span id="frame"></span></div>
<label for="slider">帧进度</label><input id="slider" type="range" min="120" max="247" value="120">
<div class="controls" id="keys"></div><div class="panels">
<section><h2>原始灰度</h2><canvas id="gray" width="188" height="120"></canvas></section>
<section><h2>R17 输出</h2><canvas id="old" width="188" height="120"></canvas></section>
<section><h2>R17-4 图像连续复算</h2><canvas id="current" width="188" height="120"></canvas></section>
<section><h2>R17-4 注入出环阶段</h2><canvas id="exit" width="188" height="120"></canvas></section></div>
<p class="note">橙色为当前路线或逐行中心，蓝色点为拒绝帧残留观测。ALIGN 阶段的曲线为提前生成的规划候选，实际控制仍采用入口近端误差。蓝色虚线为第 105 行控制区域下沿，第 106～119 行仍是可见道路。普通弯道显示实测逐行点，缺失行保留空白。</p>
<table id="details"></table><p class="note">第 111～119 帧仅用于恢复此前连续状态，页面没有嵌入这几帧的图像。按左右方向键换帧。首个拒绝帧之后的数据用于诊断。</p>
<script id="data" type="application/json">PAYLOAD</script><script>
const frames=JSON.parse(document.getElementById('data').textContent);
let index=0,timer=null,renderSerial=0;
const phases={0:'IDLE',1:'ENTER',2:'ORBIT',3:'EXIT',4:'COOLDOWN',5:'ALIGN'};
function paint(id,image,state){const c=document.getElementById(id),p=c.getContext('2d');p.drawImage(image,0,0);if(!state)return;
 p.fillStyle=state.valid?'#f08000':'#1485e5';p.strokeStyle=p.fillStyle;p.lineWidth=0.8;
 if(state.path.length){p.beginPath();state.path.forEach((v,i)=>i?p.lineTo(v[0],v[1]):p.moveTo(v[0],v[1]));p.stroke();}
 else for(let y=18;y<=105;y++){let x=state.lines[120+y];if(x)p.fillRect(x,y,1,1);}
 if(id!=='old'){p.strokeStyle='#288cf0';p.setLineDash([2,3]);p.beginPath();p.moveTo(0,105);p.lineTo(188,105);p.stroke();p.setLineDash([]);}}
function show(n){index=Math.max(0,Math.min(frames.length-1,n));const f=frames[index],serial=++renderSerial;
 document.getElementById('slider').value=f.index;document.getElementById('jump').value=f.index;
 document.getElementById('frame').textContent=`第 ${f.index} 帧 / 120～247 · ${index+1}/128`;
 const rows=[['算法有效',f.valid?'有效':'拒绝'],['阶段',phases[f.phase]],['环内普通弯道',f.local?'已接管':'等待入口离开视野'],['近端误差 / 预瞄误差',f.error+' / '+f.preview],['模拟舵机命令',f.command.toFixed(2)],['当前规划点数 / 最大段夹角',f.path.length+' / '+f.angle.toFixed(2)+'°'],['局部近端观测行数',f.debug[0]],['出环注入场景阶段',phases[f.exit.phase]],['说明',f.phase===5?'提前规划候选，入口近端误差控制':f.local?'普通弯道当前单边/双边循迹':'当前前向道路曲线']];
 document.getElementById('details').innerHTML=rows.map(r=>`<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
 const image=new Image();image.onload=()=>{if(serial!==renderSerial)return;paint('gray',image);paint('old',image,f.r17);paint('current',image,f);paint('exit',image,f.exit);document.body.dataset.loadedFrame=f.index;};image.src=f.image;}
document.getElementById('prev').onclick=()=>show(index-1);document.getElementById('next').onclick=()=>show(index+1);
document.getElementById('jump').onchange=e=>show(Number(e.target.value)-120);document.getElementById('slider').oninput=e=>show(Number(e.target.value)-120);
document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;document.getElementById('play').textContent='播放';}else{document.getElementById('play').textContent='暂停';timer=setInterval(()=>{if(index===127){document.getElementById('play').click();return;}show(index+1);},180);}};
document.getElementById('reject').onclick=()=>{const next=frames.findIndex((f,i)=>i>index&&!f.valid);if(next>=0)show(next);};
for(const n of [120,127,129,138,143,156,157,180,203,219,223,224,247]){const b=document.createElement('button');b.textContent=n;b.onclick=()=>show(n-120);document.getElementById('keys').append(b);}
document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;if(e.key==='ArrowRight')show(index+1);if(e.key==='ArrowLeft')show(index-1);});show(0);
</script></html>'''
html=html.replace('PAYLOAD',json.dumps(payload,ensure_ascii=False).replace('</','<\\/'))
(work/'逐帧回放.html').write_text(html,encoding='utf-8')
print('Rendered 128-frame playback, range 120..247')
