const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/逐帧回放.html','utf8');
const scripts=[...html.matchAll(/<script([^>]*)>([\s\S]*?)<\/script>/g)];
const payload=JSON.parse(scripts.find(m=>m[1].includes('auditData'))[2]);
assert.equal(payload.frames.length,312);
let draws=0;
function element(){return {value:1,textContent:'',className:'',children:[],append(x){this.children.push(x)},replaceChildren(...x){this.children=x},getContext(){return {drawImage(){draws++},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillRect(){},getImageData(){return {data:new Uint8ClampedArray(188*120*4)}},putImageData(){}}}};}
const elements={};for(const m of html.matchAll(/id="([^"]+)"/g))elements[m[1]]=element();
elements.auditData.textContent=JSON.stringify(payload);
class MockImage {set src(value){const b=Buffer.from(value.split(',')[1],'base64');assert.equal(b.readUInt32BE(16),188);assert.equal(b.readUInt32BE(20),120);queueMicrotask(()=>this.onload())}}
const context={document:{getElementById(id){assert(elements[id],id);return elements[id]},createElement:element},Image:MockImage,queueMicrotask,setInterval(){return 1},clearInterval(){},Uint8ClampedArray};context.window=context;
vm.createContext(context);
(async()=>{
 for(const script of scripts.filter(m=>!m[1].includes('application/json')))await vm.runInContext(script[2],context);
 for(let i=1;i<=312;i++){await context.auditSeek(i);assert.equal(context.renderedFrame,i);assert(context.auditReady);assert.equal(elements.facts.children.length,8)}
 await context.auditSeek(61);await elements.nextBad.onclick();assert.equal(context.renderedFrame,156);
 await elements.nextBad.onclick();assert.equal(context.renderedFrame,157);
 await elements.next.onclick();assert.equal(context.renderedFrame,158);
 await elements.prev.onclick();assert.equal(context.renderedFrame,157);
 elements.number.value=224;await elements.number.onchange({target:elements.number});assert.equal(context.renderedFrame,224);
 elements.play.onclick();assert.equal(elements.play.textContent,'暂停');elements.play.onclick();assert.equal(elements.play.textContent,'播放');
 console.log(JSON.stringify({checked_frames:312,image_dimensions:[188,120],canvas_draws:draws,table_rows:8,jump_and_navigation:true,method:'Node DOM/canvas API stubs; native browser rendering not exercised'}));
})().catch(e=>{console.error(e);process.exitCode=1});
