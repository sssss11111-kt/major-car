"""Regression for real CSI SDK/callback ownership and foreground seq rejection.

The harness extracts production C rather than copying its queue or callback logic.
Only register storage, IRQ scheduling and address width are host doubles. It proves
software address sequencing; injected DMA writes are not a hardware timing test.
Run in a compiler environment (Windows: vcvars64.bat, then python this_file.py).
"""
from pathlib import Path
import argparse
import os
import re
import shutil
import subprocess
import tempfile
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', help='C compiler path (default: discover cl/cc/gcc/clang)')
parser.add_argument('--cc-arg', action='append', default=[], help='Compiler prefix argument, e.g. cc for Zig')
args = parser.parse_args()
p = Path(__file__).resolve().parents[1]
def function(src, name):
    text = (p / src).read_text(encoding="utf-8", errors="replace")
    matches = re.finditer(r"^.*\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", text, re.M)
    match = next(matches, None)
    if match is None:
        raise RuntimeError(f"Definition not found: {src}: {name}")
    brace = text.index("{", match.start())
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end]

head=r'''
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
/* Address-width shim for running 32-bit embedded source on a 64-bit host. */
typedef unsigned long long uint32_t;
typedef unsigned long long uint32;
typedef unsigned char uint8_t;
typedef unsigned char uint8;
typedef int status_t;
typedef struct { uint32_t sr,cr1,fb[2]; } CSI_Type;
typedef struct csi_handle csi_handle_t;
struct csi_handle {uint8_t queueReadIdx,queueWriteIdx,activeBufferNum; uint32_t emptyBufferCnt,frameBufferAddr[5],frameBufferQueue[5]; void(*callback)(CSI_Type*,csi_handle_t*,status_t,void*);void*userData;};
#define CSI_MAX_ACTIVE_FRAME_NUM 2U
#define CSI_DRIVER_QUEUE_SIZE 4U
#define CSI_DRIVER_ACTUAL_QUEUE_SIZE 5U
#define CSI_REG_SR(b) ((b)->sr)
#define CSI_REG_CR1(b) ((b)->cr1)
#define CSI_REG_DMASA_FB1(b) ((b)->fb[0])
#define CSI_REG_DMASA_FB2(b) ((b)->fb[1])
#define CSI_ADDR_CPU_2_IP(a) (a)
#define CSI_ADDR_IP_2_CPU(a) (a)
#define CSI_SR_DMA_TSF_DONE_FB1_MASK 1U
#define CSI_SR_DMA_TSF_DONE_FB2_MASK 2U
#define CSI_CR1_FB1_DMA_DONE_INTEN_MASK 1U
#define CSI_CR1_FB2_DMA_DONE_INTEN_MASK 2U
#define kStatus_CSI_FrameDone 0
#define kStatus_CSI_NoFullBuffer 1
#define kStatus_Success 0
#define MT9V03X_W 188
static CSI_Type device;
#define CSI (&device)
static csi_handle_t csi_handle;
static uint8 mt9v03x_image1[120][188],mt9v03x_image2[120][188];
static uint8 (*mt9v03x_image)[188];
static uint8 mt9v03x_finish_flag;
static uint32_t capture_sequence,capture_ms,milliseconds,stop_epoch;
static uint32_t consumed_sequence,dropped_frames;
static uint8 gray[120*188],camera_ready=1;
static void observer(void) {capture_ms=milliseconds;capture_sequence++;}
static void (*mt9v03x_frame_observer)(void)=observer;
'''
sdk='libraries/sdk/drives/fsl_csi.c'
names=['CSI_TransferGetQueueDelta','CSI_TransferIncreaseQueueIdx','CSI_TransferGetEmptyBufferCount','CSI_TransferGetEmptyBuffer','CSI_TransferPutEmptyBuffer','CSI_GetRxBufferAddr','CSI_SetRxBufferAddr','CSI_TransferSubmitEmptyBuffer','CSI_TransferGetFullBuffer']
parts=[head]+[function(sdk,n) for n in names]
parts += [function('libraries/zf_driver/zf_driver_csi.c','csi_add_empty_buffer'),function('libraries/zf_driver/zf_driver_csi.c','csi_get_full_buffer'),function('libraries/zf_device/zf_device_mt9v03x.c','mt9v03x_finished_callback'),function(sdk,'CSI_TransferHandleIRQ')]

# Execute the actual foreground acquisition branch, including its memcpy/seq check.
app = (p / 'demo/user/src/car_app.c').read_text(encoding='utf-8')
start = app.index('    if(camera_ready && mt9v03x_finish_flag)')
end = app.index('\n#endif', start)
capture_branch = app[start:end]
parts += [r"""
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c);exit(1);}}while(0)
static uint32_t lock(void) {return 0;}
static void unlock(uint32_t key) {(void)key;}
static unsigned next_done;
static int inject_completion;
static void complete_next(void) {
    CSI->sr=next_done?CSI_SR_DMA_TSF_DONE_FB2_MASK:CSI_SR_DMA_TSF_DONE_FB1_MASK;
    milliseconds+=20;
    CSI_TransferHandleIRQ(CSI,&csi_handle);
    next_done^=1u;
}
static void *copy_with_possible_completion(void *destination,const void *source,size_t size) {
    memcpy(destination,source,size/2);
    if(inject_completion) {
        inject_completion=0;
        complete_next();
        /* Simulate a possible write into the source now reused by the next DMA. */
        memset((uint8*)source+size/2,99,size-size/2);
    }
    memcpy((uint8*)destination+size/2,(const uint8*)source+size/2,size-size/2);
    return destination;
}
static int foreground_capture(void) {
    uint32_t key,captured=milliseconds,frame_epoch=stop_epoch;
    uint8_t new_frame=0;
#define memcpy copy_with_possible_completion
""", capture_branch, r"""
#undef memcpy
    (void)captured;(void)frame_epoch;
    return new_frame;
}
static void reset_capture(void) {
    memset(CSI,0,sizeof *CSI);memset(&csi_handle,0,sizeof csi_handle);
    capture_sequence=capture_ms=consumed_sequence=dropped_frames=milliseconds=0;
    mt9v03x_finish_flag=0;next_done=0;inject_completion=0;
    csi_handle.callback=mt9v03x_finished_callback;
    csi_add_empty_buffer(&csi_handle,mt9v03x_image1[0]);
    csi_add_empty_buffer(&csi_handle,mt9v03x_image2[0]);
"""]
# Use the actual SDK start's initial empty-buffer loading, including LIFO order.
start_fn=function(sdk,'CSI_TransferStart')
load_start=start_fn.index('    CSI_REG_DMASA_FB1(base)')
load_end=start_fn.index('\n',start_fn.index('    handle->activeBufferNum = CSI_MAX_ACTIVE_FRAME_NUM;',load_start))
parts += [start_fn[load_start:load_end].replace('base','CSI').replace('handle','(&csi_handle)'),r"""
}
int main(void) {
    reset_capture();
    for(unsigned frame=0;frame<256;frame++) {
        memset((void*)CSI->fb[next_done],20+(frame%60),sizeof gray);
        complete_next();
        CHECK((uint32_t)mt9v03x_image != CSI->fb[next_done]);
        CHECK(capture_sequence==frame+1);
        CHECK(foreground_capture()==1);
    }
    puts("PASS: 256 alternating DMA completions never publish next DMA destination");
    reset_capture();
    memset((void*)CSI->fb[next_done],21,sizeof gray);
    complete_next();
    CHECK(foreground_capture()==1);
    memset((void*)CSI->fb[next_done],22,sizeof gray);
    complete_next();
    inject_completion=1;
    CHECK(foreground_capture()==0);
    CHECK(dropped_frames==1);
    CHECK(gray[0]!=gray[sizeof gray-1]);
    CHECK(foreground_capture()==1);
    puts("PASS: completion during real foreground copy rejects injected mixed data, then recovers");
    uint32_t before=capture_sequence;
    CSI->sr=CSI_SR_DMA_TSF_DONE_FB1_MASK|CSI_SR_DMA_TSF_DONE_FB2_MASK;
    CSI_TransferHandleIRQ(CSI,&csi_handle);
    CHECK(capture_sequence==before);
    puts("PASS: simultaneous done flags do not falsely publish a frame");
    return 0;
}
"""]
with tempfile.TemporaryDirectory(prefix='csi-buffers-') as temp:
    out=Path(temp)
    source=out/'test_csi_buffers.c'
    source.write_text('\n\n'.join(parts),encoding='utf-8')
    exe=out/('test_csi_buffers.exe' if os.name=='nt' else 'test_csi_buffers')
    compiler=args.cc or next((c for c in ('cl','cc','gcc','clang') if shutil.which(c)),None)
    if compiler is None:
        raise SystemExit('C compiler required; initialize vcvars64.bat or pass --cc.')
    prefix=[compiler,*args.cc_arg]
    if Path(compiler).stem.lower()=='cl':
        command=[*prefix,'/nologo','/utf-8','/W3','/O1','/D_CRT_SECURE_NO_WARNINGS',str(source),'/Fe:'+str(exe)]
    else:
        command=[*prefix,'-std=c99','-O1',str(source),'-o',str(exe)]
    subprocess.run(command,cwd=out,check=True)
    subprocess.run([str(exe)],cwd=out,check=True)
