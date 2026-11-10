"""R17-3 visual-only post-entry handoff through the production replay API."""
from pathlib import Path
import ctypes as C
import os
import subprocess

verification = Path(__file__).resolve().parent
project = verification.parent
library = verification / 'r17-3-replay.dll'
subprocess.run([
    os.environ['ZIG'], 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror',
    '-I', str(verification / 'vision-stubs'), '-I', str(project / 'demo/user/inc'),
    str(project / 'demo/user/src/img_process.c'),
    str(project / 'demo/user/src/steering_control.c'),
    str(verification / 'vision-replay-wrapper.c'), '-o', str(library)
], check=True)
native = C.CDLL(str(library))
native.replay_ring_scenario.argtypes = [C.c_int, C.c_float]
entrance = bytearray((verification / 'fixtures/ring-entry-gray.bin').read_bytes())
checks = 0


def check(condition, message):
    global checks
    assert condition, message
    checks += 1


def road(shift=0, bend=0, top=18):
    raw = bytearray([20] * 22560)
    for row in range(top, 106):
        half = int(71 + 32 * (row - 90) / 119 + 0.5)
        center = int(94 + shift + bend * ((105 - row) / 87) ** 1.4)
        for column in range(max(0, center-half), min(188, center+half)):
            raw[row*188+column] = 220
    return raw


def step(raw, tick):
    result = (C.c_float * 5)()
    native.replay_ring_step((C.c_uint8 * 22560).from_buffer_copy(raw), tick, result)
    return list(result)


def prime():
    native.replay_ring_reset()
    native.replay_ring_scenario(1, 0)
    for index in range(5):
        step(entrance, 2*index)
        check(not native.replay_ring_local_active(), 'visible entrance retains R17 planner')


def handoff(raw):
    prime()
    for index in range(3):
        result = step(raw, 10+2*index)
        check(native.replay_ring_local_active() == (index == 2), 'three new frames confirm disappearance')
    return result


# With no preceding entrance observation, ordinary pixels and encoder phase
# alone cannot authorize handoff. ALIGN/IDLE never activate the ring tracker.
for phase in [0, 1, 2, 5]:
    native.replay_ring_reset()
    native.replay_ring_scenario(phase, 60)
    for index in range(8):
        step(road(), 2*index)
        check(not native.replay_ring_local_active(), 'no visual entrance history, no handoff')

prime()
step(road(), 10)
step(road(), 10)
check(not native.replay_ring_local_active(), 'same timestamp cannot count a frame twice')
step(entrance, 12)
for tick in [14, 16]:
    step(road(), tick)
    check(not native.replay_ring_local_active(), 'visible entrance resets confirmation')
step(road(), 18)
check(native.replay_ring_local_active(), 'fresh consecutive absence confirms handoff')

prime()
native.replay_ring_scenario(2, 60)
for tick in range(10, 20, 2):
    step(entrance, tick)
    check(not native.replay_ring_local_active(), 'ORBIT phase alone cannot hide a visible entrance')
prime()
step(road(), 10)
step(road(), 12)
step(bytearray([20]*22560), 14)
for tick in [16, 18]:
    step(road(), tick)
    check(not native.replay_ring_local_active(), 'lost banks reset disappearance confirmation')
step(road(), 20)
check(native.replay_ring_local_active(), 'three valid observations required after loss')

# Ordinary bend equivalence after the entrance disappears, without changing yaw
# gates, gain, preview fusion or servo bounds.
for shift in [-12, 0, 12]:
    for bend in [-30, 0, 30]:
        raw = road(shift, bend)
        handoff(raw)
        for index in range(8):
            local = step(raw, 20+2*index)
        local_error = local[2]
        native.replay_reset()
        native.replay_guide(0)
        native.replay_entry_mode(0)
        out = (C.c_int * 4)()
        for index in range(8):
            native.replay_frame((C.c_uint8 * 22560).from_buffer_copy(raw), out)
        check(out[0] and local[3], 'ordinary and ring local detect bend')
        check(local_error == out[1], 'same weighted near-row error')
        check(69 <= local[0] <= 116, 'original servo bounds')

for shift in [-35, 35]:
    result = handoff(road(shift))
    for tick in range(20, 34, 2):
        result = step(road(shift), tick)
    check(result[3] and native.replay_entry_status() == 2, 'single bank retains reduced-speed status')
    check(result[2]*shift > 0, 'single bank retains measured direction')

handoff(road())
for tick in range(20, 34, 2):
    result = step(road(bend=20, top=80), tick)
check(result[3] and native.replay_entry_status() == 6, 'missing far road does not invalidate near bend')
check(native.replay_entry_path((C.c_uint8 * 1120)()) == 0, 'local control does not require entrance cubic')
for value in [20, 220]:
    handoff(road())
    result = step(bytearray([value]*22560), 20)
    check(not result[3], 'unbounded dark/white image rejects control')
    check(native.replay_ring_local_active(), 'loss retains ring road identity')

# The original EXIT and reset boundaries clear the local tracker.
handoff(road())
native.replay_ring_scenario(3, 300)
step(road(), 20)
check(not native.replay_ring_local_active(), 'EXIT returns to original exit planner')
native.replay_reset()
check(not native.replay_ring_local_active(), 'camera reset clears handoff state')
print('PASS', checks, 'R17-3 visual disappearance, bend equivalence, single bank, loss and exit checks')
