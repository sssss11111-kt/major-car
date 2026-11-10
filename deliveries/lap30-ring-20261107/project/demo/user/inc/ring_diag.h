#ifndef RING_DIAG_H
#define RING_DIAG_H
#include <stdint.h>
#include <string.h>

/* Diagnostic storage only: no decisions, no I/O and no control state writes.
 * 32 frames before the first candidate, then trigger + 63 further frames.
 * These are consumed camera frames, not a claim of lossless sensor capture. */
#define RD29_CAPACITY 96U
#define RD29_PRE 32U
#define RD29_META_WORDS 40U
#define RD29_PIXELS (188U*120U)
#define RD29_PAYLOAD (RD29_PIXELS+360U)
enum {
 RD_FRAME,RD_TICK,RD_RUNNING,RD_FAULT,RD_PHASE_BEFORE,RD_PHASE_AFTER,
 RD_CONFIRM_BEFORE,RD_CONFIRM_AFTER,RD_CANDIDATE_BEFORE,RD_NEAR_BEFORE,
 RD_VALID_BEFORE,RD_ROWS_BEFORE,RD_NEAR_ROWS_BEFORE,RD_STRIPE_BEFORE,
 RD_VALID_AFTER,RD_ROWS_AFTER,RD_NEAR_AFTER,RD_NEAR_ROWS_AFTER,
 RD_BOTH_AFTER,RD_EVIDENCE,RD_CORNER,RD_ISLAND_BOTTOM,RD_ENTRY_ERROR,
 RD_NEAR_ERROR,RD_PREVIEW_ERROR,RD_TURN_ERROR10,RD_COMMAND10,RD_SLEW,
 RD_GUIDE,RD_JUST_ENTERED,RD_ROUTE,RD_YAW10,RD_E1,RD_E2,RD_P1,RD_P2,
 RD_PROCESS_US,RD_COPY_US,RD_READY_CYCLE,RD_THRESHOLD
};
typedef struct {
 int32_t m[RD29_META_WORDS];
 uint8_t pixels[RD29_PAYLOAD];
} rd29_frame_t;
typedef struct {
 uint32_t write,count,triggered,after,frozen,trigger_frame,reason;
} rd29_state_t;

/* Caller writes the image only after reserving a slot. Main is the sole owner. */
static inline int rd29_reserve(rd29_state_t *s,int active,int candidate,int fault,
                              uint32_t frame,uint32_t *slot) {
 if(!active || s->frozen)return 0;
 if(!s->triggered && (candidate || fault)) {
  s->triggered=1;s->trigger_frame=frame;s->reason=fault ? 2U:1U;
 }
 *slot=s->write;
 return 1;
}
static inline void rd29_commit(rd29_state_t *s,int stopped) {
 s->write=(s->write+1U)%RD29_CAPACITY;
 if(s->count<RD29_CAPACITY)++s->count;
 if(s->triggered) {
  ++s->after;
  if(s->after>=RD29_CAPACITY-RD29_PRE)s->frozen=1;
 }else if(s->count>RD29_PRE)s->count=RD29_PRE;
 if(stopped)s->frozen=1;
}
static inline void rd29_freeze(rd29_state_t *s) { if(s->count)s->frozen=1; }
static inline uint32_t rd29_index(const rd29_state_t *s,uint32_t chronological) {
 return (s->write+RD29_CAPACITY-s->count+chronological)%RD29_CAPACITY;
}
static inline uint32_t rd29_crc(const void *data,uint32_t size) {
 const uint8_t *p=(const uint8_t *)data;uint32_t c=0xffffffffU;
 for(uint32_t i=0;i<size;++i) {
  c^=p[i];
  for(uint8_t b=0;b<8;++b)c=(c>>1)^((c&1U)?0xedb88320U:0U);
 }
 return c^0xffffffffU;
}
#endif
