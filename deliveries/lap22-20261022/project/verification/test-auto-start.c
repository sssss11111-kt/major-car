#include <assert.h>
#include <stdio.h>
#include "auto_start.h"
#undef assert
#define assert(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL at line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

int main(void)
{
    auto_start_t s = {0};
    assert(auto_start_step(&s, 0, 0, 0, 0) == AUTO_WAIT);
    assert(auto_start_step(&s, 10, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 109, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 110, 1, 0, 0) == AUTO_STARTED);
    assert(auto_start_step(&s, 111, 0, 0, 0) == AUTO_STARTED);

    s = (auto_start_t){0};
    assert(auto_start_step(&s, 10, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 109, 0, 0, 0) == AUTO_WAIT);
    assert(auto_start_step(&s, 110, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 209, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 210, 1, 0, 0) == AUTO_STARTED);

    s = (auto_start_t){0};
    assert(auto_start_step(&s, 10, 1, 1, 0) == AUTO_WAIT);
    assert(auto_start_step(&s, 20, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 119, 1, 1, 0) == AUTO_WAIT);
    assert(auto_start_step(&s, 120, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 219, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 220, 1, 0, 0) == AUTO_STARTED);

    for (int phase = AUTO_WAIT; phase <= AUTO_STARTED; ++phase) {
        s = (auto_start_t){(auto_phase_t)phase, 0};
        assert(auto_start_step(&s, 10, 1, 0, 1) == AUTO_LOCKED);
        assert(auto_start_step(&s, 1000, 1, 0, 0) == AUTO_LOCKED);
    }
    s = (auto_start_t){0};
    assert(auto_start_step(&s, 0xfffffff0U, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 0x53U, 1, 0, 0) == AUTO_CENTER);
    assert(auto_start_step(&s, 0x54U, 1, 0, 0) == AUTO_STARTED);
    puts("PASS: 1s delay, lost-track restart, snapshot inhibition, latched stop, one start, tick wrap");
    return 0;
}
