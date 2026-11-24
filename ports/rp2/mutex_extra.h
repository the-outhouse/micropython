/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_RP2_MUTEX_EXTRA_H
#define MICROPY_INCLUDED_RP2_MUTEX_EXTRA_H

#include "pico/mutex.h"

// The spinlock that guards MicroPython's own mutexes (the atomic section's and
// the "nowait" ones below). On RP2350 the pico-sdk uses software spinlocks
// because of erratum RP2350-E2, and taking one sets this core's event flag
// (LDAEX/STREX), so a WFE right after returns at once. These mutexes are taken
// inside select.poll's wait (lwIP's lock, the atomic section), where that turns
// mp_wfe_or_timeout() into a spin. So they use one hardware spinlock from the
// set the pico-sdk lists as E2-safe (5-7, 10-11, 26-31) instead: with software
// spinlocks selected, nothing else in the pico-sdk uses the hardware ones.
#if PICO_RP2350 && PICO_USE_SW_SPIN_LOCKS
#ifndef MICROPY_RP2_MUTEX_HW_SPIN_LOCK_NUM
#define MICROPY_RP2_MUTEX_HW_SPIN_LOCK_NUM (31)
#endif
#define MP_MUTEX_HW_SPIN_LOCK ((io_rw_32 *)(SIO_BASE + SIO_SPINLOCK0_OFFSET + 4 * MICROPY_RP2_MUTEX_HW_SPIN_LOCK_NUM))

__force_inline static void mp_mutex_spin_lock_unsafe(recursive_mutex_t *mtx) {
    (void)mtx;
    while (!*MP_MUTEX_HW_SPIN_LOCK) { // reading claims the lock, and posts no event
        tight_loop_contents();
    }
    __mem_fence_acquire();
}

__force_inline static void mp_mutex_spin_unlock_unsafe(recursive_mutex_t *mtx) {
    (void)mtx;
    __mem_fence_release();
    *MP_MUTEX_HW_SPIN_LOCK = 0;
}

__force_inline static uint32_t mp_mutex_spin_lock(recursive_mutex_t *mtx) {
    uint32_t save = save_and_disable_interrupts();
    mp_mutex_spin_lock_unsafe(mtx);
    return save;
}

__force_inline static void mp_mutex_spin_unlock(recursive_mutex_t *mtx, uint32_t save) {
    mp_mutex_spin_unlock_unsafe(mtx);
    restore_interrupts_from_disabled(save);
}
#else
__force_inline static void mp_mutex_spin_lock_unsafe(recursive_mutex_t *mtx) {
    spin_lock_unsafe_blocking(mtx->core.spin_lock);
}

__force_inline static void mp_mutex_spin_unlock_unsafe(recursive_mutex_t *mtx) {
    spin_unlock_unsafe(mtx->core.spin_lock);
}

__force_inline static uint32_t mp_mutex_spin_lock(recursive_mutex_t *mtx) {
    return spin_lock_blocking(mtx->core.spin_lock);
}

__force_inline static void mp_mutex_spin_unlock(recursive_mutex_t *mtx, uint32_t save) {
    spin_unlock(mtx->core.spin_lock, save);
}
#endif

uint32_t recursive_mutex_enter_blocking_and_disable_interrupts(recursive_mutex_t *mtx);
void recursive_mutex_exit_and_restore_interrupts(recursive_mutex_t *mtx, uint32_t save);

// Alternative version of recursive_mutex_t that doesn't issue WFE and SEV
// instructions. This means it will use more power (busy-waits), but exiting
// this mutex doesn't disrupt the calling CPU's event state in the same way a
// recursive mutex does (because recurse_mutex_exit() executes SEV each time the
// mutex is released.)
//
// Implement as a wrapper type because no longer compatible with the normal
// recursive_mutex functions.

typedef struct {
    recursive_mutex_t mutex;
} recursive_mutex_nowait_t;

static inline void recursive_mutex_nowait_init(recursive_mutex_nowait_t *mtx) {
    recursive_mutex_init(&mtx->mutex);
}

bool recursive_mutex_nowait_try_enter(recursive_mutex_nowait_t *mtx, uint32_t *owner_out);

void recursive_mutex_nowait_enter_blocking(recursive_mutex_nowait_t *mtx);
void recursive_mutex_nowait_exit(recursive_mutex_nowait_t *mtx);

#endif // MICROPY_INCLUDED_RP2_MUTEX_EXTRA_H
