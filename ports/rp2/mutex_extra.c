/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mutex_extra.h"

// These functions are taken from lib/pico-sdk/src/common/pico_sync/mutex.c and modified
// so that they atomically obtain the mutex and disable interrupts.

uint32_t __time_critical_func(recursive_mutex_enter_blocking_and_disable_interrupts)(recursive_mutex_t * mtx) {
    lock_owner_id_t caller = lock_get_caller_owner_id();
    do {
        uint32_t save = mp_mutex_spin_lock(mtx);
        if (mtx->owner == caller || !lock_is_owner_id_valid(mtx->owner)) {
            mtx->owner = caller;
            uint __unused total = ++mtx->enter_count;
            mp_mutex_spin_unlock_unsafe(mtx);
            assert(total); // check for overflow
            return save;
        }
        // Spin rather than WFE, so that the exit below needn't SEV (which
        // would wake mp_wfe_or_timeout() in select.poll's wait).
        mp_mutex_spin_unlock(mtx, save);
        tight_loop_contents();
    } while (true);
}

void __time_critical_func(recursive_mutex_exit_and_restore_interrupts)(recursive_mutex_t * mtx, uint32_t save) {
    mp_mutex_spin_lock_unsafe(mtx);
    assert(lock_is_owner_id_valid(mtx->owner));
    assert(mtx->enter_count);
    if (!--mtx->enter_count) {
        mtx->owner = LOCK_INVALID_OWNER_ID;
    }
    // No SEV: a waiter in the enter above spins rather than WFEs.
    mp_mutex_spin_unlock(mtx, save);
}

// A copy of the pico-sdk's recursive_mutex_try_enter(), on the spinlock above.
bool __time_critical_func(recursive_mutex_nowait_try_enter)(recursive_mutex_nowait_t * wrapper, uint32_t *owner_out) {
    recursive_mutex_t *mtx = &wrapper->mutex;
    bool entered;
    lock_owner_id_t caller = lock_get_caller_owner_id();
    uint32_t save = mp_mutex_spin_lock(mtx);
    if (!lock_is_owner_id_valid(mtx->owner) || mtx->owner == caller) {
        mtx->owner = caller;
        uint __unused total = ++mtx->enter_count;
        assert(total); // check for overflow
        entered = true;
    } else {
        if (owner_out) {
            *owner_out = (uint32_t)mtx->owner;
        }
        entered = false;
    }
    mp_mutex_spin_unlock(mtx, save);
    return entered;
}

void __time_critical_func(recursive_mutex_nowait_enter_blocking)(recursive_mutex_nowait_t * mtx) {
    while (!recursive_mutex_nowait_try_enter(mtx, NULL)) {
        tight_loop_contents();
    }
}

void __time_critical_func(recursive_mutex_nowait_exit)(recursive_mutex_nowait_t * wrapper) {
    recursive_mutex_t *mtx = &wrapper->mutex;
    // Rest of this function is a copy of recursive_mutex_exit(), with
    // lock_internal_spin_unlock_with_notify() removed.
    uint32_t save = mp_mutex_spin_lock(mtx);
    assert(lock_is_owner_id_valid(mtx->owner));
    assert(mtx->enter_count);
    if (!--mtx->enter_count) {
        mtx->owner = LOCK_INVALID_OWNER_ID;
    }
    mp_mutex_spin_unlock(mtx, save);
}
