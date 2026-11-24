/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George on behalf of Pycom Ltd
 * Copyright (c) 2017 Pycom Limited
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

#include "stdio.h"
#include <string.h>

#include "py/runtime.h"
#include "py/gc.h"
#include "py/mpthread.h"
#include "py/mphal.h"
#include "mpthreadport.h"

#include "esp_task.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#if CONFIG_IDF_TARGET_ARCH_XTENSA
#include "xtensa/hal.h"
#endif

#if MICROPY_PY_THREAD

#define MP_THREAD_MIN_STACK_SIZE                        (4 * 1024)
#define MP_THREAD_DEFAULT_STACK_SIZE                    (MP_THREAD_MIN_STACK_SIZE + MICROPY_STACK_CHECK_MARGIN)
#define MP_THREAD_PRIORITY                              (ESP_TASK_PRIO_MIN + 1)

// The core that _thread threads are pinned to. A board can set this to the
// other core (with MICROPY_PY_THREAD_GIL=0) to run Python on both cores.
#ifndef MP_THREAD_COREID
#define MP_THREAD_COREID                                MP_TASK_COREID
#endif

typedef enum {
    MP_THREAD_RUN_STATE_NEW,
    MP_THREAD_RUN_STATE_RUNNING,
    MP_THREAD_RUN_STATE_FINISHED,
} mp_thread_run_state_t;

// this structure forms a linked list, one node per active thread
typedef struct _mp_thread_t {
    TaskHandle_t id;        // system id of thread
    mp_thread_run_state_t run_state; // current run state of the thread
    void *arg;              // thread Python args, a GC root pointer
    void *stack;            // pointer to the stack
    size_t stack_len;       // number of words in the stack
    struct _mp_thread_t *next;
    #if !MICROPY_PY_THREAD_GIL
    // Stop-the-world GC handshake, see mp_thread_gc_pause_others().
    volatile uint32_t gc_state; // GC_TS_*
    uint32_t gc_depth;          // nesting of GIL_EXIT (safe regions)
    uintptr_t gc_regs[12];      // s0-s11 at the outermost GIL_EXIT
    SemaphoreHandle_t gc_sem;   // wakes the thread when parked
    StaticSemaphore_t gc_sem_buf;
    #endif
} mp_thread_t;

// the mutex controls access to the linked list
static mp_thread_mutex_t thread_mutex;
static mp_thread_t thread_entry0;
static mp_thread_t *thread = NULL; // root pointer, handled by mp_thread_gc_others
#if !MICROPY_PY_THREAD_GIL
static void gc_thread_init(mp_thread_t *th);
#endif

void mp_thread_init(void *stack, uint32_t stack_len) {
    mp_thread_set_state(&mp_state_ctx.thread);
    // create the first entry in the linked list of all threads
    thread_entry0.id = xTaskGetCurrentTaskHandle();
    thread_entry0.run_state = MP_THREAD_RUN_STATE_RUNNING;
    thread_entry0.arg = NULL;
    thread_entry0.stack = stack;
    thread_entry0.stack_len = stack_len;
    thread_entry0.next = NULL;
    thread = &thread_entry0;
    mp_thread_mutex_init(&thread_mutex);
    #if !MICROPY_PY_THREAD_GIL
    mp_thread_gc_stop_req = 0;
    gc_thread_init(&thread_entry0);
    mp_state_ctx.thread.gc_stop_state = &thread_entry0;
    #endif

    // memory barrier to ensure above data is committed
    __sync_synchronize();
}

#if !MICROPY_PY_THREAD_GIL

// Stop-the-world for the mark phase, cooperatively.
//
// Without the GIL the other core keeps running Python while this one
// collects. Scanning its stack then races with it, and its registers aren't
// seen at all: mp_map_rehash(), for one, holds a dict's old table only in a
// callee-saved register while it copies entries, so a collection could free
// the table and the keys not yet moved (docs/todos/esp32-long-soak.md in
// mpexec). So the collector stops every other Python thread before marking,
// and each thread stops *itself*, so that interrupts stay on and no watchdog
// is involved. A thread is in one of three states:
//
//  - RUNNING: may touch the heap. The collector waits for it to leave this.
//  - SAFE: between MP_THREAD_GIL_EXIT() and MP_THREAD_GIL_ENTER(), the
//    places where the GIL build releases the GIL (blocking I/O, sleeps, lock
//    waits). It doesn't touch the heap there, by the same rule as under the
//    GIL. EXIT stored its caller's s0-s11 in gc_regs, which are scanned
//    along with its stack.
//  - PARKED: blocked on gc_sem until the mark is done. Blocking saved all its
//    registers onto its stack, which is scanned.
//
// A RUNNING thread parks at MP_THREAD_GC_SAFEPOINT() (the VM's periodic
// check, and the GC-lock wait) when mp_thread_gc_stop_req is set. ENTER
// parks too if a stop is in progress. A thread stores its state, then reads
// the request; the collector stores the request, then reads the states. Both
// use seq_cst accesses, so at least one of them sees the other.
//
// The collector holds the GC lock and then thread_mutex (the order
// mp_thread_deinit() uses), so no thread can start, finish or be created
// meanwhile. Waits on any mp_thread_mutex are safe regions, so a thread
// blocked on a lock the collector holds doesn't hold the collection up.
// What can hold it up: a long C call that neither allocates nor reaches a
// safe region (hashing a big buffer, say). The collector waits it out.

enum { GC_TS_RUNNING = 0, GC_TS_SAFE = 1, GC_TS_PARKED = 2 };

volatile uint32_t mp_thread_gc_stop_req;
static uintptr_t gc_scratch_regs[12];

static mp_thread_t *gc_self(void) {
    mp_state_thread_t *ts = mp_thread_get_state();
    return ts != NULL ? ts->gc_stop_state : NULL;
}

static void gc_thread_init(mp_thread_t *th) {
    th->gc_state = GC_TS_RUNNING;
    th->gc_depth = 0;
    th->gc_sem = xSemaphoreCreateBinaryStatic(&th->gc_sem_buf);
}

uintptr_t *mp_thread_gc_safe_regs(void) {
    mp_thread_t *me = gc_self();
    // Only the outermost EXIT's registers matter; nested ones are scratch.
    return (me != NULL && me->gc_depth == 0) ? me->gc_regs : gc_scratch_regs;
}

#if CONFIG_IDF_TARGET_ARCH_XTENSA
// The RISC-V build gets gc_helper_get_regs_and_sp() from gchelper_rv32i.s.
// With Xtensa's register windows, a caller's registers stay in the register
// file until a window overflow spills them to its frame's save area on the
// stack. xthal_window_spill() (the HAL routine FreeRTOS's port uses) writes
// out every window except the calling function's, so this must stay a real
// call: noinline, or the GIL_EXIT site's own window would be the one left
// out. (Nesting calls the way gc_collect_inner() does isn't reliable: GCC
// inlines and folds the recursion.) The collector scans the whole thread
// stack, so nothing needs to go into regs.
__attribute__((noinline)) uintptr_t gc_helper_get_regs_and_sp(uintptr_t *regs) {
    memset(regs, 0, 12 * sizeof(uintptr_t));
    xthal_window_spill();
    return (uintptr_t)esp_cpu_get_sp();
}
#endif

void mp_thread_gc_safe_enter(uintptr_t sp) {
    (void)sp;
    mp_thread_t *me = gc_self();
    if (me != NULL && me->gc_depth++ == 0) {
        __atomic_store_n(&me->gc_state, GC_TS_SAFE, __ATOMIC_SEQ_CST);
    }
}

// Become RUNNING, unless a stop is in progress: then park until it's over.
static void gc_run_or_park(mp_thread_t *me) {
    for (;;) {
        __atomic_store_n(&me->gc_state, GC_TS_RUNNING, __ATOMIC_SEQ_CST);
        if (!__atomic_load_n(&mp_thread_gc_stop_req, __ATOMIC_SEQ_CST)) {
            return;
        }
        __atomic_store_n(&me->gc_state, GC_TS_PARKED, __ATOMIC_SEQ_CST);
        // A stale give from an earlier stop just goes round the loop again.
        while (__atomic_load_n(&mp_thread_gc_stop_req, __ATOMIC_SEQ_CST)) {
            xSemaphoreTake(me->gc_sem, portMAX_DELAY);
        }
    }
}

void mp_thread_gc_safe_exit(void) {
    mp_thread_t *me = gc_self();
    if (me != NULL && me->gc_depth > 0 && --me->gc_depth == 0) {
        gc_run_or_park(me);
    }
}

void mp_thread_gc_park(void) {
    mp_thread_t *me = gc_self();
    if (me != NULL && me->gc_depth == 0) {
        gc_run_or_park(me);
    }
}

// Stop-the-world timing, per core that started the collection. Read with
// esp32.gc_stop_stats(). Updated by the collector while it holds thread_mutex.
static mp_thread_gc_stop_stats_t mp_thread_gc_stop_stats[2];
static int64_t gc_stop_t0;
static int gc_stop_core;

// MICROPY_GC_HOOK_COLLECT_START: the GC lock is held, nothing is marked yet.
void mp_thread_gc_pause_others(void) {
    gc_stop_t0 = esp_timer_get_time();
    gc_stop_core = esp_cpu_get_core_id() & 1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    __atomic_store_n(&mp_thread_gc_stop_req, 1, __ATOMIC_SEQ_CST);
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    for (mp_thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == self || th->run_state != MP_THREAD_RUN_STATE_RUNNING) {
            continue;
        }
        while (__atomic_load_n(&th->gc_state, __ATOMIC_SEQ_CST) == GC_TS_RUNNING) {
            taskYIELD(); // in case it's on this core, at the same priority
        }
    }
    mp_thread_gc_stop_stats_t *st = &mp_thread_gc_stop_stats[gc_stop_core];
    uint32_t wait = (uint32_t)(esp_timer_get_time() - gc_stop_t0);
    st->n++;
    st->wait_sum_us += wait;
    if (wait > st->wait_max_us) {
        st->wait_max_us = wait;
    }
    if (wait >= 10000) {
        st->wait_over_10ms++;
    }
}

// MICROPY_GC_HOOK_MARK_DONE: before finalisers and the sweep.
void mp_thread_gc_resume_others(void) {
    __atomic_store_n(&mp_thread_gc_stop_req, 0, __ATOMIC_SEQ_CST);
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    for (mp_thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id != self && th->run_state == MP_THREAD_RUN_STATE_RUNNING
            && __atomic_load_n(&th->gc_state, __ATOMIC_SEQ_CST) == GC_TS_PARKED) {
            xSemaphoreGive(th->gc_sem);
        }
    }
    mp_thread_gc_stop_stats_t *st = &mp_thread_gc_stop_stats[gc_stop_core];
    uint32_t stop = (uint32_t)(esp_timer_get_time() - gc_stop_t0);
    st->stop_sum_us += stop;
    if (stop > st->stop_max_us) {
        st->stop_max_us = stop;
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

void mp_thread_gc_stop_stats_read(mp_thread_gc_stop_stats_t *out, bool reset) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    memcpy(out, mp_thread_gc_stop_stats, sizeof(mp_thread_gc_stop_stats));
    if (reset) {
        memset(mp_thread_gc_stop_stats, 0, sizeof(mp_thread_gc_stop_stats));
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

#endif // !MICROPY_PY_THREAD_GIL

void mp_thread_gc_others(void) {
    #if MICROPY_PY_THREAD_GIL
    mp_thread_mutex_lock(&thread_mutex, 1);
    #else
    // thread_mutex is held by mp_thread_gc_pause_others().
    #endif
    for (mp_thread_t *th = thread; th != NULL; th = th->next) {
        gc_collect_root((void **)&th, 1);
        gc_collect_root(&th->arg, 1); // probably not needed
        if (th->id == xTaskGetCurrentTaskHandle()) {
            continue;
        }
        if (th->run_state != MP_THREAD_RUN_STATE_RUNNING) {
            continue;
        }
        gc_collect_root(th->stack, th->stack_len);
        #if !MICROPY_PY_THREAD_GIL
        if (th->gc_state == GC_TS_SAFE) {
            gc_collect_root((void **)th->gc_regs, MP_ARRAY_SIZE(th->gc_regs));
        }
        #endif
    }
    #if MICROPY_PY_THREAD_GIL
    mp_thread_mutex_unlock(&thread_mutex);
    #endif
}

mp_state_thread_t *mp_thread_get_state(void) {
    return pvTaskGetThreadLocalStoragePointer(NULL, 1);
}

void mp_thread_set_state(mp_state_thread_t *state) {
    vTaskSetThreadLocalStoragePointer(NULL, 1, state);
}

mp_uint_t mp_thread_get_id(void) {
    return (mp_uint_t)xTaskGetCurrentTaskHandle();
}

void mp_thread_start(void) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (mp_thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            th->run_state = MP_THREAD_RUN_STATE_RUNNING;
            #if !MICROPY_PY_THREAD_GIL
            // No collection can be in progress: it would hold thread_mutex.
            th->gc_state = GC_TS_RUNNING;
            th->gc_depth = 0;
            MP_STATE_THREAD(gc_stop_state) = th;
            #endif
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

static void *(*ext_thread_entry)(void *) = NULL;

static void freertos_entry(void *arg) {
    // Run the Python code.
    if (ext_thread_entry) {
        ext_thread_entry(arg);
    }
    #if !MICROPY_PY_THREAD_GIL
    // The thread's mp_state_thread_t lived in that call's stack frame, and
    // the thread_mutex wait below may look at it.
    mp_thread_set_state(NULL);
    #endif

    // Remove the thread from the linked-list of active threads.
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (mp_thread_t **th = &thread; *th != NULL; th = &(*th)->next) {
        if ((*th)->id == xTaskGetCurrentTaskHandle()) {
            *th = (*th)->next;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);

    // Delete this FreeRTOS task (this call to vTaskDelete will not return).
    vTaskDelete(NULL);
}

mp_uint_t mp_thread_create_ex(void *(*entry)(void *), void *arg, size_t *stack_size, int priority, char *name) {
    // store thread entry function into a global variable so we can access it
    ext_thread_entry = entry;

    if (*stack_size == 0) {
        *stack_size = MP_THREAD_DEFAULT_STACK_SIZE; // default stack size
    } else if (*stack_size < MP_THREAD_MIN_STACK_SIZE) {
        *stack_size = MP_THREAD_MIN_STACK_SIZE; // minimum stack size
    }

    // Allocate linked-list node (must be outside thread_mutex lock)
    mp_thread_t *th = m_new_obj(mp_thread_t);

    mp_thread_mutex_lock(&thread_mutex, 1);

    // create thread
    BaseType_t result = xTaskCreatePinnedToCore(freertos_entry, name, *stack_size / sizeof(StackType_t), arg, priority, &th->id, MP_THREAD_COREID);
    if (result != pdPASS) {
        mp_thread_mutex_unlock(&thread_mutex);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("can't create thread"));
    }

    // add thread to linked list of all threads
    th->run_state = MP_THREAD_RUN_STATE_NEW;
    #if !MICROPY_PY_THREAD_GIL
    gc_thread_init(th);
    #endif
    th->arg = arg;
    th->stack = pxTaskGetStackStart(th->id);
    th->stack_len = *stack_size / sizeof(uintptr_t);
    th->next = thread;
    thread = th;

    mp_thread_mutex_unlock(&thread_mutex);

    return (mp_uint_t)th->id;
}

mp_uint_t mp_thread_create(void *(*entry)(void *), void *arg, size_t *stack_size) {
    return mp_thread_create_ex(entry, arg, stack_size, MP_THREAD_PRIORITY, "mp_thread");
}

void mp_thread_finish(void) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (mp_thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            th->run_state = MP_THREAD_RUN_STATE_FINISHED;
            #if !MICROPY_PY_THREAD_GIL
            MP_STATE_THREAD(gc_stop_state) = NULL; // the collector ignores it now
            #endif
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

void mp_thread_mutex_init(mp_thread_mutex_t *mutex) {
    // Need a binary semaphore so a lock can be acquired on one Python thread
    // and then released on another.
    mutex->handle = xSemaphoreCreateBinaryStatic(&mutex->buffer);
    xSemaphoreGive(mutex->handle);
}

int mp_thread_mutex_lock(mp_thread_mutex_t *mutex, int wait) {
    #if !MICROPY_PY_THREAD_GIL
    // A blocking wait is a safe region, so a collector holding this lock (the
    // qstr lock, say) doesn't wait for us while we wait for it.
    if (wait && xSemaphoreTake(mutex->handle, 0) != pdTRUE) {
        MP_THREAD_GIL_EXIT();
        BaseType_t got = xSemaphoreTake(mutex->handle, portMAX_DELAY);
        MP_THREAD_GIL_ENTER();
        return pdTRUE == got;
    } else if (wait) {
        return 1;
    }
    #endif
    return pdTRUE == xSemaphoreTake(mutex->handle, wait ? portMAX_DELAY : 0);
}

void mp_thread_mutex_unlock(mp_thread_mutex_t *mutex) {
    xSemaphoreGive(mutex->handle);
    // Python threads run at equal priority, so preemptively yield here to
    // prevent pathological imbalances where a thread unlocks and then
    // immediately re-locks a mutex before a context switch can occur, leaving
    // another thread waiting for an unbounded period of time.
    taskYIELD();
}

void mp_thread_deinit(void) {
    // The current task should be thread_entry0 and should be the last in the linked list.
    assert(thread_entry0.id == xTaskGetCurrentTaskHandle());
    assert(thread_entry0.next == NULL);

    #if MICROPY_PY_THREAD_RECURSIVE_MUTEX
    // Without the GIL the other threads may be running on the other core.
    // Hold the qstr and GC locks (in the order qstr_add() takes them) so no
    // thread is deleted while it holds one. A thread that is running on the
    // other core isn't deleted at once: it runs on until that core yields,
    // and may block on one of these locks meanwhile. Giving a lock back
    // would then wake a deleted task, so the locks are re-created instead.
    mp_thread_mutex_lock(&MP_STATE_VM(qstr_mutex), 1);
    mp_thread_recursive_mutex_lock(&MP_STATE_MEM(gc_mutex), 1);
    #endif

    // Delete all tasks except the main one.
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (mp_thread_t *th = thread; th != NULL; th = th->next) {
        if (th != &thread_entry0) {
            vTaskDelete(th->id);
        }
    }
    thread = &thread_entry0;

    #if MICROPY_PY_THREAD_RECURSIVE_MUTEX
    // Let the other core switch away from the deleted tasks, then start the
    // locks afresh. Soft reset re-creates them again in gc_init()/qstr_init().
    vTaskDelay(2);
    mp_thread_mutex_init(&thread_mutex);
    mp_thread_mutex_init(&MP_STATE_VM(qstr_mutex));
    mp_thread_recursive_mutex_init(&MP_STATE_MEM(gc_mutex));
    #else
    mp_thread_mutex_unlock(&thread_mutex);
    #endif

    // Give the idle task a chance to run, to clean up any deleted tasks.
    vTaskDelay(1);
}

#if MICROPY_PY_THREAD_RECURSIVE_MUTEX

// The GC lock. Without the GIL every heap operation takes it, so it must be
// cheap: a FreeRTOS recursive mutex made single-thread allocation 2-3x slower.
// This is a spinning lock like rp2's recursive_mutex_nowait_t: an owner word
// set by CAS plus a count that only the owner touches. Uncontended, a take is
// one CAS and a release is one store, with no critical section. A waiter
// spins, yielding to equal-priority tasks on its core; all MicroPython
// threads have the same priority, so there is no priority inversion. It is
// never taken from a hard ISR. The lock must be in internal RAM: on Xtensa,
// esp_cpu_compare_and_set() disables interrupts for PSRAM addresses.

void mp_thread_recursive_mutex_init(mp_thread_recursive_mutex_t *mutex) {
    mutex->count = 0;
    __atomic_store_n(&mutex->owner, 0, __ATOMIC_RELEASE);
}

int mp_thread_recursive_mutex_lock(mp_thread_recursive_mutex_t *mutex, int wait) {
    uint32_t self = (uint32_t)xTaskGetCurrentTaskHandle();
    // Only this task ever stores its own handle, so a plain read is enough.
    if (mutex->owner == self) {
        ++mutex->count;
        return 1;
    }
    while (!esp_cpu_compare_and_set(&mutex->owner, 0, self)) {
        if (!wait) {
            return 0;
        }
        // The collector holds this lock while it waits for us to stop.
        MP_THREAD_GC_SAFEPOINT();
        taskYIELD();
    }
    // IDF's CAS has no acquire ordering (and no compiler barrier), so fence
    // before touching anything the lock protects.
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    mutex->count = 1;
    return 1;
}

void mp_thread_recursive_mutex_unlock(mp_thread_recursive_mutex_t *mutex) {
    if (--mutex->count == 0) {
        __atomic_store_n(&mutex->owner, 0, __ATOMIC_RELEASE);
    }
}

#endif // MICROPY_PY_THREAD_RECURSIVE_MUTEX

#endif // MICROPY_PY_THREAD
