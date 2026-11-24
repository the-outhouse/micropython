# mpexec experiment: classic dual-core ESP32 with no GIL and _thread pinned to
# core 0, so a Python worker runs in parallel with the main task on core 1.
# Core 0 also runs the WiFi/BT stack here, unlike on a P4 + C6.
list(APPEND MICROPY_DEF_BOARD
    MICROPY_HW_BOARD_NAME="Generic ESP32 module, mpexec parallel"
    MICROPY_PY_THREAD_GIL=0
    MP_THREAD_COREID=0
)
