# mpexec experiment: pre-rev3 P4 with no GIL and _thread pinned to core 0, so
# a Python worker runs in parallel with the main task on core 1.
list(APPEND SDKCONFIG_DEFAULTS
    boards/sdkconfig.p4_pre_rev3
)

list(APPEND MICROPY_DEF_BOARD
    MICROPY_HW_BOARD_NAME="Generic ESP32P4 with pre revision 3 chip, mpexec parallel"
    MICROPY_PY_THREAD_GIL=0
    MP_THREAD_COREID=0
)
