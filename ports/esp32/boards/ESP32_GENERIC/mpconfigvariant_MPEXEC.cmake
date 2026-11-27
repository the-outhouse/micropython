# mpexec experiment: classic dual-core ESP32 with no GIL and _thread pinned to
# core 0, so a Python worker runs in parallel with the main task on core 1.
# Core 0 also runs the WiFi/BT stack here, unlike on a P4 + C6.
#
# The board name stays stock. ", mpexec" after the MCU name marks the build,
# as on rp2 ("Generic ESP32 module with ESP32, mpexec"); mpexec's caps.py
# reads it to know threads run off the loop's core.
list(APPEND MICROPY_DEF_BOARD
    MICROPY_HW_MCU_NAME="ESP32, mpexec"
    MICROPY_PY_THREAD_GIL=0
    MP_THREAD_COREID=0
)
