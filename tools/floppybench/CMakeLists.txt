cmake_minimum_required(VERSION 3.9)
project(FloppyBench C)

# A small app for 68000 Macs (Mac Plus and up, System 6.0.8+), with its own text window rather
# than Retro68's console library (which brings in libstdc++). Builds FloppyBench.dsk, an 800K
# HFS disk image holding the app, which the emulator can serve directly. floppybench.r asks for a
# 2.5 MB partition (the format test copies the disk to memory).
add_application(FloppyBench
    floppybench.c
    floppybench.r
    CREATOR "FBen"
)

set_target_properties(FloppyBench PROPERTIES
    C_STANDARD 99
    # -Wno-array-bounds: GCC flags the Toolbox's reads of low-memory globals (near address 0).
    COMPILE_OPTIONS "-ffunction-sections;-O2;-Wall;-Wno-array-bounds"
    LINK_FLAGS "-Wl,-gc-sections -Wl,--mac-strip-macsbug")
