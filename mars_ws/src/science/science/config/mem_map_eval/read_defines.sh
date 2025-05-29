#!/bin/bash
# This script extracts the #define expressions from the mem map
MEM_MAP="../../science_module_arduino/src/mem_manager/mem_map.h"
cp $MEM_MAP ./just_defines.h
swig -python -module just ./just_defines.h ## generates just_defines.py and just_defines_wrap.c
gcc -c -fpic just_defines_wrap.c -I/Library/Frameworks/Python.framework/Versions/3.10/include/python3.10 -I. ## creates just_defines_wrap.o
gcc -shared just_defines_wrap.o -o _just.so -undefined dynamic_lookup ## create _just.so, goes with just_defines.py