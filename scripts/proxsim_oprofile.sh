#!/bin/bash

sudo opcontrol --no-vmlinux --start
sudo opcontrol --reset
sudo opcontrol --callgraph=6

../build/proxsim "$@"

sudo opcontrol --stop
sudo opcontrol --shutdown

opreport \*prox\* > oprofile.out
opreport -l \*prox\* >> oprofile.out
cat oprofile.out
