#!/bin/bash

mavproxy.py --master=$1 --out=127.0.0.1:14550 --baudrate=115200 --source_system=$2
