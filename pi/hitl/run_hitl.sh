#!/bin/bash
source /Users/Nik/cmfv-closedloop/.env

/Users/Nik/cmfv-closedloop/pi/venv/bin/python cmfv_hitl.py -ox_port $OCV_PORT -fuel_port $FCV_PORT