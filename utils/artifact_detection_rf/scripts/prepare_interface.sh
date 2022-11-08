#!/bin/bash

INTERFACE=wpanda0
CHANNEL=6

sudo ifconfig $INTERFACE down
sudo iw $INTERFACE set type monitor
sudo ifconfig $INTERFACE up
sudo iw dev $INTERFACE set channel $CHANNEL


