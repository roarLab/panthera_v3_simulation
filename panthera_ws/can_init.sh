#!/bin/bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
sleep 1
cansend can0 601#2300180510
cansend can0 602#2300180510
cansend can0 603#2300180510
cansend can0 605#2300180510

sleep 2
cansend can0 000#0100
sudo chmod 666 /dev/ttyUSB*
cansend can0 000#0100


#rosrun can_encoder_pub can_encoder_pub_node
