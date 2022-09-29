#!/bin/bash

g++ -O2 -g -Wall --std=c++14 \
    -Wno-psabi \
    -I ./ -I /opt/vc/include -L /opt/vc/lib \
    -o moteus_control_example \
    moteus_pi3hat_ERT_wrapper.cc \
    pi3hat.cc \
    -lbcm_host \
    -lpthread
