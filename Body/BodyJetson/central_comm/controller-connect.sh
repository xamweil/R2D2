#!/usr/bin/env bash

MAC="70:20:84:64:07:2C"
bluetoothctl pair "$MAC"
bluetoothctl trust "$MAC"
bluetoothctl connect "$MAC"
