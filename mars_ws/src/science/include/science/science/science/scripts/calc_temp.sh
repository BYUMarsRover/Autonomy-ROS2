#!/bin/bash

temp_raw=$1
temp_mult=-0.107 
temp_add=80.3

temp_multed=$(bc -l <<<"${temp_mult}*${temp_raw}")
temp=$(bc -l <<<"${temp_multed}+${temp_add}")

echo "Temp: $temp"