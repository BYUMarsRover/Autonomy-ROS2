#!/bin/bash

moist_raw=$1
moist_exp=-1.8
moist_mult=522625.0

moist_exped=$(bc -l <<<"${moist_raw}^${moist_exp}")
moist=$(bc -l <<<"${moist_exped}*${moist_mult}")

echo "$moist"