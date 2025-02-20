# This script does a linear interpolation between
# the measured values from the science moisture module
# and the calibration curve calculated. 
# 
# tail_X is the ADC-value reported by the science module
# siemens_X is the number of microSiemens/cm (measured using garden probe)
# wat_X is the number of grams of water introduced into the calibration soil.
#   - note that the soil is assumed to have 0 grams of dirt to start.


# ENTER MEASUREMENT HERE
measurement = 894

tail_0 = 930
tail_1 = 64

siemens_0 = 2
siemens_1 = 7

wat_0 = 0
wat_1 = 1

siemens = 2 + (measurement - tail_0) / (tail_1 - tail_0) * (siemens_1 - siemens_0)
print(f'microSiemens/cm: {siemens}')

water = (measurement - tail_0) / (tail_1 - tail_0) * (wat_1 - wat_0)
print(f'g water per 202 g local soil: {water}')
print(f'mass percentage of water = {water / 202}')