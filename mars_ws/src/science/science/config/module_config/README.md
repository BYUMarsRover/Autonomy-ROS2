# module_config Documentation

## Purpose

The Science Module saves certain configuration values to the EEPROM on board of the nano. These are described in the memory map of the Arduino Source Code. These include the UV Sensitivty and extend and retract rates for each actuator. Should the module require other configuration variables, they could be added here.

The `module_config` directory contains code to author a serial communication packet to write these varibles to the correct location in the module's EEPROM.

## Usage

### Update Values

- Modify the values in the file titled `module.config`.
- Run the `generate.sh` bash script, you may use a `-o` flag to modify the output file name, else it will be called `config_upload_packets.bin`.
- Send the output file's contents to the science module via a serial connection to overwrite the configuration data.

### Adding new configuration varibles

- New configuration varibles should first be placed in the memory map of the Arduino Source Code.
- The new configuration variable should be added to the `module.config` file, mantaining the order of the variables in the memory map.
- The new configuration variable's name and datatype should be added to the `fields.csv` to allow the generate script to convert the value to the correct binary representation.
- If the datatype is unrecognized, the `compile_config.py` script may need to be modified to accomate this.

## Configuration Fields

### UV Sensitivity

The UV counts that are returned from the LTR390 sensor are divided by this value to convert to a UV index. A typical value as per the datasheet is 2300, but this can be modified to calibrate the resulting UV Index reading.

Two science module actions exisit which modify this variable. They are named `calibrate_uv_sensitivty` and `calibrate_uv_index`.

This value is read from memory after each reading. No reset is required after modifying it's value.

### Actuator Retract and Extend Times

These values are recorded as seconds in floats. They are used to estimate the current state of the actuators through integration as no encoders are currently deployed on the science module.

These values should be determined emperically by extending and retracting the actuators at full speed with a stop watch.

**These values are loaded into the Arduino's RAM upon bootup, so the module must be reset after changes are made in order to see the new behaviour.**