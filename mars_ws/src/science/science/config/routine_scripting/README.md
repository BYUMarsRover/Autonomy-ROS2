# routine_scripting Documentation

## Purpose

The Science Module can run routines, or scripted actions in sequence in order to align actuators or perform an action. These are stored in the modules EEPROM.

The `routine_scripting` directory contains code write such routines in an assembly-like style and author a serial communication packet to commit them to the modules EEPROM.

## Writing Routine Scripts

- The directory `routine_scripts` contains all the custom routines
- These will be compiled in alphabetical order such that the first listed script is indexed as Routine 0, the second as Routine 1 and so forth. It is recommended to name the scripts like `0-reset.routine`, `1-move_to_zero.routine`, `2_align_first_cache.routine` to enforce the intented order.
- Routine scripts support `#` style comments
- All tokens must be listed in the `dictionary.csv` or the compiliation will fail. **All operands must be turned into varibles and cannot be left as simple numbers in the script.** (Note: This could be improved in the future using some assumptions about what data types should follow a command to allow for numeric data)

## Command Descriptions

>`GROUP` - All actions after this and before the next `GROUP` call will be executed at the same time. The routine will not move on to the next group until all these actions are completed.

>`POS_CTRL` - Configures a positional controller request. Will move an actuator to a specified position. Takes an actuator index and a position as arguments e.g. `POS_CTRL AUGER_INDEX P_ZERO` 

>`SPD_CTRL` - Configures a speed controller request. Will move an actuator at a specified speed for a certain duration. Takes an actuator index, speed, and duration as arguments e.g. `POS_CTRL AUGER_INDEX P_ZERO` 

>`LOCKDOWN_ACTUATORS` - Normally a routine does not stop actuators which are not currently being controlled by a group from manipulation by other commands outside of the routine. If this is undesired behaviour, this command will prevent all actuators in the entire routine from begin controlled until the routine resolves.

>`FUNC` - Configure the current group to execute a callback function upon completion. These functions are set inside of the Arduino Source Code as a lookup table. The `FUNC` command takes the index of the function pointer as an operand.

## Uploading to Module

- Run the `generate.sh` bash script to compile the routines, you may use a `-o` flag to modify the output file name, else it will be called `routine_upload_packets.bin`.
- Send the output file's contents to the science module via a serial connection to write new routine data.