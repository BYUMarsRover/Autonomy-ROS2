class FunctionMap
{
    String csvPath = "../function_mapping/science_module_function_map.csv";
    Table functionTable;

    byte headerByte = 0x53;
    byte footerByte = 0x4D;

    FunctionMap()
    {
        functionTable = loadTable(csvPath, "header");
    }

    byte GetCommandWord(String function_name)
    {
        TableRow function = functionTable.findRow(function_name, "function_name");
        if (function == null) return byte(-1);
        else
        {
            String command_type = function.getString("command_type");
            int function_addr = int(function.getString("function_addr"));
            byte command_word = byte(function_addr);
            if (command_type.equals("action")) command_word |= 0b10000000;
            return command_word;
        }
    }

    ScienceModuleCommand GetCommand(String function_name, byte[] operands)
    {
        byte command_word = GetCommandWord(function_name);
        return new ScienceModuleCommand(command_word, operands);
    }

    // User mappings to keep magic values all in the same place

    ScienceModuleCommand GetCommand_GetActuatorPosition(int sensor_index)
    {
        byte command_word = GetCommandWord("get_actuator_position");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index)});
    }

    ScienceModuleCommand GetCommand_UpdateActuatorPosition(int sensor_index, int position)
    {
        byte command_word = GetCommandWord("update_actuator_position");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index), byte(position)});
    }

    ScienceModuleCommand GetCommand_GetActuatorControl(int sensor_index)
    {
        byte command_word = GetCommandWord("get_actuator_position");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index)});
    }

    ScienceModuleCommand GetCommand_UpdateActuatorControl(int sensor_index, int control)
    {
        byte command_word = GetCommandWord("update_actuator_control");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index), byte(control)});
    }

    ScienceModuleCommand GetCommand_GetAnalogSensorCalibrated(int sensor_index)
    {
        byte command_word = GetCommandWord("get_analog_sensor_calibrated");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index)});
    }

    ScienceModuleCommand GetCommand_GetAnalogSensorRaw(int sensor_index)
    {
        byte command_word = GetCommandWord("get_analog_sensor_raw");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index)});
    }

    ScienceModuleCommand GetCommand_ClearAnalogCalibration(int sensor_index)
    {
        byte command_word = GetCommandWord("clear_analog_calibration");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index)});
    }

    ScienceModuleCommand GetCommand_GetCalibrationData(int sensor_index)
    {
        byte command_word = GetCommandWord("get_calibration_data");
        return new ScienceModuleCommand(command_word, new byte[]{byte(sensor_index)});
    }

    ScienceModuleCommand GetCommand_SubmitAnalogCoefficients(int sensor_index, float[] coeffs)
    {
        byte command_word = GetCommandWord("submit_analog_coefficients");
        byte[] operands = new byte[25];
        operands[0] = byte(sensor_index);
        for (int i = 0; i < 6; i++)
        {
            float f = coeffs[i];
            byte[] b = ByteBuffer.allocate(4).putFloat(f).array(); 
            operands[(i * 4) + 0 + 1] = b[3];
            operands[(i * 4) + 1 + 1] = b[2];
            operands[(i * 4) + 2 + 1] = b[1];
            operands[(i * 4) + 3 + 1] = b[0];
        }

        return new ScienceModuleCommand(command_word, operands);
    }

    ScienceModuleCommand GetCommand_ResetSpectrograph()
    {
        byte command_word = GetCommandWord("reset_spectrograph");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_ReturnSpectrographData()
    {
        byte command_word = GetCommandWord("return_spectrograph_data");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_SampleSpectrograph(int num_samples, int duration_mus, boolean use_bulb)
    {
        byte command_word = GetCommandWord("sample_spectrograph");
        byte[] dur = ByteBuffer.allocate(4).putInt(duration_mus).array(); 
        byte[] operands = new byte[]{byte(num_samples), dur[3], dur[2], dur[1], dur[0], byte(use_bulb)};
        return new ScienceModuleCommand(command_word, operands);
    }

    ScienceModuleCommand GetCommand_ResetLTR()
    {
        byte command_word = GetCommandWord("reset_ltr");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_SampleLTR()
    {
        byte command_word = GetCommandWord("sample_ltr");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_ReturnLTRData()
    {
        byte command_word = GetCommandWord("return_ltr_data");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_RunRoutine(int routine_index)
    {
        byte command_word = GetCommandWord("run_routine");
        byte[] operands = new byte[]{byte(routine_index)};
        return new ScienceModuleCommand(command_word, operands);
    }

    ScienceModuleCommand GetCommand_PauseRoutine()
    {
        byte command_word = GetCommandWord("pause_routine");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_ResumeRoutine()
    {
        byte command_word = GetCommandWord("resume_routine");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_StepRoutine()
    {
        byte command_word = GetCommandWord("step_routine");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_AbortRoutine()
    {
        byte command_word = GetCommandWord("abort_routine");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_QueryRoutineController()
    {
        byte command_word = GetCommandWord("query_routine_controller");
        return new ScienceModuleCommand(command_word);
    }

    ScienceModuleCommand GetCommand_SubmitPositionalControl(int actuator_index, int position, int speed)
    {
        byte command_word = GetCommandWord("submit_positional_controller");
        byte[] operands = new byte[]{byte(actuator_index), byte(position), byte(speed)};
        return new ScienceModuleCommand(command_word, operands);
    }

    ScienceModuleCommand GetCommand_QueryPositionalController(int actuator_index)
    {
        byte command_word = GetCommandWord("query_positional_controller");
        return new ScienceModuleCommand(command_word, new byte[]{byte(actuator_index)});
    }

    ScienceModuleCommand GetCommand_ClearPositionalController(int actuator_index)
    {
        byte command_word = GetCommandWord("clear_positional_control");
        return new ScienceModuleCommand(command_word, new byte[]{byte(actuator_index)});
    }

    ScienceModuleCommand GetCommand_SubmitSpeedControl(int actuator_index, int control, int time)
    {
        byte command_word = GetCommandWord("submit_speed_control");
        byte[] timeout = ByteBuffer.allocate(4).putInt(time).array(); 
        byte[] operands = new byte[]{byte(actuator_index), byte(control), timeout[3], timeout[2], timeout[1], timeout[0]};
        return new ScienceModuleCommand(command_word, operands);
    }

    ScienceModuleCommand GetCommand_QuerySpeedController(int actuator_index)
    {
        byte command_word = GetCommandWord("query_speed_controller");
        return new ScienceModuleCommand(command_word, new byte[]{byte(actuator_index)});
    }

    ScienceModuleCommand GetCommand_ClearSpeedController(int actuator_index)
    {
        byte command_word = GetCommandWord("clear_speed_controller");
        return new ScienceModuleCommand(command_word, new byte[]{byte(actuator_index)});
    }

    ScienceModuleCommand GetCommand_QueryIsActuatorReserved(int actuator_index)
    {
        byte command_word = GetCommandWord("query_is_actuator_reserved");
        return new ScienceModuleCommand(command_word, new byte[]{byte(actuator_index)});
    }

    // Templates 

    ScienceModuleCommand_Template GetCommandTemplate_UpdateActuatorPosition(int sensor_index)
    {
        return new ScienceModuleCommand_Template(
            fm.GetCommand_UpdateActuatorPosition(sensor_index, 0)
        ).AddField(1, 1); // Fill in the new position
    }

    ScienceModuleCommand_Template GetCommandTemplate_UpdateActuatorControl(int sensor_index)
    {
        return new ScienceModuleCommand_Template(
            fm.GetCommand_UpdateActuatorControl(sensor_index, 0)
        ).AddField(1, 1); // Fill in the new control
    }

    ScienceModuleCommand_Template GetCommandTemplate_SubmitPositionalControl(int sensor_index)
    {
        return new ScienceModuleCommand_Template(fm.GetCommand_SubmitPositionalControl(sensor_index, 0, 0))
        .AddField(1, 1) // Fill in the position
        .AddField(2, 1); // Fill in the speed
    }

    ScienceModuleCommand_Template GetCommandTemplate_SubmitSpeedControl(int sensor_index)
    {
        return new ScienceModuleCommand_Template(fm.GetCommand_SubmitSpeedControl(sensor_index, 0, 0))
        .AddField(1, 1) // Fill in the control
        .AddField(2, 4); // Fill in the timeout
    }
}

class ScienceModuleCommand
{
    byte command_word;
    byte[] operands;

    ScienceModuleCommand(byte command_word)
    {
        this.command_word = command_word;
        this.operands = new byte[0];
    }

    ScienceModuleCommand(byte command_word, byte[] operands)
    {
        this.command_word = command_word;
        this.operands = operands;
    }

    ScienceModuleCommand(ScienceModuleCommand command)
    {
        this.command_word = command.command_word;
        this.operands = command.operands.clone();
    }

    ScienceModuleCommand SetOperands(byte[] new_ops)
    {
        this.operands = new_ops;
        return this;
    }
}

class ScienceModuleCommand_Template extends ScienceModuleCommand
{
    ArrayList<Integer> field_indices = new ArrayList<>();
    ArrayList<Integer> field_sizes = new ArrayList<>();
    int total_size = 0;

    ScienceModuleCommand_Template(byte command_word)
    {
        super(command_word);
    }

    ScienceModuleCommand_Template(byte command_word, byte[] operands)
    {
        super(command_word, operands);
    }

    ScienceModuleCommand_Template(ScienceModuleCommand command)
    {
        super(command);
    }

    ScienceModuleCommand_Template AddField(int index, int size)
    {
        if (index + size > this.operands.length)
        {
            throw new IllegalArgumentException("Error: this field doesn't fit in this template. Index: " + index + ", Size: " + size + ", Operands Length: " + this.operands.length);
        }
        field_indices.add(index);
        field_sizes.add(size);
        total_size += size;
        
        return this;
    }

    ScienceModuleCommand FillFields(byte[] ops)
    {
        if (ops.length != total_size)
        {
            println("Error: Provided data length (" + ops.length + ") does not match the total size of the template fields (" + total_size + ")");
            return null;
        }

        ScienceModuleCommand new_command = new ScienceModuleCommand(this);

        int tracer = 0;
        for (int i = 0; i < field_indices.size(); i++)
        {
            int field_index = field_indices.get(i);
            int field_size = field_sizes.get(i);
            arrayCopy(ops, tracer, new_command.operands, field_index, field_size);
            tracer += field_size;
        }

        return new_command;
    }
}
