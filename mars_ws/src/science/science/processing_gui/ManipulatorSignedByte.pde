class ManipulatorSignedByte extends ManipulatorUnsignedByte
{
    ManipulatorSignedByte(String labelName, String tabName, ScienceModuleCommand_Template send_template, ScienceModuleCommand query_command)
    {
        super(labelName, tabName, send_template, query_command);
        byteSlider.slider.setRange(-128, 127);
        byteSlider.entryField.setLabel("Enter -128, 127]");
    }
}

class ManipulatorActuatorControl extends ManipulatorSignedByte
{
    ManipulatorActuatorControl(String labelName, String tabName, int sensor_index)
    {
        super(
            labelName,
            tabName,
            fm.GetCommandTemplate_UpdateActuatorControl(sensor_index),
            fm.GetCommand_GetActuatorControl(sensor_index)
        );
    }
}
