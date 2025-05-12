class ManipulatorUnsignedByte extends Manipulator
{
    SliderAndField byteSlider;

    ManipulatorUnsignedByte(String labelName, String tabName, ScienceModuleCommand_Template send_template, ScienceModuleCommand query_command)
    {
        super(labelName, tabName, send_template, query_command);

        ManipOperand sliderManip = new ManipOperand(1);
        byteSlider = new SliderAndField(labelName, tabName, 0, 255, sliderManip, "", 100, 100, 20);
        this.addFields(new ManipOperand[]{ sliderManip });
    }

    void add_custom_controllers()
    {
        ControllerBlock<Slider> sliderBlock = new ControllerBlock<Slider>(byteSlider.slider);
        row_2.add_container(sliderBlock);

        ControllerBlock<Textfield> entryBlock = new ControllerBlock<Textfield>(byteSlider.entryField);
        row_2.add_container(entryBlock);
    }

    void receiveModuleResponse(int error_code, byte[] response)
    {
        if (response.length == 1)
        {
            int value = int(response[0]);
            while (value < byteSlider.slider.getMin()) value += 256;
            while (value > byteSlider.slider.getMax()) value -= 256;
            byteSlider.slider.setValue(float(value));
            byteSlider.slider_update(value);
        }
    }

}

class ManipulatorActuatorPosition extends ManipulatorUnsignedByte
{
    ManipulatorActuatorPosition(String labelName, String tabName, int sensor_index)
    {
        super(
            labelName,
            tabName,
            fm.GetCommandTemplate_UpdateActuatorPosition(sensor_index),
            fm.GetCommand_GetActuatorPosition(sensor_index)
        );
    }
}
