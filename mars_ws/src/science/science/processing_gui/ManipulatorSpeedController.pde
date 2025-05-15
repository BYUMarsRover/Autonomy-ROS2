class ManipulatorSpeedController extends Manipulator
{
    SliderAndField speedSlider;
    SliderAndField timeSlider;
    Button resolveButton;
    ScienceModuleCommand resolve_command;

    ManipulatorSpeedController(String labelName, String tabName, ScienceModuleCommand_Template send_template, ScienceModuleCommand query_command, ScienceModuleCommand resolve_command)
    {
        super(labelName, tabName, send_template, query_command);
        this.resolve_command = resolve_command;

        ManipOperand speedManip = new ManipOperand(1);
        ManipOperand timeManip = new ManipOperand(4);

        speedSlider = new SliderAndField(labelName + "_speed", tabName, -128, 127, speedManip, "Speed", 60, 60, 20);
        timeSlider = new SliderAndField(labelName + "_timeout", tabName, 0, 30000, timeManip, "Timeout_ms", 60, 60, 20);

        speedSlider.slider.setValue(127);
        timeSlider.slider.setValue(3e3);

        this.addFields(new ManipOperand[]{ speedManip, timeManip });

        resolveButton = cp5.addButton(labelName + "_resolve")
            .setValue(0)
            .setSize(70,20)
            .setLabel("Resolve")
            .moveTo(tabName)
            .plugTo(this, "resolve");
            ;
    }

    ManipulatorSpeedController(String labelName, String tabName, int sensor_index)
    {
        this(
            labelName,
            tabName,
            fm.GetCommandTemplate_SubmitSpeedControl(sensor_index),
            fm.GetCommand_QuerySpeedController(sensor_index),
            fm.GetCommand_ClearSpeedController(sensor_index)
        );
    }

    void add_custom_controllers()
    {
        ControllerBlock<Slider> sliderBlock = new ControllerBlock<Slider>(speedSlider.slider);
        row_2.add_container(sliderBlock);

        ControllerBlock<Textfield> entryBlock = new ControllerBlock<Textfield>(speedSlider.entryField);
        row_2.add_container(entryBlock);

        ControllerBlock<Slider> sliderBlock2 = new ControllerBlock<Slider>(timeSlider.slider);
        row_2.add_container(sliderBlock2);

        ControllerBlock<Textfield> entryBlock2 = new ControllerBlock<Textfield>(timeSlider.entryField);
        row_2.add_container(entryBlock2);

        ControllerBlock<Button> resolveButtonBlock = new ControllerBlock<Button>(resolveButton);
        row_2.add_container(resolveButtonBlock);
    }

    void resolve()
    {
        byte[] b = sm_interface.author_packet(
            resolve_command,
            overrideBit(),
            ackBit()
        );
        
        lastSentCommand = b;
        parse.joinCallbackList(this);
        send_serial_bytes(b);
    }

    void receiveModuleResponse(int error_code, byte[] response)
    {
        println("Speed Controller Response not implemented");
        // if (response.length == 1)
        // {
        //     int value = int(response[0]);
        //     while (value < unsignedByteSlider.slider.getMin()) value += 256;
        //     while (value > unsignedByteSlider.slider.getMax()) value -= 256;
        //     unsignedByteSlider.slider.setValue(float(value));
        //     unsignedByteSlider.slider_update(value);
        // }
    }

}
