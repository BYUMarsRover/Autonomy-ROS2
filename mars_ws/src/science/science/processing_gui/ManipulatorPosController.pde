class ManipulatorPosController extends Manipulator
{
    SliderAndField positionSlider;
    SliderAndField speedSlider;
    Button resolveButton;
    ScienceModuleCommand resolve_command;

    ManipulatorPosController(String labelName, String tabName, ScienceModuleCommand_Template send_template, ScienceModuleCommand query_command, ScienceModuleCommand resolve_command)
    {
        super(labelName, tabName, send_template, query_command);
        this.resolve_command = resolve_command;

        ManipOperand posManip = new ManipOperand(1);
        ManipOperand speedManip = new ManipOperand(1);

        positionSlider = new SliderAndField(labelName + "_pos", tabName, 0, 255, posManip, "Position", 60, 60, 20);
        speedSlider = new SliderAndField(labelName + "_speed", tabName, 0, 255, speedManip, "Speed", 60, 60, 20);
        speedSlider.slider.setValue(255);

        this.addFields(new ManipOperand[]{ posManip, speedManip });

        resolveButton = cp5.addButton(labelName + "_resolve")
            .setValue(0)
            .setSize(70,20)
            .setLabel("Resolve")
            .moveTo(tabName)
            .plugTo(this, "resolve");
            ;
    }

    ManipulatorPosController(String labelName, String tabName, int sensor_index)
    {
        this(
            labelName,
            tabName,
            fm.GetCommandTemplate_SubmitPositionalControl(sensor_index),
            fm.GetCommand_QueryPositionalController(sensor_index),
            fm.GetCommand_ClearPositionalController(sensor_index)
        );
    }

    void add_custom_controllers()
    {
        ControllerBlock<Slider> sliderBlock = new ControllerBlock<Slider>(positionSlider.slider);
        row_2.add_container(sliderBlock);

        ControllerBlock<Textfield> entryBlock = new ControllerBlock<Textfield>(positionSlider.entryField);
        row_2.add_container(entryBlock);

        ControllerBlock<Slider> sliderBlock2 = new ControllerBlock<Slider>(speedSlider.slider);
        row_2.add_container(sliderBlock2);

        ControllerBlock<Textfield> entryBlock2 = new ControllerBlock<Textfield>(speedSlider.entryField);
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
        println("Pos Controller Response not implemented");
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
