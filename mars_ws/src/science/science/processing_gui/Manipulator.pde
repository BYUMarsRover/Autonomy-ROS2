abstract class Manipulator extends Container implements Callback
{
    Textlabel label;
    Button sendButton;
    Button queryButton;
    Toggle ackToggle;
    Toggle overrideToggle;

    ScienceModuleCommand_Template send_template;
    ScienceModuleCommand query_command;
    
    Container row_1;
    Container row_2;

    byte[] lastSentCommand;
    byte[] prefixOperands = null;
    ManipOperand[] fields;

    protected class ManipOperand
    {
        byte[] fieldValue;
        int byte_count;

        ManipOperand(int byte_count)
        {
            this.byte_count = byte_count;
            fieldValue = new byte[byte_count];
        }

        void updateFieldValue(byte[] newValue)
        {
            if (newValue.length != byte_count)
            {
              println("WARN: fieldValue was attempted to be update with incorrectly sized data");
              return;
            }
            this.fieldValue = newValue;
        }
    }

    protected class SliderAndField
    {
        Textfield entryField;
        Slider slider;
        ManipOperand manipLink;

        SliderAndField(String labelName, String tabName, int min, int max, ManipOperand manipLink, String displayName, int sliderWidth, int entryWidth, int height)
        {
            slider = cp5.addSlider(labelName + "_slider")
                .setSize(sliderWidth, height)
                .setRange(min, max)
                .setLabel(displayName)
                .plugTo(this, "slider_call")
                .moveTo(tabName)
                ;

            slider.getCaptionLabel().align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE);

            entryField = cp5.addTextfield(labelName + "_textField")
                .setSize(entryWidth, height)
                .setAutoClear(true)
                .moveTo(tabName)
                .plugTo(this, "manual_entry")
                .setLabel("Enter [" + str(min) + ", " + str(max) +"]")
                ;
            
            this.manipLink = manipLink;
        }

        void manual_entry(String text)
        {
            int value = int(text);
            slider.setValue(value);
            send();
        }

        void slider_call(float newVal)
        {
            slider_update(int(newVal));
        }

        void slider_update(int slider_val)
        {
            byte[] newFieldValue = new byte[manipLink.byte_count];

            // Make a buffer
            ByteBuffer buffer = ByteBuffer.allocate(4);
            // Set the byte order to little-endian
            buffer.order(java.nio.ByteOrder.LITTLE_ENDIAN);
            // Get the byte of the int
            byte[] sliderBytes = buffer.putInt(slider_val).array();

            arrayCopy(sliderBytes, 0, newFieldValue, 0, manipLink.byte_count);
            manipLink.updateFieldValue(newFieldValue);
        }

    }

    Manipulator(String labelName, String tabName, ScienceModuleCommand_Template send_template, ScienceModuleCommand query_command)
    {
        super(false, 0, 5);
        this.send_template = send_template;
        this.query_command = query_command;

        row_1 = new Container(true, 5, 5);
        row_2 = new Container(true);
        add_container(row_1);
        add_container(row_2);
        
        label = cp5.addTextlabel(labelName + "_label")
            .setText(labelName)
            .moveTo(tabName)
            .setColorValue(0xffffff00)
            ;
      
        sendButton = cp5.addButton(labelName + "_send")
            .setValue(0)
            .setSize(70,20)
            .setLabel("Submit")
            .moveTo(tabName)
            .plugTo(this, "send");
            ;

        queryButton = cp5.addButton(labelName + "_query")
            .setValue(0)
            .setSize(70,20)
            .setLabel("Query")
            .moveTo(tabName)
            .plugTo(this, "query");
            ;

        ackToggle = cp5.addToggle(labelName + "_ack_toggle")
            .setSize(20,20)
            .setLabel("ACK")
            .moveTo(tabName)
            ;

        overrideToggle = cp5.addToggle(labelName + "_override_toggle")
            .setSize(20,20)
            .setLabel("OVR")
            .moveTo(tabName)
            ;
    }

    Container addFields(ManipOperand[] fields)
    {
        this.fields = fields;
        return this;
    }

    Container init()
    {
        ControllerBlock<Textlabel> labelBlock = new ControllerBlock<Textlabel>(label);
        labelBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        ControllerBlock<Button> buttonBlock1 = new ControllerBlock<Button>(sendButton);
        ControllerBlock<Button> buttonBlock2 = new ControllerBlock<Button>(queryButton);
        ControllerBlock<Toggle> toggleBlock = new ControllerBlock<Toggle>(ackToggle);
        ControllerBlock<Toggle> toggleBlock2 = new ControllerBlock<Toggle>(overrideToggle);

        row_1.add_container(labelBlock);
        row_2.add_container(toggleBlock2);
        row_2.add_container(toggleBlock);

        add_custom_controllers();

        row_2.add_container(buttonBlock1);
        row_2.add_container(buttonBlock2);
        
        return this;
    }
    
    abstract void add_custom_controllers();

    boolean ackBit()
    {
        return ackToggle.getValue() > 0;
    }

    boolean overrideBit()
    {
        return overrideToggle.getValue() > 0;
    }

    byte[] getOperands()
    {
        int totalLength = 0;
        for (ManipOperand field : fields) {
            totalLength += field.byte_count;
        }

        byte[] operands = new byte[totalLength];
        int offset = 0;

        for (ManipOperand field : fields) {
            arrayCopy(field.fieldValue, 0, operands, offset, field.byte_count);
            offset += field.byte_count;
        }

        return operands;
    }

    void send()
    {
        byte[] b = sm_interface.author_packet(
            send_template.FillFields(getOperands()),
            overrideBit(),
            ackBit()
        );
        lastSentCommand = b;
        send_serial_bytes(b);
    }

    void query()
    {
        byte[] b = sm_interface.author_packet(
            query_command,
            overrideBit(),
            ackBit()
        );

        lastSentCommand = b;
        parse.joinCallbackList(this);
        send_serial_bytes(b);
    }

    abstract void receiveModuleResponse(int error_code, byte[] response);

    byte[] getCallbackSignature()
    {
        return lastSentCommand;
    }
}
