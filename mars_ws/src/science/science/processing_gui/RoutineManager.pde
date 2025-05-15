class RoutineManager extends Container implements Callback
{

    Container primary;
    Container row_1;

    Textlabel label;
    Toggle ackToggle;
    Toggle overrideToggle;

    Container row_2;
    Container row_3;

    Button pauseButton;
    Button queryButton;
    Button resumeButton;
    Button stepButton;
    Button abortButton;

    byte[] lastSentCommand;

    class RoutineButton
    {
        String name;
        int index;
        RoutineManager manager;
        Button button;

        RoutineButton(RoutineManager routineManager, String labelName, String tabName, String name, int index)
        {
            this.manager = routineManager;
            this.name = name;
            this.index = index;

            button = cp5.addButton(labelName + "_routinebutton_" + str(index))
                .setValue(0)
                .setSize(200,20)
                .setLabel("Begin " + name)
                .moveTo(tabName)
                .plugTo(this, "begin");
        }

        void begin()
        {
            byte[] b = sm_interface.author_packet(
                fm.GetCommand_RunRoutine(index),
                overrideBit(),
                ackBit()
            );
            manager.lastSentCommand = b;
            send_serial_bytes(b);
        }
    }

    RoutineManager(String labelName, String tabName, String[] routineNames)
    {
        super(false, 5, 5);

        row_1 = new Container(true, 10, 5);
        row_2 = new Container(false, 10, 5);
        row_3 = new Container(true, 10, 5);
        add_container(row_1);
        add_container(row_2);
        add_container(row_3);
        
        // Add Label
        label = cp5.addTextlabel(labelName + "_label")
            .setText(labelName)
            .moveTo(tabName)
            .setColorValue(0xffffff00)
            ;

        ControllerBlock<Textlabel> labelBlock = new ControllerBlock<Textlabel>(label);
        labelBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        row_1.add_container(labelBlock);

        // Add Toggles
        ackToggle = cp5.addToggle(labelName + "_ack_toggle")
            .setSize(20,20)
            .setLabel("ACK")
            .moveTo(tabName)
            ;
        row_1.add_container(new ControllerBlock<Toggle>(ackToggle));

        overrideToggle = cp5.addToggle(labelName + "_override_toggle")
            .setSize(20,20)
            .setLabel("OVR")
            .moveTo(tabName)
            ;
        row_1.add_container(new ControllerBlock<Toggle>(overrideToggle));

        // Add Routine buttons
        for (int i = 0; i < routineNames.length; i++)
        {
            RoutineButton rb = new RoutineButton(this, labelName, tabName, routineNames[i], i);
            row_2.add_container(new ControllerBlock<Button>(rb.button));
        }

        // Add Buttons
        pauseButton = cp5.addButton(labelName + "_pause")
            .setValue(0)
            .setSize(80,150)
            .setLabel("Pause")
            .moveTo(tabName)
            .plugTo(this, "pause");
        row_3.add_container(new ControllerBlock<Button>(pauseButton));

        queryButton = cp5.addButton(labelName + "_query")
            .setValue(0)
            .setSize(80,150)
            .setLabel("Query")
            .moveTo(tabName)
            .plugTo(this, "query");
        row_3.add_container(new ControllerBlock<Button>(queryButton));

        resumeButton = cp5.addButton(labelName + "_resume")
            .setValue(0)
            .setSize(80,150)
            .setLabel("Resume")
            .moveTo(tabName)
            .plugTo(this, "resume");
        row_3.add_container(new ControllerBlock<Button>(resumeButton));

        stepButton = cp5.addButton(labelName + "_step")
            .setValue(0)
            .setSize(80,150)
            .setLabel("Step")
            .moveTo(tabName)
            .plugTo(this, "step");
        row_3.add_container(new ControllerBlock<Button>(stepButton));

        abortButton = cp5.addButton(labelName + "_abort")
            .setValue(0)
            .setSize(80,150)
            .setLabel("Abort")
            .moveTo(tabName)
            .plugTo(this, "abort");
        row_3.add_container(new ControllerBlock<Button>(abortButton));

    }

    boolean ackBit()
    {
        return ackToggle.getValue() > 0;
    }

    boolean overrideBit()
    {
        return overrideToggle.getValue() > 0;
    }

    void pause() { send(sm_interface.author_packet(fm.GetCommand_PauseRoutine(), overrideBit(), ackBit())); }
    void resume() { send(sm_interface.author_packet(fm.GetCommand_ResumeRoutine(), overrideBit(), ackBit())); }
    void step() { send(sm_interface.author_packet(fm.GetCommand_StepRoutine(), overrideBit(), ackBit())); }
    void abort() { send(sm_interface.author_packet(fm.GetCommand_AbortRoutine(), overrideBit(), ackBit())); }
    void query() { send_query(sm_interface.author_packet(fm.GetCommand_QueryRoutineController(), overrideBit(), ackBit())); }

    void send(byte[] b)
    {
        lastSentCommand = b;
        send_serial_bytes(b);
    }

    void send_query(byte[] b)
    {
        parse.joinCallbackList(this);
        send(b);
    }

    void receiveModuleResponse(int error_code, byte[] response)
    {
        return;
    }

    byte[] getCallbackSignature()
    {
        return lastSentCommand;
    }
}
