
int PROBE_ACTUATOR_INDEX = 0;
int AUGER_ACTUATOR_INDEX = 1;
int PRIMARY_DOOR_ACTUATOR_INDEX = 2;
int SECONDARY_DOOR_ACTUATOR_INDEX = 3;
int SECONDARY_CACHE_ACTUATOR_INDEX = 4;
int DRILL_ACTUATOR_INDEX = 5;

class LinearActuatorController extends Container
{
    Textlabel actuatorLabel;
    Button retractButton;
    Button extendButton;
    Poll poll;

    byte retract_speed = byte(-127);
    byte extend_speed = byte(127);
    int actuator_index;

    Toggle ackToggleRef;
    Toggle overrideToggleRef;

    LinearActuatorController(String actuatorName, String tabName, int actuator_index, Poll poll, Toggle ackToggleRef, Toggle overrideToggleRef)
    {
        super(false, 5, 10);
        this.actuator_index = actuator_index;
        this.poll = poll;

        this.ackToggleRef = ackToggleRef;
        this.overrideToggleRef = overrideToggleRef;
        
        actuatorLabel = cp5.addTextlabel(actuatorName + "_lac_label")
            .setText(actuatorName)
            .moveTo(tabName)
            .setColorValue(0xffffff00)
            ;
      
        retractButton = cp5.addButton(actuatorName + "_retractButton")
            .setValue(0)
            .setSize(80,20)
            .setLabel("Retract")
            .moveTo(tabName)
            ;

        retractButton.addCallback(
            new CallbackListener() {
                public void controlEvent(CallbackEvent theEvent) {
                    switch(theEvent.getAction()) {
                        case(ControlP5.ACTION_PRESSED): retract(); break;
                        case(ControlP5.ACTION_RELEASED): stop(); break;
                    }
                }
            }
            );

        extendButton = cp5.addButton(actuatorName + "_extendButton")
            .setValue(0)
            .setSize(80,20)
            .setLabel("Extend")
            .moveTo(tabName)
            ;

        extendButton.addCallback(
            new CallbackListener() {
                public void controlEvent(CallbackEvent theEvent) {
                    switch(theEvent.getAction()) {
                        case(ControlP5.ACTION_PRESSED): extend(); break;
                        case(ControlP5.ACTION_RELEASED): stop(); break;
                    }
                }
            }
            );

        Container labelBlock = new ControllerBlock<Textlabel>(actuatorLabel);
        labelBlock.update_size(new Point2(100,5));
        add_container(labelBlock);

        add_container(new ControllerBlock<Button>(retractButton));
        add_container(new ControllerBlock<Button>(extendButton));
    }

    void extend()
    {
        println("EXTEND LA");
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_UpdateActuatorControl(actuator_index, extend_speed),
                getOverrideBit(),
                getAckBit()
            )
        );
        poll.value = extend_speed;
    }

    void retract()
    {
        println("RETRACT LA");
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_UpdateActuatorControl(actuator_index, retract_speed),
                getOverrideBit(),
                getAckBit()
            )
        );
        poll.value = retract_speed;
    }

    void stop()
    {
        println("STOP LA");
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_UpdateActuatorControl(actuator_index, 0),
                getOverrideBit(),
                getAckBit()
            )
        );
        poll.value = 0;
    }

    boolean getAckBit()
    {
        return (ackToggleRef != null && ackToggleRef.getValue() > 0);
    }

    boolean getOverrideBit()
    {
        return (overrideToggleRef != null && overrideToggleRef.getValue() > 0);
    }

}
