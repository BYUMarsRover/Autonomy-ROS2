class DrillActuatorController extends LinearActuatorController
{
    DrillActuatorController(String actuatorName, String tabName, int actuator_index, Poll poll, Toggle ackToggleRef, Toggle overrideToggleRef)
    {
        super(actuatorName, tabName, actuator_index, poll, ackToggleRef, overrideToggleRef);
        retractButton.setLabel("Anti-Drillwise");
        extendButton.setLabel("Drillwise");
    }

}
