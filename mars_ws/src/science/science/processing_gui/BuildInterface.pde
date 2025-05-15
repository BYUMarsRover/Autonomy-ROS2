void buildEnvironment()
{
    Tab manualTab = cp5.addTab("manualTab")
        .setLabel("Manual")
        .activateEvent(true)
        .setId(0)
        ;
  
    cp5.getTab("actionTab")
        .setLabel("Actions")
        .activateEvent(true)
        .setId(1)
        ;

    cp5.getTab("controllerTab")
        .setLabel("Controllers")
        .activateEvent(true)
        .setId(2)
        ;

    cp5.getTab("routineTab")
        .setLabel("Routines")
        .activateEvent(true)
        .setId(3)
        ;

    cp5.getTab("curveTab")
        .setLabel("Curves")
        .activateEvent(true)
        .setId(4)
        ;
        
    visualTab = cp5.getTab("visualTab")
        .setLabel("Visual")
        .activateEvent(true)
        .setId(5)
        ;

    visualTab = cp5.getTab("i2cTab")
        .setLabel("I2C")
        .activateEvent(true)
        .setId(6)
        ;
        
    // Create Serial Windows
    serialTx = new ConsoleWindow("serialTx").resize(width*0.6 + 10, 30, int((width - 10) - (width*0.6 + 10)), 200);
    serialRx = new ConsoleWindow("serialRx").resize(width*0.6 + 10, 270, int((width - 10) - (width*0.6 + 10)), 200);
    serialTx.addToTab("global");
    serialRx.addToTab("global");
  
    // Port Select
    portList = cp5.addScrollableList("portSelect")
        .setPosition(width - 210, 0)
        .setSize(200, 100)
        .setBarHeight(20)
        .setItemHeight(20)
        .setItems(Serial.list())
        .setValue(0)
        .setType(ScrollableList.DROPDOWN) // currently supported DROPDOWN and LIST
        .bringToFront()
        .setLabel("Choose Port")
        .moveTo("global")
        ;
    
    cp5.addButton("disconnectPort")
        .setPosition(width - 300, 5)
        .setSize(80, 12)
        .bringToFront()
        .setColorBackground(#FFFF0000)
        .moveTo("global")
        ;

    manualTab.bringToFront();
}

void buildManualPage()
{
    Container cont = new Container(false, 5, 20);
    cont.update_position(new Point2(0, 20));

    ManualController manualController =
        new ManualController("manualTab");

    cont.add_container(manualController);
}

void buildRoutineManager()
{
    RoutineManager routineController =
        new RoutineManager("Routine Controller", "routineTab", new String[]{"Reset All Actuators", "Move All Actators to Zero", "Test Routine", "Align First Cache"});
    routineController.update_position(new Point2(0, 20));
}

void buildCurvePage()
{
    // Analog Curve Stuff
    Container curveCont = new Container(false, 5, 20);
    curveCont.update_position(new Point2(0, 20));

    AnalogCurveManipulator tempCurve = new AnalogCurveManipulator("Temperature Curve", "curveTab", byte(0), 0, 100, "C");
    curveCont.add_container(tempCurve.init());

    AnalogCurveManipulator humCurve = new AnalogCurveManipulator("Humidity Curve", "curveTab", byte(1), 0, 100, "%");
    curveCont.add_container(humCurve.init());

    SpectrographManipulator specCurve = new SpectrographManipulator("Spectrometer", "curveTab", 0, 1000, "Counts");
    curveCont.add_container(specCurve.init());

    UVManipulator uvCurve = new UVManipulator("UV Sensor", "curveTab", "");
    curveCont.add_container(uvCurve.init());
}

void buildVisualGui()
{
    // Ack and Override Toggles
    Toggle visualAckToggle = cp5.addToggle("visual_ack_toggle")
        .setSize(20,20)
        .setPosition(280,100)
        .setLabel("ACK")
        .moveTo("visualTab")
        ;

    Toggle visualOverrideToggle = cp5.addToggle("visual_override_toggle")
        .setSize(20,20)
        .setPosition(305,100)
        .setLabel("OVR")
        .moveTo("visualTab")
        ;

    // Buttons in visual GUI
    LinearActuatorController probeControl =
        new LinearActuatorController("Probe Actuator", "visualTab", PROBE_ACTUATOR_INDEX, pollingManager.getPoll("probe_control"), visualAckToggle, visualOverrideToggle);
    probeControl.update_position(new Point2(500, 270));

    LinearActuatorController augerControl =
        new LinearActuatorController("Auger Actuator", "visualTab", AUGER_ACTUATOR_INDEX, pollingManager.getPoll("auger_control"), visualAckToggle, visualOverrideToggle);
    augerControl.update_position(new Point2(150, 150));

    LinearActuatorController primaryDoorControl =
        new LinearActuatorController("Primary Cache Door", "visualTab", PRIMARY_DOOR_ACTUATOR_INDEX, pollingManager.getPoll("primary_door_control"), visualAckToggle, visualOverrideToggle);
    primaryDoorControl.update_position(new Point2(150, 250));

    DrillActuatorController drillControl =
        new DrillActuatorController("Drill Control", "visualTab", DRILL_ACTUATOR_INDEX, pollingManager.getPoll("drill_control"), visualAckToggle, visualOverrideToggle);
    drillControl.update_position(new Point2(250, 250));

    LinearActuatorController cacheControl =
        new LinearActuatorController("Cache Actuator", "visualTab", SECONDARY_CACHE_ACTUATOR_INDEX, pollingManager.getPoll("secondary_cache_control"), visualAckToggle, visualOverrideToggle);
    cacheControl.update_position(new Point2(250, 550));

    LinearActuatorController secondaryDoorControl =
        new LinearActuatorController("Secondary Cache Door", "visualTab", SECONDARY_DOOR_ACTUATOR_INDEX, pollingManager.getPoll("secondary_door_control"), visualAckToggle, visualOverrideToggle);
    secondaryDoorControl.update_position(new Point2(350, 550));
}

void buildManipulatorPage()
{
    Container manipCont = new Container(false, 5, 20);
    manipCont.update_position(new Point2(0, 20));
    
    manipCont.add_container(new ManipulatorActuatorPosition("Probe Position", "actionTab", PROBE_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorPosition("Auger Position", "actionTab", AUGER_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorPosition("Primary Cache Door Position", "actionTab", PRIMARY_DOOR_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorPosition("Secondary Cache Door Position", "actionTab", SECONDARY_DOOR_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorPosition("Secondary Cache Position", "actionTab", SECONDARY_CACHE_ACTUATOR_INDEX).init());

    manipCont.add_container(new ManipulatorActuatorControl("Probe Control", "actionTab", PROBE_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorControl("Auger Control", "actionTab", AUGER_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorControl("Primary Cache Door Control", "actionTab", PRIMARY_DOOR_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorControl("Secondary Cache Door Control", "actionTab", SECONDARY_DOOR_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorControl("Secondary Cache Control", "actionTab", SECONDARY_CACHE_ACTUATOR_INDEX).init());
    manipCont.add_container(new ManipulatorActuatorControl("Drill Control", "actionTab", DRILL_ACTUATOR_INDEX).init());

}

void buildControllerPage()
{
    Container controllerCont = new Container(false, 5, 20);
    controllerCont.update_position(new Point2(0, 20));

    controllerCont.add_container(new ManipulatorPosController("Probe Positional Controller", "controllerTab", PROBE_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorPosController("Auger Positional Controller", "controllerTab", AUGER_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorPosController("Primary Cache Door Positional Controller", "controllerTab", PRIMARY_DOOR_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorPosController("Secondary Cache Door Positional Controller", "controllerTab", SECONDARY_DOOR_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorPosController("Secondary Cache Positional Controller", "controllerTab", SECONDARY_CACHE_ACTUATOR_INDEX).init());

    controllerCont.add_container(new ManipulatorSpeedController("Probe Speed Controller", "controllerTab", PROBE_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorSpeedController("Auger Speed Controller", "controllerTab", AUGER_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorSpeedController("Primary Cache Door Speed Controller", "controllerTab", PRIMARY_DOOR_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorSpeedController("Secondary Cache Door Speed Controller", "controllerTab", SECONDARY_DOOR_ACTUATOR_INDEX).init());
    controllerCont.add_container(new ManipulatorSpeedController("Secondary Cache Speed Controller", "controllerTab", SECONDARY_CACHE_ACTUATOR_INDEX).init());
}

void buildI2CPage()
{
    Container i2cCont = new Container(false, 5, 20);
    i2cCont.update_position(new Point2(0, 20));

    // TODO add stuff here
}
