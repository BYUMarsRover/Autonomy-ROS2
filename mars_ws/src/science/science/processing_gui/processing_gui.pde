
import controlP5.*;
ControlP5 cp5;
import processing.serial.*;
import java.util.Arrays;

color tabBackground = color(200);
color moduleResponseBackground = color(100);

Serial myPort;
int baud_rate = 9600;

ConsoleWindow serialTx, serialRx;
ModuleResponse moduleResponse;
ScrollableList portList;

ModuleResponse mres;
ResponseParser parse;

byte[] lastSentMessage;

String fileFetcherFilename = "fetchFiles.txt";

Tab visualTab;
int activeTab;

ScadaView scadaView;
PollingManager pollingManager;

FunctionMap fm;
ScienceModuleInterface sm_interface;

void setup() {
    size(1100,700);
    noStroke();
    cp5 = new ControlP5(this);
    cp5.getTab("default").remove();

    fm = new FunctionMap();
    sm_interface = new ScienceModuleInterface();
    parse = new ResponseParser();
    pollingManager = new PollingManager();
    scadaView = new ScadaView(pollingManager);

    buildEnvironment();
    buildManualPage();

    mres = new ModuleResponse();

    buildManipulatorPage();
    buildControllerPage();
    buildCurvePage();
    buildVisualGui();
    buildRoutineManager();
    buildI2CPage();
}

void draw() {
    background(#000000);
    
    // Draw the Tab area
    fill(tabBackground);
    rect(0, 16, width * 0.6, height);
    
    // Draw the module Response Area
    fill(moduleResponseBackground);
    rect(width * 0.6, 16, width, height);
    
    if (activeTab == 5)
    {
        scadaView.tick();
        scadaView.draw();
    }

    pollingManager.tick();
    
    read_serial_port();
}

void controlEvent(ControlEvent theControlEvent) {
    if (theControlEvent.isTab()) {
        activeTab = theControlEvent.getTab().getId();
        parse.callbackList.clear();
    }
}
