import java.nio.ByteBuffer;

class SpectrographManipulator extends Container implements Callback
{
    Textlabel spectrographLabel;

    // Textlabel sampNumLabel;
    Textfield sampNumField;
    // Textlabel durationLabel;
    Textfield durationField;
    Textfield nameField;

    Button bulbOnButton;
    Button bulbOffButton;
    Button takeSampleButton;
    Button retrieveDataButton;
    Button resetButton;
    Button saveFileButton;

    Toggle bulbToggle;
    Toggle autoSaveToggle;

    Container itemCont;
    Container col_1;
    Container col_2;
    Container col_3;
    Chart barChart;

    int minChannelValue;
    int maxChannelValue;

    int default_duration = 1000000;
    int default_num_samples = 1;

    Poll spectroPoll;
    boolean awaiting_data = false;

    int[] wavelengths = new int[]{410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 960};

    SpectrographManipulator(String spectroName, String tabName, int min, int max, String units)
    {
        super(false, 5, 10);

        this.minChannelValue = min;
        this.maxChannelValue = max;

        // Contains Bar chart and buttons
        itemCont = new Container(true);

        // Contains buttons
        col_1 = new Container(false, 0, 20);
        col_2 = new Container(false, 0, 20);
        col_3 = new Container(false, 0, 20);

        ////////////////////////////

        spectrographLabel = cp5.addTextlabel(spectroName + "_label")
            .setText(spectroName)
            .moveTo(tabName)
            .setColorValue(0xffffff00)
            ;

        ////////////////////////////
      
        // bulbOnButton = cp5.addButton(spectroName + "_bulbOn")
        //     .setValue(0)
        //     .setSize(90,20)
        //     .setLabel("Turn Bulb On")
        //     .moveTo(tabName)
        //     .plugTo(this, "bulbOn")
        //     ;

        // bulbOffButton = cp5.addButton(spectroName + "_bulbOff")
        //     .setValue(0)
        //     .setSize(90,20)
        //     .setLabel("Turn Bulb Off")
        //     .moveTo(tabName)
        //     .plugTo(this, "bulbOff")
        //     ;

        takeSampleButton = cp5.addButton(spectroName + "_takeSampleButton")
            .setValue(0)
            .setSize(90,20)
            .setLabel("Take Sample")
            .moveTo(tabName)
            .plugTo(this, "takeSample")
            ;

        retrieveDataButton = cp5.addButton(spectroName + "_retrieveDataButton")
            .setValue(0)
            .setSize(90,20)
            .setLabel("Retrieve Data")
            .moveTo(tabName)
            .plugTo(this, "retrieveData")
            ;

        resetButton = cp5.addButton(spectroName + "_resetButton")
            .setValue(0)
            .setSize(90,20)
            .setLabel("Reset")
            .moveTo(tabName)
            .plugTo(this, "reset")
            ;
        
        saveFileButton = cp5.addButton(spectroName + "_saveFileButton")
            .setValue(0)
            .setSize(90,20)
            .setLabel("Save File")
            .moveTo(tabName)
            .plugTo(this, "saveFile")
            ;

        ////////////////////////////

        sampNumField = cp5.addTextfield(spectroName + "_sampNumField")
            .setSize(100,20)
            .setLabel("# samples")
            .moveTo(tabName)
            .setValue(str(default_num_samples))
            .setAutoClear(false)
            .plugTo(this, "sampNum")
            ;

        durationField = cp5.addTextfield(spectroName + "_durationField")
            .setSize(100,20)
            .setLabel("Duration")
            .moveTo(tabName)
            .setValue(str(default_duration))
            .setAutoClear(false)
            .plugTo(this, "duration")
            ;

        nameField = cp5.addTextfield(spectroName + "_nameField")
            .setSize(100,20)
            .setLabel("File name")
            .moveTo(tabName)
            .setAutoClear(true)
            .plugTo(this, "saveFileWrapper")
            ;

        ////////////////////////////

        bulbToggle = cp5.addToggle(spectroName + "_bulbToggle")
            .setSize(20,20)
            .setLabel("Bulb")
            .moveTo(tabName)
            ;

        autoSaveToggle = cp5.addToggle(spectroName + "_autoSaveToggle")
            .setSize(20,20)
            .setLabel("Autosave")
            .moveTo(tabName)
            ;
        
        ////////////////////////////

        barChart = cp5.addChart(spectroName + "_barChart")
            .setSize(200, 100)
            .setRange(min, max)
            .setView(Chart.BAR) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
            .setStrokeWeight(1.5)
            //.setColor(color(155,0,0))
            .setColorCaptionLabel(color(40))
            .moveTo(tabName)
            .addDataSet("channelValues")
            .setData("channelValues", new float[18])
            ;
    }

    void beginPollingForData()
    {
        awaiting_data = true;
        takeSampleButton.hide();
        retrieveDataButton.hide();
    }

    void stopPollingForData()
    {
        awaiting_data = false;
        takeSampleButton.show();
        retrieveDataButton.show();
    }

    void reset()
    {
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_ResetSpectrograph()
            )
        );
        stopPollingForData();
    }

    void saveFileWrapper(String v)
    {
        saveFile();
    }

    void saveFile() 
    {
        PrintWriter output = createWriter("spectrometer_readings/" + nameField.getText());
        float[] values = barChart.getDataSet("channelValues").getValues();

        output.println("wavelength, value"); //Header
        for (int i = 0; i < wavelengths.length; i++)
        {
            output.println(wavelengths[i] + ", " + values[i]);
        }
        output.flush();
        output.close();
    }

    byte[] getRetrieveDataBytes()
    {
        return sm_interface.author_packet(fm.GetCommand_ReturnSpectrographData());
    }

    void retrieveData()
    {
        send_serial_bytes(getRetrieveDataBytes());
        parse.joinCallbackList(this);
    }

    void takeSample()
    {
        int num_samples = int(sampNumField.getText());
        int duration = int(durationField.getText());
        int use_bulb = int(bulbToggle.getValue());
        
        // Instruct the device to sample
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_SampleSpectrograph(num_samples, duration, use_bulb > 0)
            )
        );
        
        // Start requesting the data back
        beginPollingForData();
    }

    Container init()
    {
        // Curve Label
        Container labelBlock = new ControllerBlock<Textlabel>(spectrographLabel);
        labelBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        this.add_container(labelBlock);

        // First Column of Controllers
        col_1.add_container(new ControllerBlock<Textfield>(sampNumField));
        col_1.add_container(new ControllerBlock<Textfield>(durationField));
        col_1.add_container(new ControllerBlock<Textfield>(nameField));

        // Second Column of Controllers
        col_2.add_container(new ControllerBlock<Button>(takeSampleButton));
        col_2.add_container(new ControllerBlock<Button>(retrieveDataButton));
        col_2.add_container(new ControllerBlock<Button>(saveFileButton));

        // Third Column of Controllers
        col_3.add_container(new ControllerBlock<Toggle>(bulbToggle));
        col_3.add_container(new ControllerBlock<Toggle>(autoSaveToggle));
        col_3.add_container(new ControllerBlock<Button>(resetButton));

        
        // Add Chart and Buttons to big cont
        itemCont.add_container(new ControllerBlock<Chart>(barChart));
        itemCont.add_container(col_1);
        itemCont.add_container(col_2);
        itemCont.add_container(col_3);
        this.add_container(itemCont);
        
        // Initialize chart
        generateBarChart();

        // Add poll
        spectroPoll = new SpectorgraphDataPoll(this);
        pollingManager.addPoll(spectroPoll, "spectrograph_data");
        
        return this;
    }

    void generateBarChart()
    {
        int channel_cnt = 18;
        float[] channelValues = new float[channel_cnt];
        for (int i = 0; i < channel_cnt; i++)
        {
            channelValues[i] = 0;
        }
        barChart.setData("channelValues", channelValues);
        color green = color(0, 255, 0);
        color red = color(255, 0, 0);
        color blue = color(0, 0, 255);
        barChart.setColors("channelValues",
            red, green, blue, red, green,
            blue, red, green, blue, red,
            green, blue, red, green, blue,
            red, green, blue
        );
    }

    byte[] getCallbackSignature()
    {
        return getRetrieveDataBytes();
    } 

    void receiveModuleResponse(int error_code, byte[] response)
    {
        println("Received data of size " + response.length + " for spectrograph with error code: " + error_code);
        if (error_code != 0) return;
        println("Plotting...");

        int channel_cnt = 18;
        float[] channelValues = new float[channel_cnt];
        for (int i = 0; i < channel_cnt; i++)
        {
            int index = i*4;
            byte[] b = new byte[] {response[index + 3], response[index + 2], response[index + 1], response[index]};
            float received = ByteBuffer.wrap(b).getFloat();
            channelValues[i] = received;
            println("Channel " + i + ": " + received);
        }
        barChart.setData(channelValues);

        if (autoSaveToggle.getValue() > 0 && (nameField.getText().length() > 0))
        {
            saveFile();
        }

        // Stop polling
        stopPollingForData();
    }
}
