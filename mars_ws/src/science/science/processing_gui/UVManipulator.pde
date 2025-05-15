import java.nio.ByteBuffer;

class UVManipulator extends Container implements Callback
{
    Textlabel UVLabel;
    Textlabel latestUVReadingField;
    Textlabel latestLuxReadingField;
    Textfield nameField;

    Button takeSampleButton;
    Button retrieveDataButton;
    Button resetButton;
    Button saveFileButton;

    Toggle autoPollUV;

    Container itemCont;
    Container col_1;
    Container col_2;
    Container reading_row;
    Chart uvChart;
    Chart alsChart;

    Poll uvPoll;
    boolean polling_uv = false;

    int uvPlotMax = 1;
    int uvPlotMin = 0;
    int alsPlotMax = 1;
    int alsPlotMin = 0;

    UVManipulator(String UVName, String tabName, String units)
    {
        super(false, 5, 15);

        // Contains line chart and buttons
        itemCont = new Container(true);

        // Contains buttons
        col_1 = new Container(false, 0, 20);
        col_2 = new Container(false, 0, 20);
        reading_row = new Container(true, 0, 110);

        ////////////////////////////

        UVLabel = cp5.addTextlabel(UVName + "_label")
           .setText(UVName)
           .moveTo(tabName)
           .setColorValue(0xffffff00)
           ;

        latestUVReadingField = cp5.addTextlabel(UVName + "_uv_reading_label")
           .setText("UV Index: Null")
           .moveTo(tabName)
           .setColorValue(0xffffff00)
           ;

        latestLuxReadingField = cp5.addTextlabel(UVName + "_lux_reading_label")
           .setText("Lux: Null")
           .moveTo(tabName)
           .setColorValue(0xffffff00)
           ;

        ////////////////////////////

        takeSampleButton = cp5.addButton(UVName + "_takeSampleButton")
           .setValue(0)
           .setSize(90,20)
           .setLabel("Take Sample")
           .moveTo(tabName)
           .plugTo(this, "takeSample")
           ;

        retrieveDataButton = cp5.addButton(UVName + "_retrieveDataButton")
           .setValue(0)
           .setSize(90,20)
           .setLabel("Retrieve Data")
           .moveTo(tabName)
           .plugTo(this, "retrieveData")
           ;

        resetButton = cp5.addButton(UVName + "_resetButton")
           .setValue(0)
           .setSize(90,20)
           .setLabel("Reset")
           .moveTo(tabName)
           .plugTo(this, "reset")
           ;
        
        saveFileButton = cp5.addButton(UVName + "_saveFileButton")
           .setValue(0)
           .setSize(90,20)
           .setLabel("Save File")
           .moveTo(tabName)
           .plugTo(this, "saveFile")
           ;

        ////////////////////////////

        nameField = cp5.addTextfield(UVName + "_nameField")
            .setSize(100,20)
            .setLabel("File name")
            .moveTo(tabName)
            .setAutoClear(true)
            .plugTo(this, "saveFileWrapper")
            ;

        ////////////////////////////

        autoPollUV = cp5.addToggle(UVName + "_autoPollUV")
            .setSize(20,20)
            .setLabel("Poll")
            .moveTo(tabName)
            .plugTo(this, "setPolling")
            ;
        
        ////////////////////////////

        uvChart = cp5.addChart(UVName + "_uvChart")
            .setSize(200, 100)
            .setRange(0, uvPlotMax)
            .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
            .setStrokeWeight(1.5)
            .setColorCaptionLabel(color(40))
            .moveTo(tabName)
            .addDataSet("indexValues")
            .setColors("indexValues", color(255, 0, 255))
            ;

        alsChart = cp5.addChart(UVName + "_alsChart")
            .setSize(200, 100)
            .setRange(0, alsPlotMax)
            .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
            .setStrokeWeight(1.5)
            .setColorCaptionLabel(color(40))
            .moveTo(tabName)
            .addDataSet("ambientValues")
            .setColors("ambientValues", color(0, 255, 255))
            ;
    }

    void setPolling(boolean val)
    {
        polling_uv = val;
    }

    void clearData()
    {
        uvChart.setData("indexValues", new float[0]);
        alsChart.setData("ambientValues", new float[0]);
    }

    void reset()
    {
        send_serial_bytes(sm_interface.author_packet(fm.GetCommand_ResetLTR()));
        clearData();
        setPolling(false);
    }

    void saveFileWrapper(String v)
    {
        saveFile();
    }

    void saveFile() 
    {
        PrintWriter output = createWriter("uv_readings/" + nameField.getText());
        output.println("uvindex, als"); //Header

        float[] values1 = uvChart.getDataSet("indexValues").getValues();
        float[] values2 = alsChart.getDataSet("ambientValues").getValues();
        for (int i = 0; i < values1.length; i++)
        {
           output.print(values1[i]);
           output.print(", ");
           output.println(values2[i]);
        }
        output.flush();
        output.close();
    }

    byte[] getTakeSampleBytes()
    {
        return sm_interface.author_packet(fm.GetCommand_SampleLTR());
    }

    byte[] getRetrieveDataBytes()
    {
        return sm_interface.author_packet(fm.GetCommand_ReturnLTRData());
    }

    void takeSample()
    {
        send_serial_bytes(getTakeSampleBytes());
        setPolling(true);
    }

    void retrieveData()
    {
        send_serial_bytes(getRetrieveDataBytes());
        parse.joinCallbackList(this);
    }

    Container init()
    {
        // Curve Label
        Container labelBlock = new ControllerBlock<Textlabel>(UVLabel);
        labelBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        this.add_container(labelBlock);

        // First Column of Controllers
        col_1.add_container(new ControllerBlock<Textfield>(nameField));
        col_1.add_container(new ControllerBlock<Button>(saveFileButton));
        col_1.add_container(new ControllerBlock<Button>(resetButton));

        // Second Column of Controllers
        col_2.add_container(new ControllerBlock<Toggle>(autoPollUV));
        col_2.add_container(new ControllerBlock<Button>(takeSampleButton));
        col_2.add_container(new ControllerBlock<Button>(retrieveDataButton));

        // Add Chart and Buttons to big cont
        itemCont.add_container(new ControllerBlock<Chart>(uvChart));
        itemCont.add_container(new ControllerBlock<Chart>(alsChart));
        itemCont.add_container(col_1);
        itemCont.add_container(col_2);
        this.add_container(itemCont);

        // Bottom row of readings
        Container latestUVReadingFieldBlock = new ControllerBlock<Textlabel>(latestUVReadingField);
        latestUVReadingFieldBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        reading_row.add_container(latestUVReadingFieldBlock);

        Container latestLuxReadingFieldBlock = new ControllerBlock<Textlabel>(latestLuxReadingField);
        latestLuxReadingFieldBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        reading_row.add_container(latestLuxReadingFieldBlock);
        
        this.add_container(reading_row);

        // Add poll
        uvPoll = new UVDataPoll(this);
        pollingManager.addPoll(uvPoll, "UV_data");
            
        return this;
    }

    byte[] getCallbackSignature()
    {
        return getRetrieveDataBytes();
    } 

    void receiveModuleResponse(int error_code, byte[] response)
    {
        println("Received data of size " + response.length + " for UV sensor with error code: " + error_code);
        if (error_code != 0 || response.length != 8) return;
        println("Plotting...");
        
        // UV Chart
        byte[] b = new byte[] {response[3], response[2], response[1], response[0]};
        float temp = ByteBuffer.wrap(b).getFloat();
        latestUVReadingField.setText("UV Index: " + str(temp));
        uvChart.addData("indexValues", temp);

        if (uvChart.getDataSet("indexValues").getValues().length == 1) {
            uvPlotMin = (int) temp - 1;
            uvPlotMax = (int) temp + 1;
        }
        if (temp > uvPlotMax) uvPlotMax = (int) temp + 1;
        if (temp < uvPlotMin) uvPlotMin = (int) temp - 1;
        if (uvPlotMin < 0) uvPlotMin = 0;
        uvChart.setRange(uvPlotMin, uvPlotMax);
        
        // ALS Chart
        b = new byte[] {response[7], response[6], response[5], response[4]};
        temp = ByteBuffer.wrap(b).getFloat();
        latestLuxReadingField.setText("Lux: " + str(temp));
        alsChart.addData("ambientValues", temp);

        if (alsChart.getDataSet("ambientValues").getValues().length == 1) {
            alsPlotMin = (int) temp - 1;
            alsPlotMax = (int) temp + 1;
        }
        if (temp > alsPlotMax) alsPlotMax = (int) temp + 1;
        if (temp < alsPlotMin) alsPlotMin = (int) temp - 1;
        if (alsPlotMin < 0) alsPlotMin = 0;
        alsChart.setRange(alsPlotMin, alsPlotMax);

        // Set Polling false unless
        //the autopoll toggle is on
        if (autoPollUV.getValue() > 0) takeSample();
        else setPolling(false);
    }
}
