import java.nio.ByteBuffer;

class AnalogCurveManipulator extends Container implements Callback
{
    Textlabel curveLabel;
    Button fetchRawValueButton;
    Button fetchCurvedValueButton;
    Textfield testField;
    Textlabel curveOutLabel;

    Textfield coeff0Field;
    Textfield coeff1Field;
    Textfield coeff2Field;
    Textfield coeff3Field;
    Textfield coeff4Field;
    Textfield coeff5Field;

    Button submitCurveButton;
    Button clearCurveButton;
    Button fetchCurveButton;

    Container row_1;
    Container row_2;
    Container buttonCont1;
    Container coeffCont1;
    Container coeffCont2;
    Container buttonCont2;

    Chart curveChart;
    String units;

    // 0 - Temp
    // 1 - Humidity
    byte sensor_index;

    String callbackCode = "none";

    AnalogCurveManipulator(String curveName, String tabName, byte sensor_index, int min, int max, String units)
    {
        super(false, 5, 10);

        this.sensor_index = sensor_index;
        this.units = units;

        // Build Primary Containers
        row_1 = new Container(true, 0, 2);
        row_2 = new Container(true, 0, 5);

        // Build Secondary Containers
        buttonCont1 = new Container(false, 0, 15);
        coeffCont1 = new Container(false, 0, 15);
        coeffCont2 = new Container(false, 0, 15);
        buttonCont2 = new Container(false, 0, 15);
        
        curveLabel = cp5.addTextlabel(curveName + "_label")
            .setText(curveName)
            .moveTo(tabName)
            .setColorValue(0xffffff00)
            ;

        testField = cp5.addTextfield(curveName + "_testField")
            .setSize(100,20)
            .setLabel("Test Field")
            .moveTo(tabName)
            .plugTo(this, "testCurve")
            ;

        curveOutLabel = cp5.addTextlabel(curveName + "_curveOutLabel")
            .setText("Output: ")
            .moveTo(tabName)
            .setColorValue(0xffffff00)
            ;
      
        fetchRawValueButton = cp5.addButton(curveName + "_fetchRawButton")
            .setValue(0)
            .setSize(90,20)
            .setLabel("Fetch Raw Value")
            .moveTo(tabName)
            .plugTo(this, "fetchRawValue")
            ;

        fetchCurvedValueButton = cp5.addButton(curveName + "_fetchCurvedValueButton")
            .setValue(0)
            .setSize(90,20)
            .setLabel("Fetch Curved Value")
            .moveTo(tabName)
            .plugTo(this, "fetchCurvedValue")
            ;

        submitCurveButton = cp5.addButton(curveName + "_submitCurveButton")
            .setValue(0)
            .setSize(80,20)
            .setLabel("Submit Curve")
            .moveTo(tabName)
            .plugTo(this, "submitCurve")
            ;

        clearCurveButton = cp5.addButton(curveName + "_clearCurveButton")
            .setValue(0)
            .setSize(80,20)
            .setLabel("Clear Curve")
            .moveTo(tabName)
            .plugTo(this, "clearCurve")
            ;
        
        fetchCurveButton = cp5.addButton(curveName + "_fetchCurveButton")
            .setValue(0)
            .setSize(100,20)
            .setLabel("Fetch Curve Coeffs")
            .moveTo(tabName)
            .plugTo(this, "fetchCurve")
            ;

        coeff0Field = cp5.addTextfield(curveName + "_coeff0Field")
            .setSize(100,20)
            .setLabel("Coeff 0")
            .moveTo(tabName)
            .setValue(str(min))
            .setAutoClear(false)
            .plugTo(this, "generateCurveChartWrapper")
            ;

        coeff1Field = cp5.addTextfield(curveName + "_coeff1Field")
            .setSize(100,20)
            .setLabel("Coeff 1")
            .moveTo(tabName)
            .setValue(str(max - min))
            .setAutoClear(false)
            .plugTo(this, "generateCurveChartWrapper")
            ;

        coeff2Field = cp5.addTextfield(curveName + "_coeff2Field")
            .setSize(100,20)
            .setLabel("Coeff 2")
            .moveTo(tabName)
            .setValue("0.0")
            .setAutoClear(false)
            .plugTo(this, "generateCurveChartWrapper")
            ;

        coeff3Field = cp5.addTextfield(curveName + "_coeff3Field")
            .setSize(100,20)
            .setLabel("Coeff 3")
            .moveTo(tabName)
            .setValue("0.0")
            .setAutoClear(false)
            .plugTo(this, "generateCurveChartWrapper")
            ;

        coeff4Field = cp5.addTextfield(curveName + "_coeff4Field")
            .setSize(100,20)
            .setLabel("Coeff 4")
            .moveTo(tabName)
            .setValue("0.0")
            .setAutoClear(false)
            .plugTo(this, "generateCurveChartWrapper")
            ;

        coeff5Field = cp5.addTextfield(curveName + "_coeff5Field")
            .setSize(100,20)
            .setLabel("Coeff 5")
            .moveTo(tabName)
            .setValue("0.0")
            .setAutoClear(false)
            .plugTo(this, "generateCurveChartWrapper")
            ;

        curveChart = cp5.addChart(curveName + "_curveChart")
            .setSize(200, 100)
            .setRange(min, max)
            .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
            .setStrokeWeight(1.5)
            .setColorCaptionLabel(color(40))
            .moveTo(tabName)
            .addDataSet("curvePoints")
            .setData("curvePoints", new float[100])
            ;
    }

    Container init()
    {
        // Curve Label
        Container labelBlock = new ControllerBlock<Textlabel>(curveLabel);
        labelBlock.update_size(new Point2(100,5)); // Can't update textlabel directly for some reason
        row_1.add_container(labelBlock);

        // First Column of Controllers
        buttonCont1.add_container(new ControllerBlock<Button>(fetchRawValueButton));
        buttonCont1.add_container(new ControllerBlock<Button>(fetchCurvedValueButton));
        buttonCont1.add_container(new ControllerBlock<Textfield>(testField));

        // Out label
        Container labelBlock2 = new ControllerBlock<Textlabel>(curveOutLabel);
        labelBlock2.update_size(new Point2(100,5));
        buttonCont1.add_container(labelBlock2);

        // Second Column of Controllers
        coeffCont1.add_container(new ControllerBlock<Textfield>(coeff0Field));
        coeffCont1.add_container(new ControllerBlock<Textfield>(coeff1Field));
        coeffCont1.add_container(new ControllerBlock<Textfield>(coeff2Field));

        // Third Column of Controllers
        coeffCont2.add_container(new ControllerBlock<Textfield>(coeff3Field));
        coeffCont2.add_container(new ControllerBlock<Textfield>(coeff4Field));
        coeffCont2.add_container(new ControllerBlock<Textfield>(coeff5Field));

        // Fourth Column of Controllers
        buttonCont2.add_container(new ControllerBlock<Button>(submitCurveButton));
        buttonCont2.add_container(new ControllerBlock<Button>(clearCurveButton));
        buttonCont2.add_container(new ControllerBlock<Button>(fetchCurveButton));

        row_2.add_container(buttonCont1);
        row_2.add_container(coeffCont1);
        row_2.add_container(coeffCont2);
        row_2.add_container(buttonCont2);

        // Add Chart
        row_2.add_container(new ControllerBlock<Chart>(curveChart));

        this.add_container(row_1);
        this.add_container(row_2);
        
        generateCurveChart();
        
        return this;
    }

    byte[] getFetchRawValueBytes()
    {
        return sm_interface.author_packet(fm.GetCommand_GetAnalogSensorRaw(sensor_index));
    }

    void fetchRawValue()
    {`
        callbackCode = "fetchRawValue";
        parse.joinCallbackList(this);
        send_serial_bytes(getFetchRawValueBytes());
    }

    byte[] getFetchCalculatedValueBytes()
    {
        return sm_interface.author_packet(fm.GetCommand_GetAnalogSensorCalibrated(sensor_index));
    }

    void fetchCurvedValue()
    {
        callbackCode = "fetchCurvedValue";
        parse.joinCallbackList(this);
        send_serial_bytes(getFetchCalculatedValueBytes());
    }

    void submitCurve()
    {
        generateCurveChart();
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_SubmitAnalogCoefficients(sensor_index, getCoefficients())
            )
        );
    }

    void clearCurve()
    {
        send_serial_bytes(
            sm_interface.author_packet(
                fm.GetCommand_ClearAnalogCalibration(sensor_index)
            )
        );
    }

    byte[] getFetchCurveBytes()
    {
        return sm_interface.author_packet(fm.GetCommand_GetCalibrationData(sensor_index));
    }

    void fetchCurve()
    {
        callbackCode = "fetchCurve";
        parse.joinCallbackList(this);
        send_serial_bytes(getFetchCurveBytes());
    }

    void generateCurveChartWrapper(String v)
    {
        generateCurveChart();
    }

    void generateCurveChart()
    {
        float[] coeffs = getCoefficients();
        
        int curve_size = 100;
        float[] curvePoints = new float[curve_size];
        for (int i = 0; i < curve_size; i++)
        {
            float result = calculateCurve(float(i) / (curve_size - 1), coeffs);
            curvePoints[i] = result;
        }
        curveChart.setData(curvePoints);
    }

    void writeOutputFloat(float outNumber)
    {
        curveOutLabel.setText("Output: " + nf(outNumber, 0, 2) + units);
    }

    void writeOutputAnalog16Bit(int out)
    {
        curveOutLabel.setText("Output: " + str(out));
    }

    void testCurve(String numberString)
    {
        float outResult;
        if (float(numberString) <= 1)
        {
            outResult = calculateCurve(float(numberString), getCoefficients());
        }
        else
        {
            outResult = calculateCurve(max(min((float(numberString) / 1023.0), 1.0), 0.0), getCoefficients());
        }
        writeOutputFloat(outResult);
    }

    Textfield[] getCoeffFields() {
        Textfield[] coefficientArray = 
            {
                coeff0Field,
                coeff1Field,
                coeff2Field,
                coeff3Field,
                coeff4Field,
                coeff5Field
            };
        return coefficientArray;
    }

    float[] getCoefficients()
    {
        float[] coeffs = new float[6];
        Textfield[] coefficientArray = getCoeffFields();
        for (int j = 0; j < 6; j++)
        {
            coeffs[j] = float(coefficientArray[j].getText());
        }
        return coeffs;
    }

    float calculateCurve(float input, float[] coeffs)
    {
        float result = 0.0;
        for (int i = 0; i < 6; i++)
        {
            result += coeffs[i] * pow(input, i);
        }
        return result;
    }

    byte[] getCallbackSignature()
    {
        if (callbackCode == "fetchRawValue")
        {
            return getFetchRawValueBytes();
        }
        else if (callbackCode == "fetchCurvedValue")
        {
            return getFetchCalculatedValueBytes();
        }
        else if (callbackCode == "fetchCurve")
        {
            return getFetchCurveBytes();
        }
        else return new byte[0];
    } 

    void receiveModuleResponse(int error_code, byte[] response)
    {
        if (callbackCode == "fetchRawValue")
        {
            byte[] b = new byte[] {0, 0, response[1], response[0]};
            int analogOut = ByteBuffer.wrap(b).getInt();
            writeOutputAnalog16Bit(analogOut);
        }
        else if (callbackCode == "fetchCurvedValue")
        {
            byte[] b = new byte[] {response[3], response[2], response[1], response[0]};
            float calculated = ByteBuffer.wrap(b).getFloat();
            writeOutputFloat(calculated);
        }
        else if (callbackCode == "fetchCurve")
        {
            int float_cnt = response.length / 4;
            Textfield[] getCoeffFields = getCoeffFields();
            for (int i = 0; i < 6; i++)
            {
                if (i < float_cnt) {
                    byte[] b = new byte[] {response[i*4 + 3], response[i*4 + 2], response[i*4 + 1], response[i*4 + 0]};
                    getCoeffFields[i].setText(nf(ByteBuffer.wrap(b).getFloat(), 0, 2));
                } else {
                    getCoeffFields[i].setText(nf(0.0, 0, 2));
                }
            }
            generateCurveChart();
        }
        else return;
    }
}
