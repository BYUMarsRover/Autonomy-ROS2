class ManualController extends Container
{
  
    Textfield filenameField;
    FileFetcherManager fileFetcherManager;
    Textfield bytefieldLabel;
  
    ManualController(String tabName)
    {
        super(false, 0, 20);

        Container row_1 = new Container(true);
        add_container(row_1);

        bytefieldLabel = cp5.addTextfield("byteField")
            .setSize(200,20)
            .setColor(color(255,0,0))
            .setLabel("Send Hex Bytes")
            .setAutoClear(true)
            .plugTo(this, "byteField")
            .moveTo(tabName)
            ;
        row_1.add_container(new ControllerBlock<Textfield>(bytefieldLabel));
        
        Button createPacketFile = cp5.addButton("createPacketFile")
            .setValue(0)
            .setSize(100,20)
            .setLabel("Create Packet File")
            .moveTo(tabName)
            .plugTo(this, "createNewPacketFile")
            ;
        row_1.add_container(new ControllerBlock<Button>(createPacketFile));

        Textfield asciiField = cp5.addTextfield("asciiField")
            .setSize(200,20)
            .setColor(color(255,0,0))
            .setLabel("Send Ascii")
            .setAutoClear(true)
            .plugTo(this, "asciiField")
            .moveTo(tabName)
            ;
        add_container(new ControllerBlock<Textfield>(asciiField));

        Container row_3 = new Container(true);
        add_container(row_3);

        filenameField = cp5.addTextfield("fileNameEntry")
            .setSize(200, 20)
            .setColor(color(255,0,0))
            .setLabel("File Name")
            .setAutoClear(false)
            .moveTo(tabName)
            ;
        row_3.add_container(new ControllerBlock<Textfield>(filenameField));
        
        Button submitFile = cp5.addButton("submitFile")
            .setValue(0)
            .setSize(40,20)
            .setLabel("Fetch")
            .moveTo(tabName)
            .plugTo(this, "submitFile")
            ;
        row_3.add_container(new ControllerBlock<Button>(submitFile));

        fileFetcherManager = new FileFetcherManager(tabName, fileFetcherFilename);
        add_container(fileFetcherManager);
    }

    void createNewPacketFile()
    {
        String filename = filenameField.getText();
        String byteString = bytefieldLabel.getText();
        fileFetcherManager.createPacketFile(byteString, filename);
    }

    void submitFile()
    {
        String filename = filenameField.getText();
        send_file_bytes("packets/" + filename);
        
        if (!fileFetcherManager.filenames.contains(filename))
        {
            println("Creating new file...");
            fileFetcherManager.createNewFileButton(filename);
        }
    }

    void byteField(String byteString)
    {
        // Identify all hex codes
        ArrayList<String> byteCodes = new ArrayList<String>();
        while(byteString.length() > 0)
        {
            while (byteString.charAt(0) == ' ') byteString = byteString.substring(1);
            String b = byteString.substring(0,2);
            byteString = byteString.substring(2);
            byteCodes.add(b);
        }

        // Send hex bytes
        byte[] buffer = new byte[byteCodes.size()];
        for (int i = 0; i < byteCodes.size(); i++) buffer[i] = byte(unhex(byteCodes.get(i)));
        send_serial_bytes(buffer);

    }

    void asciiField(String byteString)
    {
        byte[] buffer = new byte[byteString.length()];
        for (int i = 0; i < byteString.length(); i++)
        {
            buffer[i] = byte(byteString.charAt(i));
        }
        send_serial_bytes(buffer);
    }
}
