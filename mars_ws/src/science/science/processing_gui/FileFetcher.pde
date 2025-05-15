class SendFileButton
{
    Button send;
    String filename;
    
    SendFileButton(String tabName, String filename)
    {
        this.filename = filename;
        send = cp5.addButton(filename + "_fetchfile")
            .setValue(0)
            .setSize(180,19)
            .setLabel(filename)
            .moveTo(tabName)
            .plugTo(this, "fetchFile");
    }
    
    void fetchFile()
    {
        byte[] b = loadBytes("packets/" + this.filename);
        if (b == null)
        {
        println("File does not exist");
        return;
        }
        println("Loaded file " + this.filename);
        send_serial_bytes(b);
    }
}

class FileFetcherManager extends Container
{
    ArrayList<SendFileButton> buttons;
    ArrayList<String> filenames;
    String tabName;
    String lookupfile;
    Container currentColumn;
    
    FileFetcherManager(String tabName, String lookupfile)
    {
        super(true, 0, 5);
        this.tabName = tabName;
        this.lookupfile = lookupfile;
        filenames = new ArrayList<String>();
        currentColumn = new Container(false, 0, 5);
        add_container(currentColumn);
        
        loadLookupFile();
    }

    void loadLookupFile()
    {
        String[] names = loadStrings(lookupfile);
        if (names == null)
        {
            print("Failed to open file " + lookupfile);
            return;
        }
        println("Opened file " + lookupfile + " found " + names.length + " entries");
        for (String s : names) 
        {
            createNewFileButton(s);
        }
    }
    
    void createNewFileButton(String filename)
    {
        if (filenames.contains(filename)) return;

        SendFileButton ff = new SendFileButton(tabName, filename);
        currentColumn.add_container(new ControllerBlock<Button>(ff.send));

        if (currentColumn.elements.size() > 20) {
            Container c = new Container(false, 0, 5);
            add_container(c);
            currentColumn = c;
            println("Added new column");
        }

        filenames.add(filename);
        saveFilenames();
    }

    void createPacketFile(String byteString, String filename)
    { 
        // Get the byte field string
        if (filenames.contains(filename)) return;

        ArrayList<String> codes = new ArrayList<String>();

        for (int i = 0; i < byteString.length(); i+=2)
        {
            while (byteString.charAt(i) == ' ') i++;
            codes.add(byteString.substring(i, i+2));
        }

        byte hex_array[] = new byte[codes.size()];
        for (int i = 0; i < hex_array.length; i++)
        {
            hex_array[i] = byte(unhex(codes.get(i)));
        }

        saveBytes("packets/" + filename, hex_array);
        createNewFileButton(filename);
    }

    void saveFilenames()
    {
        Object[] names = filenames.toArray();
        String[] stringArray = Arrays.copyOf(names, names.length, String[].class);
        saveStrings(lookupfile, stringArray);
        println("Saved " + stringArray.length + " entries to file " + lookupfile);
    }
}
