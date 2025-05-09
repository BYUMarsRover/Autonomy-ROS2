void read_serial_port()
{
    // Read from the Serial Port
    if (myPort == null) return;
    byte[] b = myPort.readBytes(256);
    if (b != null)
    {
        // println(b.length, myPort.available());
        for (int i = 0; i < b.length; i++)
        {
            serialRx.send_byte(b[i]);
            if (parse.observeByte(b[i]))
            {
                // Response Packet Receieved
                mres.updateTimestamp();
                mres.sendMessage(parse.message);
                mres.sendReceivedPacket(parse.echo);
                mres.updateErrorCode(parse.errorCode);

                // Echo Match
                mres.updatePacketMatch(Arrays.equals(parse.echo, lastSentMessage));
            }
        }
    }
}

void portSelect(int n)
{
    portList.setItems(Serial.list());
    Object portString = cp5.get(ScrollableList.class, "portSelect").getItem(n).get("name");
    myPort = new Serial(this, (String)portString, baud_rate);
    println("Selected Port: " + portString);
}

void disconnectPort(int n)
{
    portList.setLabel("Choose Port");
    myPort.stop();
    myPort = null;
    println("Disconnected from Port");
}

void send_file_bytes(String filename)
{
    byte[] b = loadBytes(filename);
    if (b == null)
    {
        println("File does not exist");
        return;
    }
    println("Loaded file " + filename);
    send_serial_bytes(b);
}

void send_serial_bytes(byte[] buffer)
{
    lastSentMessage = buffer;
    for (int i = 0; i < buffer.length; i++) send_serial_byte(buffer[i]);
}

void send_serial_byte(byte b)
{
    serialTx.send_byte(b);
    if (myPort == null) return;
    myPort.write(b);
}
