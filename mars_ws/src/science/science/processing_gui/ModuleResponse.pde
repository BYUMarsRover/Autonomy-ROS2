class ModuleResponse
{
  Textlabel timeStampLabel;
  Textlabel timeStampValue;
  Textlabel packetLabel;
  Textlabel packetValue;
  Textlabel packetMatchValue;
  Textlabel errorCodeValue;
  Textlabel messageLengthValue;
  AsciiHex moduleResponseWindow;
  
  ModuleResponse()
  {
    timeStampLabel = cp5.addTextlabel("timeStampLabel")
        .setText("Timestamp:")
        .setPosition(width*0.6 + 10, 535)
        .moveTo("global");
        ;
      
    timeStampValue = cp5.addTextlabel("timeStampValue")
        .setText("--:--:--")
        .setPosition(width*0.6 + 90, 535)
        .setColorValue(0xffffff00)
        .moveTo("global");
        ;
        
    packetLabel = cp5.addTextlabel("receivedPacketLabel")
        .setText("Received Packet:")
        .setPosition(width*0.6 + 10, 550)
        .moveTo("global");
        ;
      
    packetValue = cp5.addTextlabel("receivedPacketValue")
        .setText("")
        .setPosition(width*0.6 + 90, 550)
        .setColorValue(0xffffff00)
        .moveTo("global");
        ;
        
    cp5.addTextlabel("packetMatchLabel")
        .setText("Echo'ed Packet Match:")
        .setPosition(width*0.6 + 10, 565)
        .moveTo("global");
        ;
      
    packetMatchValue = cp5.addTextlabel("packetMatchValue")
        .setText("--")
        .setPosition(width*0.6 + 112, 565)
        .setColorValue(0xffffff00)
        .moveTo("global");
        ;
    
    cp5.addTextlabel("errorCodeLabel")
        .setText("Returned Error Code:")
        .setPosition(width*0.6 + 160, 565)
        .moveTo("global");
        ;
      
    errorCodeValue = cp5.addTextlabel("errorCodeValue")
        .setText("-")
        .setPosition(width*0.6 + 252, 565)
        .setColorValue(0xffffff00)
        .moveTo("global");
        ;
        
    cp5.addTextlabel("messageLengthLabel")
        .setText("Message Length:")
        .setPosition(width*0.6 + 300, 565)
        .moveTo("global");
        ;
      
    messageLengthValue = cp5.addTextlabel("messageLengthValue")
        .setText("-")
        .setPosition(width*0.6 + 372, 565)
        .setColorValue(0xffffff00)
        .moveTo("global");
        ;
    
    moduleResponseWindow = new AsciiHex("moduleResponseWindow").resize(width*0.6 + 10, 590, int((width - 10) - (width*0.6 + 10)), 100);
    moduleResponseWindow.addToTab("global");
  }
  
  void updateErrorCode(int errorCode)
  {
    if (errorCode > 0) errorCodeValue.setText(Integer.toString(errorCode)).setColorValue(0xffff0000);
    else errorCodeValue.setText(Integer.toString(errorCode)).setColorValue(0xff00ff00);
  }
  
  void updateMessageLength(int length)
  {
    messageLengthValue.setText(Integer.toString(length));
  }
  
  void updatePacketMatch(boolean match)
  {
    if (match) packetMatchValue.setText("TRUE").setColorValue(0xff00ff00);
    else packetMatchValue.setText("FALSE").setColorValue(0xffff0000);;
  }
  
  void sendMessage(byte[] bytes)
  {
    moduleResponseWindow.clear();
    for (int i = 0; i < bytes.length; i++)
    {
      moduleResponseWindow.send_byte(bytes[i]);
    }
    updateMessageLength(bytes.length);
  }
  
  void sendReceivedPacket(byte[] bytes)
  {
    String str = "";
    int chperline = 0;
    int newLines = 0;
    for (int i = 0; i < bytes.length; i++)
    {
      str += hex(bytes[i]) + " ";
      chperline += 1;
      if (chperline > 23)
      {
        newLines += 1;
        if (newLines == 4)
        {
          str += "...";
          break;
        }
        str += "\n";
        chperline = 0;
      }
    }
    packetValue.setText(str);
    packetLabel.setPosition(width*0.6 + 10, 550 - (newLines * 6));
    packetValue.setPosition(width*0.6 + 90, 550 - (newLines * 6));
    timeStampLabel.setPosition(width*0.6 + 10, 535 - (newLines * 6));
    timeStampValue.setPosition(width*0.6 + 90, 535 - (newLines * 6));
  }
  
  void updateTimestamp()
  {
    String stamp = hour() + ":" + minute() + ":" + ((second() + (millis() % 1000)*0.001));
    timeStampValue.setText(stamp);
  }
}
