class AsciiHex
{
  Textarea textareaAscii;
  Textarea textareaHex;
  Textlabel label;
  int text_size = 10;
  
  AsciiHex(String callback)
  {
    textareaAscii = cp5.addTextarea(callback + "_ascii")
      .setFont(createFont("", 10))
      .setLineHeight(14)
      .setColor(color(255))
      .setColorBackground(color(10, 100))
      .setColorForeground(color(255, 100));
      
    textareaHex = cp5.addTextarea(callback + "_hex")
      .setFont(createFont("", 10))
      .setLineHeight(14)
      .setColor(color(255))
      .setColorBackground(color(10, 100))
      .setColorForeground(color(255, 100));
      
    label = cp5.addLabel(callback + "_label")
      .setText(callback);
  }
  
  void send_byte(byte b)
  {
    
    if (textareaAscii.getText().length() == 0 && (char(b) == ' ' || b <= byte(0x0C)))
        textareaAscii.append(">" + char(b) + " "); // fix a really strange crash with a first char space
    else
    {
        textareaAscii.append(char(b) + " ");
    }
      
    textareaHex.append(hex(b) + " ");
    
    // Sanitize input
    // Slows down on the largest windows
    // Catches weird crashes
    String str = textareaAscii.getText();
    int buffer_size = min(50, str.length());
    String begin = str.substring(0, str.length() - buffer_size);
    String s = str.substring(str.length() - buffer_size);
    s = s.replaceAll("\r", "\n");    // Use just one type of new line
    s = s.replaceAll("\n +", "\n");  // Case 1
    s = s.replaceAll(" +\n", "\n");  // Case 2
    textareaAscii.setText(begin + s);
  }
  
  void clear()
  {
    textareaAscii.clear();
    textareaHex.clear();
  }
  
  AsciiHex resize(float x, float y, int w, int h)
  {
    textareaAscii.setPosition(x, y);
    textareaAscii.setSize(w/2, h);
    textareaHex.setPosition(x + w/2 + 1, y);
    textareaHex.setSize(w/2, h);
    
    label.setPosition(x, y - text_size);
    label.setSize(w, text_size);
    
    return this;
  }
  
  void addToTab(String tabName)
  {
    textareaAscii.moveTo(tabName);
    textareaHex.moveTo(tabName);
    label.moveTo(tabName);
  }
}

class ConsoleWindow extends AsciiHex
{
  Button clearButton;
  
  ConsoleWindow(String callback)
  {
    super(callback);
    
    print("making clear button");
    clearButton = cp5.addButton(callback + "_clear")
      .setValue(0)
      .setSize(100,19)
      .setLabel("Clear")
      .plugTo(this, "clear");
  }
  
  ConsoleWindow resize(float x, float y, int w, int h)
  {
    super.resize(x, y, w, h);
    clearButton.setPosition(x, y + h);
    return this;
  }
  
  void addToTab(String tabName)
  {
    super.addToTab(tabName);
    clearButton.moveTo(tabName);
  }
}
