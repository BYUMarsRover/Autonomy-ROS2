import java.util.LinkedList;
import java.util.Queue;

class ResponseParser
{
    // New Design
    // Add a byte into the buffer
    // If its not the header byte it is ignored
    // If it is the header byte the next bytes are queued until
    // the entire possible packet can be read
    // If it not a valid packet, the queue is emptied from the front until
    // the header byte is found.

    LinkedList<Byte> buffer;

    byte headerByte = byte(0x52);
    byte footerByte = byte(0x46);

    byte[] echo;
    byte[] message;
    byte errorCode;

    ArrayList<Callback> callbackList;

    ResponseParser()
    {
        buffer = new LinkedList<Byte>();
        callbackList = new ArrayList<Callback>();
    }
    
    void consumeBuffer()
    {
      do {
          buffer.remove();
      } while (buffer.size() > 0 && buffer.peek() != headerByte);
    }

    // Returns true if the buffer contains a valid packet
    boolean observeByte(byte new_byte)
    {
        // Dont bother if this isnt the header and the buffer is empty
        if (buffer.size() == 0 && new_byte != headerByte) return false;
        else buffer.add(new_byte); // Add the new byte to the buffer

        while (true) // Checking to see if this is a complete packet
        {
            // Response Packet Structure
            // 0        - [HeaderResponse] 'R'
            // 1        - [EchoLength] 'l'
            // 2+l      - [ErrorCode] 
            // 3+l      - [LengthOfError] e
            // 4+l+e    - [FooterResponse] 'F'

            if (buffer.size() - 1 < 1) return false;
            // Echo Operand Length has not been reached
            int echo_length = buffer.get(1);
            if (echo_length < 0) {
                consumeBuffer();
                return false;
            }

            if (buffer.size() - 1 < 3 + echo_length) return false;
            // Error Message Length has not been reached
            int error_message_length = buffer.get(3 + echo_length);
            if (error_message_length < 0) {
                consumeBuffer();
                return false;
            }

            if (buffer.size() - 1 < 4 + echo_length + error_message_length) return false;
            // End of Message has not yet been reached

            if (((4 + echo_length + error_message_length) > 0) && ((4 + echo_length + error_message_length) < buffer.size()) && (buffer.get(4 + echo_length + error_message_length) == footerByte))
            {
                // Found a valid packet!
                processValidPacket(echo_length, error_message_length);
                return true;
            }
            else
            {
                // This packet is bad, consume until the next header
                do {
                    buffer.remove();
                } while (buffer.size() > 0 && buffer.peek() != headerByte);
                if (buffer.size() == 0)
                {
                    // Buffer was emptied, cease the search
                    return false;
                }
            }
        }
    }

    void processValidPacket(int echo_length, int error_message_length)
    {
        // Process the command echo
        echo = new byte[echo_length];
        for (int i = 0; i < echo_length; i++) echo[i] = buffer.get(2 + i);

        // Log the Error Code
        errorCode = buffer.get(2 + echo_length);

        // Process the message array
        message = new byte[error_message_length];
        for (int i = 0; i < error_message_length; i++)
            message[i] = buffer.get(4 + echo_length + i);

        notifyCallback();
        buffer.clear();
    }

    void notifyCallback()
    {
        Callback matchCallback = null;
        for (Callback callback : callbackList)
        {
            if (Arrays.equals(echo, callback.getCallbackSignature()))
            {
                matchCallback = callback;
                callback.receiveModuleResponse(errorCode, message);
            }
        }
        // callbackList.size() > 0 Attempt to fix a strange crash
        if (matchCallback != null && callbackList.size() > 0) callbackList.remove(matchCallback);
    }

    void joinCallbackList(Callback callback)
    {
        if (!callbackList.contains(callback))
            callbackList.add(callback);
    }
}
