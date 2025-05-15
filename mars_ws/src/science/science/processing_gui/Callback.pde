interface Callback
{
    void receiveModuleResponse(int error_code, byte[] response);
    byte[] getCallbackSignature();
}