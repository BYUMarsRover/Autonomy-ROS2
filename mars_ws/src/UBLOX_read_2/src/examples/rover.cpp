#include <iostream>
#include <fstream>
#include <signal.h>

#include "UBLOX/ublox.h"

using namespace std;

int i = 1;

bool stop = false;
void inthand(int signum)
{
    stop = true;
}

void relposned_callback(uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg, uint8_t f9pID=0)
{
    int RTK_flag;
    const ublox::NAV_RELPOSNED_t& msg(in_msg.NAV_RELPOSNED);
    if (msg.flags.diffSoln)
    {
        printf("RTK");
        RTK_flag = 1;
    }
    else
    {
        printf("No RTK \n");
        RTK_flag = 0;
        i++;
    }
    if (RTK_flag == 1)
    {
        if (msg.flags.isMoving)
            printf(", Moving Base");
        if (msg.flags.floatCarrSoln)
            printf(" , Floating \n");
        else if (msg.flags.fixedCarrSoln)
            printf(" , Fixed \n");
        if (msg.flags.relPosValid)
            printf("valid relative position components \n");
        printf("%d relative t: %d, NED: %d, %d, %d, Distance: %d\n",
               i,
               msg.iTow/1000,
               msg.relPosN, msg.relPosE, msg.relPosD, msg.relPosLength);
        // ofstream myfile;
        // myfile.open ("../textfiles/9test_3ft_little_move/data.txt", ios::app);
        // myfile <<  msg.iTow
        //        << " " << msg.relPosN << " " << msg.relPosE << " " << msg.relPosD
        //        << " " << msg.relPosLength << " \n";
        // myfile.close();
        i++;
    }
    fflush(stdout); // Will now print everything in the stdout buffer
}

int main(int argc, char** argv)
{
    // Create a UBLOX instance

    std::string port = "/dev/ttyACM1";
    int message_rate = 10;
    if(argc > 1)
        port = argv[1];
    ublox::UBLOX ublox(port, message_rate);
    ublox.initRover("localhost", 16145, "localhost", 16140);

    // look for Ctrl+C and quit
    signal(SIGINT, inthand);

    // Connect a callback to the RELPOSNED message
    ublox.registerUBXCallback(ublox::CLASS_NAV, ublox::NAV_RELPOSNED, &relposned_callback);

    while (!stop)
    {
    }

    std::cout << "\nquitting" << std::endl;
    return 0;
}
