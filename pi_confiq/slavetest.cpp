#include <pigpio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <cstring>

using namespace std;
const char *path0 = "/var/www/html/pro3e/webinterface/files/weight.txt";
const char *path1 = "/var/www/html/pro3e/webinterface/files/KP.txt";
const char *path2 = "/var/www/html/pro3e/webinterface/files/KI.txt";
const char *path3 = "/var/www/html/pro3e/webinterface/files/LOW.txt";
const char *path4 = "/var/www/html/pro3e/webinterface/files/HIGH.txt";
const char *path5 = "/var/www/html/pro3e/webinterface/files/TS.txt";
const char *path6 = "/var/www/html/pro3e/webinterface/files/IdleValue.txt";

void runSlave();
//void closeSlave();
int getControlBits(int, bool);

const int slaveAddress = 0x03; // <-- Your address of choice
bsc_xfer_t xfer;               // Struct to control data flow

int main()
{
    // Chose one of those two lines (comment the other out):
    runSlave();
    //closeSlave();

    return 0;
}

int32_t addup(int i)
{
    int32_t j;
    switch (i % 4)
    {
    case 0:
        j = xfer.rxBuf[i];
        j << 24;
        return j;
        break;
    case 1:
        j = xfer.rxBuf[i];
        j << 16;
        return j;
        break;
    case 2:
        j = xfer.rxBuf[i];
        j << 8;
        return j;
        break;
    case 3:
        j = xfer.rxBuf[i];
        return j;
        break;
    default:
        break;
    }
    return 0;
}
void output (int cnt,int32_t k)
{
    ofstream MyFile;
    switch (cnt)
    {
    case 0:
        
        MyFile.open(path0, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();
        break;
    case 1:
        //ofstream MyFile;
        MyFile.open(path1, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();
        break;
    case 2:
        //ofstream MyFile;
        MyFile.open(path2, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();
        break;
    case 3:
       // ofstream MyFile;
        MyFile.open(path3, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();
        break;
    case 4:
        //ofstream MyFile;
        MyFile.open(path5, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();
        break;
    case 5:
        //ofstream MyFile;
        MyFile.open(path5, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();

        break;
    case 6:
       // ofstream MyFile;
        MyFile.open(path6, ios::out | ios::trunc);
        MyFile << k;
        MyFile.close();

        break;

    default:
        break;
    }
}
void runSlave()
{
    gpioInitialise();
    cout << "Initialized GPIOs\n";
    // Close old device (if any)
    xfer.control = getControlBits(slaveAddress, false); // To avoid conflicts when restarting
    bscXfer(&xfer);
    // Set I2C slave Address to 0x0A
    xfer.control = getControlBits(slaveAddress, true);
    int status = bscXfer(&xfer); // Should now be visible in I2C-Scanners

    if (status >= 0)
    {
        cout << "Opened slave\n";
        xfer.rxCnt = 0;
        while (1)
        {
            bscXfer(&xfer);
            if (xfer.rxCnt > 0)
            {
                cout << "Received " << xfer.rxCnt << " bytes: ";

                int cnt = 0;
                for (int i = 0; i < (xfer.rxCnt - 1);)
                {
                    int32_t k = 0;

                    k += addup(i);
                    i++;
                    if (i % 4 == 0)
                    {
                      
                        output(cnt,k);
                        k = 0;
                        cnt++;
                    }

                    // MyFile << (int) xfer.rxBuf[i];
                    cout << xfer.rxBuf[i]; //used for testing
                }

                cout << "\n";
            }
            // sleep(2000);

            //if (xfer.rxCnt > 0){
            //    cout << xfer.rxBuf;
            //}
        }
    }
    else
        cout << "Failed to open slave!!!\n";
}

/*void writefile(char rxBuf){
    ofstream MyFile("i2c_output.txt");

  // Write to the file
  MyFile << rxBuf ;

  // Close the file
  MyFile.close();
}*/

/*void closeSlave() {
    gpioInitialise();
    cout << "Initialized GPIOs\n";

    xfer.control = getControlBits(slaveAddress, false);
    bscXfer(&xfer);
    cout << "Closed slave.\n";

    gpioTerminate();
    cout << "Terminated GPIOs.\n";
}*/

int getControlBits(int address /* max 127 */, bool open)
{
    /*
    Excerpt from http://abyz.me.uk/rpi/pigpio/cif.html#bscXfer regarding the control bits:

    22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    a  a  a  a  a  a  a  -  -  IT HC TF IR RE TE BK EC ES PL PH I2 SP EN

    Bits 0-13 are copied unchanged to the BSC CR register. See pages 163-165 of the Broadcom 
    peripherals document for full details. 

    aaaaaaa defines the I2C slave address (only relevant in I2C mode)
    IT  invert transmit status flags
    HC  enable host control
    TF  enable test FIFO
    IR  invert receive status flags
    RE  enable receive
    TE  enable transmit
    BK  abort operation and clear FIFOs
    EC  send control register as first I2C byte
    ES  send status register as first I2C byte
    PL  set SPI polarity high
    PH  set SPI phase high
    I2  enable I2C mode
    SP  enable SPI mode
    EN  enable BSC peripheral
    */

    // Flags like this: 0b/*IT:*/0/*HC:*/0/*TF:*/0/*IR:*/0/*RE:*/0/*TE:*/0/*BK:*/0/*EC:*/0/*ES:*/0/*PL:*/0/*PH:*/0/*I2:*/0/*SP:*/0/*EN:*/0;

    int flags;
    if (open)
        flags = /*RE:*/ (1 << 9) | /*TE:*/ (1 << 8) | /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);
    else // Close/Abort
        flags = /*BK:*/ (1 << 7) | /*I2:*/ (0 << 2) | /*EN:*/ (0 << 0);

    return (address << 16 /*= to the start of significant bits*/) | flags;
}
