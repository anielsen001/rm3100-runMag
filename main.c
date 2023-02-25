//=========================================================================
// main.c
//
// A command line interface for the RM3100 3-axis magnetometer from PNI Sensor Corp.
// Derived in part from several sources:
//      HTTPS://github.com/miguelrasteiro/RM3100.X
//      Jeremiah Mattison / Rm3100: https://os.mbed.com/users/fwrawx/code/Rm3100/
//      HTTPS://github.com/shannon-jia/rm3100
//      Song Qiang <songqiang1304521@gmail.com (Linux driver):
//          Linux kernel driver: HTTPS://github.com/torvalds/linux/tree/v5.3/drivers/iio/magnetometer
//
// Author:      David Witten, KD0EAG
// Date:        May 12, 2020
// License:     GPL 3.0
//=========================================================================
#include "cmdmgr.h"
#include "main.h"

//------------------------------------------
// Static variables
//------------------------------------------
char version[] = RUNMAG_VERSION;
char outFilePath[MAXPATHBUFLEN] = "./logs/";
char workFilePath[MAXPATHBUFLEN] = "";
char rollOverTime[UTCBUFLEN] = "00:00";
char sitePrefixString[SITEPREFIXLEN] = "SITEPREFIX";
#if (USE_PIPES)
char outputPipeName[MAXPATHBUFLEN] = "/home/web/wsroot/pipein.fifo";
char inputPipeName[MAXPATHBUFLEN] = "/home/web/wsroot/pipeout.fifo";
#endif
static char  mSamples[9];

//------------------------------------------
// readTemp()
//------------------------------------------
int readTemp(pList *p, int devAddr)
{
    int temp = -9999;
    char data[2] = {0};
    char reg[1] = {MCP9808_REG_AMBIENT_TEMP};

    i2c_setAddress(p->i2c_fd, devAddr);
    write(p->i2c_fd, reg, 1);
    if(read(p->i2c_fd, data, 2) != 2)
    {
        fprintf(stderr, "Error : I/O error reading temp sensor at address: [0x%2X].\n", devAddr);
    }
    else
    {
        // Convert the data to 13-bits
        temp = ((data[0] & 0x1F) * 256 + data[1]);
        if(temp > 4095)
        {
            temp -= 8192;
        }
    }
    return temp;
}

//------------------------------------------
// readMagCMM()
//------------------------------------------
int readMagCMM(pList *p, int devAddr, int32_t *XYZ)
{
    int rv = 0;
    int bytes_read = 0;

    i2c_setAddress(p->i2c_fd, devAddr);
    // Check if DRDY went high and wait unit high before reading results
    while((rv = (i2c_read(p->i2c_fd, RM3100I2C_STATUS)) & RM3100I2C_READMASK) != RM3100I2C_READMASK)
    {
    }
    // Read the XYZ registers
    if((bytes_read = i2c_readbuf(p->i2c_fd, RM3100I2C_XYZ, (unsigned char*) &mSamples, sizeof(mSamples)/sizeof(char))) != sizeof(mSamples)/sizeof(char))
    {
        perror("i2c transaction i2c_readbuf() failed.\n");
    }
    XYZ[0] = ((signed char)mSamples[0]) * 256 * 256;
    XYZ[0] |= mSamples[1] * 256;
    XYZ[0] |= mSamples[2];

    XYZ[1] = ((signed char)mSamples[3]) * 256 * 256;
    XYZ[1] |= mSamples[4] * 256;
    XYZ[1] |= mSamples[5];

    XYZ[2] = ((signed char)mSamples[6]) * 256 * 256;
    XYZ[2] |= mSamples[7] * 256;
    XYZ[2] |= mSamples[8];

    return bytes_read;
}

//------------------------------------------
// readMagPOLL()
//------------------------------------------
int readMagPOLL(pList *p, int devAddr, int32_t *XYZ)
{
    int rv = 0;
    int bytes_read = 0;
    short pmMode = (PMMODE_ALL);

    i2c_setAddress(p->i2c_fd, devAddr);
    // Write command to  use Continuous measurement Mode.
    i2c_write(p->i2c_fd, RM3100_MAG_POLL, pmMode);
    // if a delay is specified after DRDY goes high, sleep it off.
    if(p->DRDYdelay)
    {
        usleep(p->DRDYdelay);
    }
    // Check if DRDY went high and wait unit high before reading results
    while((rv = (i2c_read(p->i2c_fd, RM3100I2C_STATUS)) & RM3100I2C_READMASK) != RM3100I2C_READMASK)
    {
    }
    // Read the XYZ registers
    if((bytes_read = i2c_readbuf(p->i2c_fd, RM3100I2C_XYZ, (unsigned char*) &mSamples, sizeof(mSamples)/sizeof(char))) != sizeof(mSamples)/sizeof(char))
    {
        perror("i2c transaction i2c_readbuf() failed.\n");
    }
    XYZ[0] = ((signed char)mSamples[0]) * 256 * 256;
    XYZ[0] |= mSamples[1] * 256;
    XYZ[0] |= mSamples[2];

    XYZ[1] = ((signed char)mSamples[3]) * 256 * 256;
    XYZ[1] |= mSamples[4] * 256;
    XYZ[1] |= mSamples[5];

    XYZ[2] = ((signed char)mSamples[6]) * 256 * 256;
    XYZ[2] |= mSamples[7] * 256;
    XYZ[2] |= mSamples[8];

    return bytes_read;
}

//------------------------------------------
//  main()
//------------------------------------------
int main(int argc, char** argv)
{
    pList p;
    char utcStr[UTCBUFLEN] = "";
    //long runTime = 0;
    int currentDay = 0;
    struct tm *utcTime = getUTC();
    int32_t rXYZ[3];
    double xyz[3];
    int temp = 0;
    float lcTemp = 0.0;
    float rcTemp = 0.0;
    int rv = 0;
    time_t sec_count;
    time_t new_count;
    FILE *outfp = stdout;
#if (USE_PIPES)
    int  fdPipeIn;
    int  fdPipeOut;
#endif

    currentDay = utcTime->tm_mday;

    if((rv = getCommandLine(argc, argv, &p)) != 0)
    {
        return rv;
    }
    // Open log file.
    if(p.buildLogPath)
    {
        buildLogFilePath(&p);
        if((outfp = fopen(p.outputFilePath, "a+"))!= NULL)
        {
            printf("\nLog File: %s\n", p.outputFilePath);
        }
        else
        {
            perror("\nLog File: ");
        }
    }
    // if Verbose == TRUE
    if(p.verboseFlag)
    {
        // always stdout!
        fprintf(stdout,"\nStartup UTC time: %s", asctime(utcTime));
    }
    // Open I2C bus (only one at a time for now)
    openI2CBus(&p);
    // Setup the magnetometer.
    setup_mag(&p);
    // Show initial (command line) parameters
    if(p.showParameters)
    {
        showSettings(&p);
    }
    if(p.readBackCCRegs)
    {
        readCycleCountRegs(&p);
    }
    // Start CMM on X, Y, Z
    if(p.samplingMode == CONTINUOUS)
    {
        startCMM(&p);
    }

    if(!(p.jsonFlag))
    {
        // DRL put meta data here
        // DRL should be printed only at the top of the log file
        // DMW respect -H switch (but not other rtemp, ltemp, etc. options yet.)
        if(p.hideRaw)
        {
            fprintf(outfp, "\"time\", \"rtemp\", \"ltemp\", \"x\", \"y\", \"z\", \"total\"\n");
        }
        else
        {
            fprintf(outfp, "\"time\", \"rtemp\", \"ltemp\", \"x\", \"y\", \"z\", \"rx\", \"ry\", \"rz\", \"total\"\n");
        }
    }

#if (USE_PIPES)

    if(p.useOutputPipe = TRUE)
    {
        // Notide that fdPipeOut and fdPipeIn are intentionally reversed.
        if(!(fdPipeOut = open(p.pipeInPath, O_WRONLY | O_CREAT)))
        {
            perror("Open PIPE Out failed: ");
            fprintf(stderr, p.pipeInPath);
            exit(1);
        }
        if(!(fdPipeIn = open(p.pipeOutPath, O_RDONLY | O_CREAT)))
        {
            perror("Open PIPE In failed: ");
            fprintf(stderr, p.pipeInPath);
            exit(1);
        }
    }

#endif //USE_PIPES

    // loop
    while(1)
    {
        //  Read temp sensor.
        if(!p.magnetometerOnly)
        {
            if(p.remoteTempOnly)
            {
                temp = readTemp(&p, p.remoteTempAddr);
                rcTemp = temp * 0.0625;
            }
            else if(p.localTempOnly)
            {
                temp = readTemp(&p, p.localTempAddr);
                lcTemp = temp * 0.0625;
            }
            else
            {
                temp = readTemp(&p, p.remoteTempAddr);
                rcTemp = temp * 0.0625;
                temp = readTemp(&p, p.localTempAddr);
                lcTemp = temp * 0.0625;
            }
        }
        // Set magnetometer sampling mode.
        if((!p.localTempOnly) || (!p.remoteTempOnly))
        {
            if(p.samplingMode == POLL)                      // (p->samplingMode == POLL [default])
            {
                readMagPOLL(&p, p.magnetometerAddr, rXYZ);
            }
            else                                            // (p->samplingMode == CONTINUOUS)
            {
                readMagCMM(&p, p.magnetometerAddr, rXYZ);
            }
            xyz[0] = (((double)rXYZ[0] / p.NOSRegValue) / p.x_gain) * 1000;   // make microTeslas -> nanoTeslas
            xyz[1] = (((double)rXYZ[1] / p.NOSRegValue) / p.y_gain) * 1000;   // make microTeslas -> nanoTeslas
            xyz[2] = (((double)rXYZ[2] / p.NOSRegValue) / p.z_gain) * 1000;   // make microTeslas -> nanoTeslas
        }

        // Output the results.
        if(!(p.jsonFlag))
        {
            if(p.tsMilliseconds)
            {
                struct timeval time;
                gettimeofday(&time, NULL);
                //fprintf(outfp, "%ld ", currentTimeMillis());
                fprintf(outfp, "%ld ", time.tv_sec);
                fprintf(outfp, ", %ld ", time.tv_usec*1000);
            }
            else
            {
                utcTime = getUTC();
                strftime(utcStr, UTCBUFLEN, "%d %b %Y %T", utcTime);
                fprintf(outfp, "\"%s\"", utcStr);
            }
            if(!p.magnetometerOnly)
            {
                if(p.remoteTempOnly)
                {
                    if(rcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"ERROR\"");
                    }
                    else
                    {
                        fprintf(outfp, ", %.2f", rcTemp);
                    }
                }
                else if(p.localTempOnly)
                {
                    if(lcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"ERROR\"");
                    }
                    else
                    {
                        fprintf(outfp, ", %.2f", lcTemp);
                    }
                }
                else
                {
                    if(rcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"ERROR\"");
                    }
                    else
                    {
                        fprintf(outfp, ", %.2f", rcTemp);
                    }
                    if(lcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"ERROR\"");
                    }
                    else
                    {
                        fprintf(outfp, ", %.2f", lcTemp);
                    }
                }
            }
            fprintf(outfp, ", %.4f", xyz[0]/1000);
            fprintf(outfp, ", %.4f", xyz[1]/1000);
            fprintf(outfp, ", %.4f", xyz[2]/1000);
            if(!p.hideRaw)
            {
                fprintf(outfp, ", %i", rXYZ[0]/1000);
                fprintf(outfp, ", %i", rXYZ[1]/1000);
                fprintf(outfp, ", %i", rXYZ[2]/1000);
            }
            if(p.showTotal)
            {
                double x = xyz[0]/1000;
                double y = xyz[1]/1000;
                double z = xyz[2]/1000;
                fprintf(outfp, ", %.4f", sqrt((x * x) + (y * y) + (z * z)));
            }
            fprintf(outfp, "\n");
        }
        else    // JSON output ------------------------------------------------
        {
            fprintf(outfp, "{ ");
            if(p.tsMilliseconds)
            {
                fprintf(outfp, "\"ts\":\"%ld\"",  currentTimeMillis());
            }
            else
            {
                utcTime = getUTC();
                strftime(utcStr, UTCBUFLEN, "%d %b %Y %T", utcTime);        // RFC 2822: "%a, %d %b %Y %T %z"      RFC 822: "%a, %d %b %y %T %z"
                fprintf(outfp, "\"ts\":\"%s\"", utcStr);
            }
            if(!p.magnetometerOnly)
            {
                if(p.remoteTempOnly)
                {
                    if(rcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"rt\":0.0");
                    }
                    else
                    {
                        fprintf(outfp, ", \"rt\":%.2f",  rcTemp);
                    }
                }
                else if(p.localTempOnly)
                {
                    if(lcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"lt\":0.0");
                    }
                    else
                    {
                        fprintf(outfp, ", \"lt\":%.2f",  lcTemp);
                    }
                }
                else
                {
                    if(rcTemp < -100.0)
                    {
                        fprintf(outfp, ", \"rt\":0.0");
                    }
                    else
                    {
                        fprintf(outfp, ", \"rt\":%.2f",  rcTemp);
                    }
                    if(lcTemp <-100.0)
                    {
                        fprintf(outfp, ", \"lt\":0.0");
                    }
                    else
                    {
                        fprintf(outfp, ", \"lt\":%.2f",  lcTemp);
                    }
                }
            }
            fprintf(outfp, ", \"x\":%.4f", xyz[0]/1000);
            fprintf(outfp, ", \"y\":%.4f", xyz[1]/1000);
            fprintf(outfp, ", \"z\":%.4f", xyz[2]/1000);
            if(!p.hideRaw)
            {
                fprintf(outfp, ", \"rx\":%i", rXYZ[0]/1000);
                fprintf(outfp, ", \"ry\":%i", rXYZ[1]/1000);
                fprintf(outfp, ", \"rz\":%i", rXYZ[2]/1000);
            }
            if(p.showTotal)
            {
                double x = xyz[0]/1000;
                double y = xyz[1]/1000;
                double z = xyz[2]/1000;
                fprintf(outfp, ", \"Tm\": %.4f",  sqrt((x * x) + (y * y) + (z * z)));
            }
            fprintf(outfp, " }\n");
        }
        fflush(outfp);
        if(p.singleRead)
        {
            break;
        }
        // Per Bill Englkey
        do
        {
            time(&new_count);
            usleep(100000);
        } while (new_count==sec_count);
        sec_count = new_count;

        //printf("Seconds ctr=%ld\n",sec_count);

        // wait p.outDelay (1000 ms default) for next poll.
        // usleep(p.outDelay);
        utcTime = getUTC();
        if(p.buildLogPath)
        {
            if(utcTime->tm_mday != currentDay)
            {
                currentDay = utcTime->tm_mday;
                // currentDay = utcTime->tm_min;
                fclose(outfp);
                buildLogFilePath(&p);
                if((outfp = fopen(p.outputFilePath, "a+"))!= NULL)
                {
                    fprintf(stdout,"\nNew Log File: %s\n", p.outputFilePath);
                }
                else
                {
                    fprintf(stdout,"\nNew Log File: %s\n", p.outputFilePath);
                    perror("\nLog File: ");
                    exit(1);
                }
            }
        }
    }
    closeI2CBus(p.i2c_fd);
    return 0;
}
