
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// XDCtools Header files
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
#include <ti/drivers/I2C.h>
#include "Board.h"

#include <sys/socket.h>
#include <arpa/inet.h>

//#define HOSTNAME          "api.openweathermap.org"
//#define REQUEST_URI       "/data/2.5/forecast/?id=315202&APPID=b9bdaf75a7b1e96362a172ec83cb9303"
//#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
#define SOCKETTEST_IP     "192.168.1.25"
#define TASKSTACKSIZE     4096
#define OUTGOING_PORT     5011
#define INCOMING_PORT     5030

extern Semaphore_Handle semaphore0;     // posted by httpTask and pended by clientTask
char   tempstr[20];                     // temperature string

extern Semaphore_Handle semaphore1;
extern Semaphore_Handle semaphore2;

extern Mailbox_Handle mailbox0;
extern Mailbox_Handle mailbox1;

int32_t UP;
   int64_t OFF, OFF2, SENS, SENS2;
   int   T, Pa, TEMP, T2;
   uint32_t P;
   uint16_t   C1,C2,C3,C4,C5,C6;
   uint32_t  D1, D2;
   uint16_t TEMPSENS;
   uint8_t         txBuffer[3];
   uint8_t         rxBuffer[5];


//int8_t         txBuffer[3];
//int8_t         rxBuffer[2];
I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;



Void tim0(UArg arg0){

    Semaphore_post(semaphore1);

}

Void tim1(UArg arg0){

    Semaphore_post(semaphore2);

}

void delay(val){
   int i=0;
    for(i=0;i<(val*1000000);i++){};
}

void IIC_Init()
{

/* Create I2C for usage */
I2C_Params_init(&i2cParams);
i2cParams.bitRate = I2C_400kHz;
i2c = I2C_open(Board_I2C_TMP, &i2cParams);
if (i2c == NULL) {
    System_abort("Error Initializing I2C\n");
}
else {
    System_printf("I2C Initialized!\n");
}

if( I2C_transfer(i2c, &i2cTransaction)){

   System_printf("init edildi...\n");

}
    System_flush();
}



void IIC_Write(addr, data, w_count, r_count){

    txBuffer[0]=addr;
    txBuffer[1]=data;

    i2cTransaction.slaveAddress = 0x1D;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = w_count;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = r_count;


     if (    I2C_transfer(i2c, &i2cTransaction)) {
         /* Extract degrees C from the received data; see TMP102 datasheet */


        // System_printf("sended data \n ");
     }
     else {
         System_printf("I2C Bus fault\n");
     }
     System_flush();

}




int IIC_Read(addr){

txBuffer[0]=addr;

   i2cTransaction.slaveAddress = 0x1D;
   i2cTransaction.writeBuf = txBuffer;
   i2cTransaction.writeCount = 1;
   i2cTransaction.readBuf = rxBuffer;
   i2cTransaction.readCount = 1;

        int data=I2C_transfer(i2c, &i2cTransaction);

            if (data) {
                /* Extract degrees C from the received data; see TMP102 datasheet */


              //  System_printf("deger: %d \n", rxBuffer[0]);
            }
            else {
                System_printf("I2C Bus fault\n");
            }
            System_flush();

return  rxBuffer[0];
}



void freefall(){

IIC_Write(0x16, 0x12, 2, 0);
IIC_Write(0x18, 0x40, 2, 0);
IIC_Write(0x19, 0x01, 2, 0);
IIC_Write(0x1A, 0x0F, 2, 0);
}


void int_clear(){

IIC_Write(0x17, 0x03, 2, 0);
IIC_Write(0x17, 0x00, 2, 0);

}



Void app(UArg arg0, UArg arg1){

int a;
    Semaphore_pend(semaphore1, BIOS_WAIT_FOREVER);

IIC_Init();


IIC_Write(0x16, 0x15, 2, 0);


while(1){

freefall();

System_printf("FREE FALL:  %d\n",a=IIC_Read(0x0A));
Mailbox_post(mailbox0, &a, BIOS_NO_WAIT);
/*
System_printf("x value: %d\n",IIC_Read(0x02));
System_flush();
System_printf("y value: %d\n",IIC_Read(0x07));
System_flush();
System_printf("z value: %d\n",IIC_Read(0x08));*/
System_flush();


int_clear();
//delay();

Task_sleep(5000);

}

  }

////////////////////////////////////////////////////////////////////////////

void IIC_Write1(addr, w_count, r_count){

        txBuffer[0]=addr;


        i2cTransaction.slaveAddress = 0x77;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = w_count;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = r_count;


         if (    I2C_transfer(i2c, &i2cTransaction)) {
             /* Extract degrees C from the received data; see TMP102 datasheet */


          //  System_printf("sended data \n ");
         }
         else {
             System_printf("I2C Bus fault in read\n");
         }
         System_flush();

}


int IIC_Read1(addr,count,val){

    txBuffer[val]=addr;

       i2cTransaction.slaveAddress = 0x77;
       i2cTransaction.writeBuf = txBuffer;
       i2cTransaction.writeCount = 1;
       i2cTransaction.readBuf = rxBuffer;
       i2cTransaction.readCount = count;

            int data=I2C_transfer(i2c, &i2cTransaction);

                if (data) {
                    /* Extract degrees C from the received data; see TMP102 datasheet */


                  //  System_printf("deger: %d \n", rxBuffer[0]);
                }
                else {
                    System_printf("I2C Bus fault in read\n");
                }
                System_flush();

return  rxBuffer[val];
}




void reset(){

    IIC_Write1(0x1E, 1, 0);
    System_printf("caiss \n" );
}

void BMP180_getPressure()
{


    txBuffer[0] = 0x00;                                 // temperature register
    i2cTransaction.slaveAddress = 0x77;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 1;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 3;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
     //  System_printf("Pressure value acquired\n");
    }

    UP = rxBuffer[0]<<16 | rxBuffer[1]<<8 |rxBuffer[0] ;  //UT = raw pressure data
//    System_printf("VAL : %d\n", UP);
    System_flush();


}

void promData(){

    C1=((IIC_Read1(0xA2,2,0))<<8) | ((IIC_Read1(0xA2,2,1)));
//    System_printf("C1 : %d   \n",C1);
    System_flush();

    C2=((IIC_Read1(0xA4,2,0))<<8) | ((IIC_Read1(0xA4,2,1)));
  //  System_printf("C2 : %d \n",C2);
    System_flush();

    C3=((IIC_Read1(0xA6,2,0))<<8) | ((IIC_Read1(0xA6,2,1)));
   // System_printf("C3 : %d \n",C3);
    System_flush();

    C4=((IIC_Read1(0xA8,2,0))<<8) | ((IIC_Read1(0xA8,2,1)));
  //  System_printf("C4 : %d \n",C4);
    System_flush();

    C5=((IIC_Read1(0xAA,2,0))<<8) | ((IIC_Read1(0xAA,2,1)));
  //  System_printf("C5 : %d \n",C5);
    System_flush();

    C6=((IIC_Read1(0xAC,2,0))<<8) | ((IIC_Read1(0xAC,2,1)));
  //  System_printf("C6 : %d \n",C6);
    System_flush();

     /*    Tref = (C5 * 256.0);
          TEMPSENS = (C6 / 8388608);
      OFFt1 = (C2 * 65536.0);
          SENSt1 = (C1 * 32768.0);
*/
        // System_printf("c1 : %d  \n",C1);

}


void Data(){


   D1= D2=0;
    delay(2);
   // IIC_Write(0x40, 1, 0);
    IIC_Write1(0x48, 1, 0);
     delay(10);

    BMP180_getPressure();

    D1= UP;
    System_flush();
     delay(5);



    IIC_Write1(0x58, 1, 0);
    delay(5);


    BMP180_getPressure();
    D2= UP;
         System_flush();


        T = D2 - ((int)C5 << 8);
        TEMP = (2000 + (((int64_t)T * (int64_t)C6) >> 23));

        if (TEMP<2000)  //if temperature of the sensor goes below 20°C, it activates "second order temperature compensation"
           {
             T2=(T^2)/2147483648;
             OFF2=5*((TEMP-2000)^2)/2;
             SENS2=5*((TEMP-2000)^2)/4;
             if (TEMP<-1500) //if temperature of the sensor goes even lower, below -15°C, then additional math is utilized
               {
                 OFF2=OFF2+7*((TEMP+1500)^2);
                 SENS2=SENS2+11*((TEMP+1500)^2)/2;
               }
           }
           else
             {
                 T2=0;
                 OFF2=0;
                 SENS2=0;
             }


        TEMP = ((2000 + (((int64_t)T * (int64_t)C6) >> 23))-T2); //second order compensation included
         OFF = (((unsigned int)C2 << 16) + (((int64_t)C4 * T) >> 7)-OFF2); //second order compensation included
         SENS = (((unsigned int)C1 << 15) + (((int64_t)C3 * T) >> 8)-SENS2); //second order compensation included
         P = (((D1 * SENS) >> 21) - OFF) >> 15;



  //System_printf("D1 : %d  \n",D1);
  //  System_printf("Temprature : %d  \n", TEMP/100);
   // System_flush();
    System_printf("Pressure : %d\n", P);
    System_flush();
}


Void app2(UArg arg0, UArg arg1){
    Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);

reset();
delay(1);
     promData();
    delay(2);

while(1){

  Data();
  Mailbox_post(mailbox1, &P, BIOS_NO_WAIT);
  delay(5);
Task_sleep(1000);
}

}




void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}

bool sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd, connStat, numSend;
    bool retval=false;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        close(sockfd);
        return false;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     // convert port # to network order
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    connStat = connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("sendData2Server::Error while connecting to server\n");
    }
    else {
        numSend = send(sockfd, data, size, 0);       // send data to the server
        if(numSend < 0) {
            System_printf("sendData2Server::Error while sending data to server\n");
        }
        else {
            retval = true;      // we successfully sent the temperature string
        }
    }
    System_flush();
    close(sockfd);
    return retval;
}

Void clientSocketTask(UArg arg0, UArg arg1)
{
    static uint32_t tempval,tempval2;
    char state1[]="\fFREE FALL!!!\r\n";
    char state2[]="\fEverything is ok.\r\n";
    while(1) {
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site

        GPIO_write(Board_LED0, 1); // turn on the LED

        Mailbox_pend(mailbox0, &tempval, BIOS_WAIT_FOREVER);
        Mailbox_pend(mailbox1, &tempval2, BIOS_WAIT_FOREVER);

                if((int)tempval==1 && ((int) tempval2-9240)>0 ){
                    if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, state1, strlen(state1))) {

                               System_printf("clientSocketTask:: Temperature is sent to the server\n");
                               //System_printf("degerler %d %d\n",tempval,tempval2);

                               System_flush();
                           }

                }
                else{

                    if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, state2, strlen(state2))) {

                        System_printf("clientSocketTask:: Temperature is sent to the server\n");
                        System_flush();
                }


        GPIO_write(Board_LED0, 0);  // turn off the LED
    }
                    Task_sleep(500);

}

}

void getTimeStr(char *str)
{
    // dummy get time as string function
    // you may need to replace the time by getting time from NTP servers
    //
    strcpy(str, "2021-01-07 12:34:56");
}

float getTemperature(void)
{
    // dummy return
    //
    return atof(tempstr);
}

bool createTasks(void)
{
    static Task_Handle  taskHandle1, taskHandle2, taskHandle3, taskHandle4 ,taskHandle5;
    Task_Params taskParams;
    Error_Block eb;

    Error_init(&eb);


    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle1 = Task_create((Task_FuncPtr)clientSocketTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle2 = Task_create((Task_FuncPtr)app, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle3 = Task_create((Task_FuncPtr)app2, &taskParams, &eb);

    if (taskHandle1 == NULL || taskHandle2 == NULL || taskHandle3 == NULL) {
        printError("netIPAddrHook: Failed to create HTTP, Socket and Server Tasks\n", -1);
        return false;
    }

    return true;
}

//  This function is called when IP Addr is added or deleted
//
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    // Create a HTTP task when the IP address is added
    if (fAdd) {
        createTasks();
    }
}

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();
    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_flush();


    /* Start BIOS */
    BIOS_start();

    return (0);
}
