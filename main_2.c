/*
 * File:   main_2.c
 * Author: Omar Al-Bayati
 *
 * Created on January 9, 2018, 1:04 PM
 */


#include "xc.h"
#include "configBits.h"
#include "stdbool.h"
#include "trans_header.h"


void remapSpiIO(void);
void delay();

//void _ISR _AltSPI2Interrupt (void);


void spi_write(unsigned short put);
unsigned char spi_read(unsigned char v);

void spi_write_short(unsigned char address, unsigned char data);


void spi_write_long(unsigned short address, unsigned char data);
void spi_write_long_read(unsigned short address, unsigned char data);
unsigned char spi_read_long(unsigned short v);

//Minco
void int_interrupt(void);
void _ISRFAST __attribute__((interrupt, auto_psv)) _INT1Interrupt(void);
unsigned char spi_read(unsigned char v);
unsigned char PHYGetLongRAMAddr(unsigned short addressL);
unsigned char abc, v, read, he;
unsigned char PHYGetShortRAMAddr(unsigned char address);
void PHYSetShortRAMAddr(unsigned char address, unsigned char data);
unsigned char li;
char PredefinedPacket[] = {0x01, 0x08, 0xC4, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00};

int main(void) {
//    unsigned char tt = 0;
//    unsigned char i;
//    initSPI();
//    remapSpiIO();
//
//    while (1) {
//        //spi_write(tt);
//        delay();
//        _LATB7 = 0X00;
//        // spi_write_short(0x22, 0x59);
//        // spi_write_long(0x222, 0x45);
//        tt = spi_read_long(0X222);
//        delay();
//        delay();
//        _LATB7 = 0XFF;
//        spi_write(tt);
//        for (i = 0; i < 32; i++)
//            delay();
//
//
//
//    }
    unsigned char adress=0x01,adress1=0x00;
 unsigned char data=0;
 unsigned char v,p1,p2;
 initSPI();
// int_interrupt();
 remapSpiIO();
 MRF24J40Init();


 li=0x00;
 while(1)
 { 
	 
  
  //PHYSetShortRAMAddr(0xff,0x23);
  //PHYSetLongRAMAddr(0x010,0x03);
  li=li+1;
  p1=(li<<1)+1;
  p1=p1&0x7f;
  //PHYSetShortRAMAddr(0x01,0x23);
  PHYSetLongRAMAddr(0x05,0x07);
  Nop();Nop();Nop();
  Nop();Nop();Nop();
  //dell();
  //read=(unsigned char)PHYGetLongRAMAddr(0x200);
  delay1(0x10);
  p2=(li<<1)+0;
  p2=p2&0x7f;
  //read=PHYGetShortRAMAddr(0x00);
  read=PHYGetLongRAMAddr(0x05);
  Nop();
  Nop();
  Nop(); 
  TX();
  //Send_Packet();
  delay1 (30);
  Nop();
  Nop();
  Nop(); 
  
  
 }   
    return 0;
}

void initSPI(void) {
    IFS0bits.SPI1IF = 0; //Clear the Interrupt Flag
    IEC0bits.SPI1IE = 0; //disable the Interrupt

    // SPI1CON1 Register Settings
    SPI1CON1bits.DISSCK = 0; //Internal Serial Clock is Enabled.
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1CON1bits.DISSDO = 0; //SDOx pin is controlled by the module.
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1CON1bits.MODE16 = 0; //Communication is byte-wide (8 bits).
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1CON1bits.SMP = 0; //Input Data is sampled at the middle of data output time.CHANGED
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1CON1bits.CKE = 1; //Serial output data changes on transition from
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    //Idle clock state to active clock state
    SPI1CON1bits.CKP = 0; //Idle state for clock is a low level;
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    //active state is a high level
    SPI1CON1bits.MSTEN = 1; //Master Mode Enabled
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1STATbits.SPIEN = 1; //Enable SPI Module
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1CON1bits.PPRE = 0b00; //primary scealar 64:1
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    SPI1CON1bits.SPRE = 0b000; //secondary prscalar 8:1

    //Interrupt Controller Settings
    IFS0bits.SPI1EIF = 0;
    Nop();
    Nop();
    IFS0bits.SPI1IF = 0; //Clear the Interrupt Flag
    IEC0bits.SPI1IE = 0; //Enable the Interrupt
}

void spi_write(unsigned short put) {
    unsigned short b;
    IFS0bits.SPI1IF = 0;
    b = SPI1BUF;
    Nop();
    SPI1BUF = put;
    Nop();
    while (IFS0bits.SPI1IF == 0) {
    }
}

unsigned char spi_read(unsigned char v) {
    //spi_write(0x00);
    // while (SPI1STATbits.SPIRBF==0) {};

    return (SPI1BUF);

}

unsigned char spi_read_long(unsigned short address) {
    // to read from another slave device
    ///spi_write_long_read(v, 0x00);
    // while (SPI1STATbits.SPIRBF==0) {};


    unsigned char a;
    unsigned char d;
    address = address << 5;
    Nop();
    address = address & 0xFFE0;
    Nop();
    address = address | 0x8000;
    Nop();
    a = address >> 8;
    spi_write(a);
    Nop();
    a = (unsigned char) address;
    spi_write(a);
    Nop();
    d = 0;
    spi_write(d);
    Nop();
    return (SPI1BUF);

}

void delay(void) {
    //delay
    unsigned int i;
    for (i = 0; i < 250; i++);

}

void remapSpiIO(void) {
    // port remap
    RPINR20bits.SCK1R = 0b01000; // Clock Input (Pin 17)
    RPINR20bits.SDI1R = 0b00110; // Slave Data IN (Pin 15)  

    _TRISB9 = 0; // B9 is output (Pin 18)
    _RP9R = 7; // Remap Data output to (Pin 18)

    _TRISB7 = 0; // B7 is output (Pin 16)
    _RP7R = 9; // Remap chip select to (Pin 16)

    _TRISB4 = 0; // B8 is output (Pin 11)
    _RP4R = 8; // Remap clock output select to (Pin 11)
    
     RPINR20=0b1100;  // 0000 0000 1000,RP12(SDI1)_CN7
 
 AD1PCFGL=0XFFFF;  //DISBLE ANOLOG
}

void spi_write_short(unsigned char address, unsigned char data) {
    unsigned char a;
    unsigned char d;

    address = address << 1;
    address = address | 0X45;
    address = address & 0X7f;
    a = (unsigned char) address;
    spi_write(a);
    d = data;
    d = (unsigned char) data;
    spi_write(d);
}

void spi_write_long(unsigned short address, unsigned char data) {
    // spi write long address
    unsigned char a;
    unsigned char d;
    address = address << 4;
    Nop();
    address = address | 0x8010;
    Nop();
    a = address >> 8;
    spi_write(a);
    Nop();
    a = (unsigned char) address;
    spi_write(a);
    Nop();
    d = (unsigned char) data;
    spi_write(d);
    Nop();
}

//void spi_write_long_read(unsigned short address, unsigned char data) {
//     spi write used in long address read
//
//    unsigned char a;
//    unsigned char d;
//    address = address << 4;
//    Nop();
//    address = address & 0xFFE0;
//    Nop();
//    address = address | 0x8000;
//    Nop();
//    a = address >> 8;
//    spi_write(a);
//    Nop();
//    a = (unsigned char) address;
//    spi_write(a);
//    Nop();
//    d = (unsigned char) data;
//    spi_write(d);
//    Nop();
//    
//
//}






//void _ISR _AltSPI2Interrupt (void){
//    
//    
//}

/*Minco*/
void int_interrupt(void) {
    IFS1bits.INT1IF = 0; //Clear the Interrupt Flag
    IEC1bits.INT1IE = 1; //Enable the Interrupt
    IPC2bits.SPI1IP =7; // privilidge

}

void PHYSetShortRAMAddr(unsigned char address, unsigned char data) {

    PORTBbits.RB7 = 0; //CS LOW
    //dell();
    spi_write(address);
    spi_write(data);
    dell();
    PORTBbits.RB7 = 1; //CS high

    //RFIE = tmpRFIE; 

}

unsigned char PHYGetShortRAMAddr(unsigned char address) {
    PORTBbits.RB7 = 0; //CS LOW
    //dell();
    spi_write(address);
    he = spi_read(0x55);
    dell();
    PORTBbits.RB7 = 1; //CS high

    return he;
}

void PHYSetLongRAMAddr(unsigned short addressL, unsigned char dataL) {
    unsigned short abc, addxx, addyy;
    unsigned char addH, addL;

    PORTBbits.RB7 = 0; //CS LOW
    //dell();
    abc = addressL;
    //abc=0x0200;
    abc = abc | 0x0400;
    abc = ((abc << 1) + 1);
    addyy = abc >> 4;
    addH = (unsigned char) addyy;
    addyy = abc << 4;
    addL = (unsigned char) addyy;
    spi_write(addH);
    spi_write(addL);
    spi_write(dataL);
    dell();
    PORTBbits.RB7 = 1; //CS hign

}

unsigned char PHYGetLongRAMAddr(unsigned short address2) {
    unsigned short aba, addxx, addzz;
    unsigned char addH, addL, ww;

    PORTBbits.RB7 = 0; //CS LOW
    //dell();
    aba = address2;
    //aba=0x200;
    aba = aba | 0x0400;

    aba = aba << 1 + 0; //for read
    addzz = aba >> 4;
    addH = (unsigned char) addzz;
    addzz = aba << 4;
    addL = (unsigned char) addzz;
    spi_write(addH);
    spi_write(addL);
    //Spi_write(addL);
    ww = spi_read(v);
    dell();
    PORTBbits.RB7 = 1; //CS hign

    return ww;

}

void dell(void) {
    short abs;
    for (abs = 0; abs < 100; abs++) {
        Nop();
    }
}

void TX(void) { //check the channel busy or not ,setChannel(currentChannel)
    unsigned char currentChannel = 0, uu, yy;
    //SetChannel(currentChannel);
    //currentChannel += 0x10;
    //*******Omar: The next 18 lines are used to create the transmiter package.                           
    PHYSetLongRAMAddr(0x000, 2); /* packet header length (2 Bytes in length (16bits)) */
    PHYSetLongRAMAddr(0x001, 16); /* total packet length (not including the FCS/CRC or length) */
    PHYSetLongRAMAddr(0x002, 0x00); /* data byte */
    PHYSetLongRAMAddr(0x003, 0x01); /* data byte */
    PHYSetLongRAMAddr(0x004, 0x02); /* data byte */
    PHYSetLongRAMAddr(0x005, 0x03); /* data byte */
    PHYSetLongRAMAddr(0x006, 0x04); /* data byte */
    PHYSetLongRAMAddr(0x007, 0x05); /* data byte */
    PHYSetLongRAMAddr(0x008, 0x06); /* data byte */
    PHYSetLongRAMAddr(0x009, 0x07); /* data byte */
    PHYSetLongRAMAddr(0x00a, 0x08); /* data byte */
    PHYSetLongRAMAddr(0x00b, 0x09); /* data byte */
    PHYSetLongRAMAddr(0x00c, 0x0a); /* data byte */
    PHYSetLongRAMAddr(0x00d, 0x0b); /* data byte */
    PHYSetLongRAMAddr(0x00e, 0x0c); /* data byte */
    PHYSetLongRAMAddr(0x00f, 0x0d); /* data byte */
    PHYSetLongRAMAddr(0x010, 0x0e); /* data byte */
    PHYSetLongRAMAddr(0x011, 0x0f); /* data byte */
    //*******Omar: Next line is used to send(shoot) the data.                       
    PHYSetShortRAMAddr(WRITE_TXNCON, 0b00000001);

    delay1(3);
    uu = PHYGetShortRAMAddr(READ_INTSTAT);
    //uu=PHYGetShortRAMAddr(READ_TXSTAT);
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    if (uu & 0x01) {
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
    }
    yy = PHYGetShortRAMAddr(READ_INTSTAT);
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    //while(INTSTATbits.TXNIF== 0){};
    //while(IFS0bits.SPI1IF== 0){};
    /* for (i=0;i<400;i++)
     {
         PHYSetShortRAMAddr(WRITE_TXNCON,0b00000001);
         for(j = 0; j < 1000; j++) ;
         if(ConsoleIsGetReady())
         {
         input = ConsoleGet();
								
         if((input == 24) | (input == 26))
             {	ConsolePut(input);
                 nl();	
             if(input == 24)
                 ResetMRF24J40();
             goto MRF24J40_SubMenu;
             }
         }
     }*/

    //read RSSI before transmit

}

void MRF24J40Init(void) {
    unsigned short j;
    unsigned char i;


    //PHYSetShortRAMAddr(WRITE_SOFTRST,0x07);//SOFT RESET

    PHYSetShortRAMAddr(WRITE_PACON2, 0x98);
    //PACON2 = 0x98, Initialize FFOEN=1 and TXONTS = 0x6
    PHYSetShortRAMAddr(WRITE_TXSTBL, 0x95);
    //Initialize RFSTBL = 0x9
    //PHYSetLongRAMAddr(RFCON0,0x03);

    PHYSetLongRAMAddr(RFCON1, 0x01);
    //Initialize VCOOPT=1
    PHYSetLongRAMAddr(RFCON2, 0x80);
    //Enable PLL
    PHYSetLongRAMAddr(RFCON3, 0x00);
    //set tx power to max		
    PHYSetLongRAMAddr(RFCON6, 0x90);
    //Initialize TXFIL=1(TX filter), 20MRECVR=1
    PHYSetLongRAMAddr(RFCON7, 0x80);
    //Initialize SLPCLKSEL = 0x2 (100KHz internal oscialltor)
    PHYSetLongRAMAddr(RFCON8, 0x10);
    //Initialize RFVCO =1
    PHYSetLongRAMAddr(SLPCON1, 0x21);
    //Initialize CLKOUTEN=1 and SLPCLKDIV = 0x01
    PHYSetShortRAMAddr(WRITE_BBREG2, 0x80);
    //Set CCA mode to ED
    PHYSetShortRAMAddr(WRITE_CCAEDTH, 0x60);
    //Set CCA-ED Threshold
    PHYSetShortRAMAddr(WRITE_BBREG6, 0x40);
    //Set appended RSSI value to RX FIFO
    PHYSetShortRAMAddr(WRITE_INTCON, 0xF6);


    PHYSetLongRAMAddr(RFSTATE, 0xA0);
    //set receive mode
    //INTCON (0x32) = 0xF6 - Enables only TXNIF and RXIF interrupts	
    SetChannel(CHANNEL_15);
    //set operating channel as channel 11
    for (j = 0; j < (unsigned short) 800; j++) {
        Nop();
    }
    //Delay for 192 us after RF State Machine Reset
    /*do
    {
        i = PHYGetLongRAMAddr(RFSTATE);
    }
    while((i&0xA0) != 0xA0);  
        //Wait until the RFSTATE machine indicates RX state
     */
    /* Program the short MAC Address, 0xffff */
    PHYSetShortRAMAddr(WRITE_SADRL, 0xFF);
    PHYSetShortRAMAddr(WRITE_SADRH, 0xFF);
    //load the short address of the device with 0xffff
    // which means that it will be ignored upon receipt
    PHYSetShortRAMAddr(WRITE_PANIDL, 0xFF);
    PHYSetShortRAMAddr(WRITE_PANIDH, 0xFF);
    //load the pan address also with 0xffff; 

    /* Program Long MAC Address*/
    /*  for(i=0;i<(unsigned char)8;i++)
      {
          PHYSetShortRAMAddr(WRITE_EADR0+i*2,myLongAddress[i]);
      }*/
    // PHYSetShortRAMAddr(WRITE_WAKECON,0x80);
    //WAKECON = 0x80 Enable immediate wakeup mode

}
void SetChannel(unsigned char channel)
{
    
    PHYSetLongRAMAddr(RFCON0, channel );
				//RFCON0 = 0x02 for Channel number 11
    PHYSetShortRAMAddr(WRITE_RFCTL,0x04);
    PHYSetShortRAMAddr(WRITE_RFCTL,0x00);
				//Reset RF State machine
				//no delay loop is present after performing RF State machine reset
}





void delay1(unsigned short p1)
{ unsigned short p2;
  for (p2=1;p2<p1;p2++ )
  {
   Nop();
   Nop();
  Nop();Nop();
  Nop();Nop();
  Nop();Nop();
  Nop();Nop();
  Nop();Nop();
  Nop();Nop();
  Nop();Nop();

  
  }

}
