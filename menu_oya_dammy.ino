/***********************************************************************************************************************/
/* File Name    : 
/* Version      : 
/* Device(s)    :   Ardino nano
/*              :    
/* Description  :   REM >> nano
/* Creation Date: 
/***********************************************************************************************************************/
#include    <MsTimer2.h>
#include    <SPI.h>
#include    <SoftwareSerial.h>

//#include    "Debug.h"


/***********************************************************************************************************************/
/*  
/***********************************************************************************************************************/
 
#define     DE          12

#define PMD2_OUTPUT() (DDRD |= _BV(2))              // pinMode(2,OUTPUT)
#define PMD3_OUTPUT() (DDRD |= _BV(3))              // pinMode(3,OUTPUT)
#define PMD4_OUTPUT() (DDRD |= _BV(4))              // pinMode(4,OUTPUT)
#define PMD5_OUTPUT() (DDRD |= _BV(5))              // pinMode(5,OUTPUT)
#define PMD6_OUTPUT() (DDRD |= _BV(6))              // pinMode(6,OUTPUT)
#define PMD7_OUTPUT() (DDRD |= _BV(7))              // pinMode(7,OUTPUT)

#define PMD8_OUTPUT() (DDRB |= _BV(0))              // pinMode(8,OUTPUT)
#define PMD9_OUTPUT() (DDRB |= _BV(1))              // pinMode(9,OUTPUT)
#define PMD10_OUTPUT() (DDRB |= _BV(2))             // pinMode(10,OUTPUT)
#define PMD11_OUTPUT() (DDRB |= _BV(3))             // pinMode(11,OUTPUT)
#define PMD12_OUTPUT() (DDRB |= _BV(4))             // pinMode(12,OUTPUT)
#define PMD13_OUTPUT() (DDRB |= _BV(5))             // pinMode(13,OUTPUT)
 
#define DWD13_HIGH()   (PORTB |= _BV(5))            // digitalWrite(13,HIGH)
#define DWD13_LOW()    (PORTB &= ~_BV(5))           // digitalWrite(13,LOW)

#define DWD12_HIGH()   (PORTB |= _BV(4))            // digitalWrite(12,HIGH)
#define DWD12_LOW()    (PORTB &= ~_BV(4))           // digitalWrite(12,LOW)


//#define PINB    0x03
//#define DDRB    0x04
//#define PORTB   0x05

//#define PINC    0x06
//#define DDRC    0x07
//#define PORTC   0x08

//#define PIND    0x09
//#define DDRD    0x0A
//#define PORTD   0x0B

/***********************************************************************************************************************/
/*   Prototype
/***********************************************************************************************************************/
void    timerFire();
void    remRec();
void    serRead();
void    enqComm();
void    stxComm();
byte    makeLrc(byte* d);
void    clkWait();
//void    select(int key);
char    select(int key);
void    vStatus();
/***********************************************************************************************************************/
/*  Global variables and functions
/***********************************************************************************************************************/

volatile unsigned long  old_ulMicros = 0;
volatile unsigned long  ulMicros = 0;
volatile unsigned long  ulwith = 0;
volatile unsigned long  uldata = 0;
volatile int plus = 0;
volatile unsigned long  ulwait = 0;

volatile    boolean     remrecv =   false;
volatile    int     count1  =   8;
volatile    int     count2  =   1800;

boolean ansY   =   false;
byte    ansM    =   2;

byte    data[31];                               //rxdBff
byte    g_index   =   0;                        //bffCount
byte    rxFlg   =   0;


boolean sendwait=   false;



byte    dipaddr  =   0x00;
byte    dipmode =   0x01;                       /* test */

byte    outp[16]    ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte    instatus[18];                           /* 0=0x02 1.2=FF 3=g 4-7 change 8-11 status 12-15 REM 16=0x03 17=LRC */

unsigned int     inputdata;
unsigned int     lastdata    =0x0000;           /* LastTime DATA */


unsigned int     seridata;
byte    openretry   =   0;
byte    closeretry  =   0;

byte    ledout;                                 /* test */

int     testyou;

union {
        unsigned long   rem32;

            struct {                                /* 16bit2陋滂ｿｽ=32bit邵ｺ�ｽｮ隶堤洸�ｿｽ�ｿｽ闖ｴ�ｿｽ */
                unsigned int     code1;
                unsigned int     comm1;
            } data;
} remin;

const byte tbuff[]= { 0x02,"F0",
                     "H001","H011","H021","H031",
                     "H041","H051","H061","H071",
                     "H081","H091",'H','0',0x3a,'1',
                     "H0",0x3b,'1',"H0",0x3c,'1',"H0",0x3d,'1',"H0",0x3e,'1',
                     "H0",0x3f,'1','\0'
                    };


byte    hflag= 0;

                     
//* SoftwareSerial softSerial(6,5);     //* (rx,tx)
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void setup() {
    Serial.begin(19200,SERIAL_8E1);     //* Serial.begin(19200);
    
    //* softSerial.begin(19200);
    
    //BeginDebugPrint();
    //DebugPrint("Initialize..");
    
    pinMode(13, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(DE, LOW);        //40clock
    attachInterrupt(0, blink, RISING);            /* INT0 = pin2 */
    MsTimer2::set(50,timerFire);                  /* 50mSEC */
    MsTimer2::start();
    //menuE();
    //digitalWrite(DE, HIGH);
    //softSerial.write(0x05);
    //softSerial.write(0x05);
    //softSerial.write(0x05);
    //softSerial.write(0x05);
    //softSerial.write(0x05);  
    delay(500);
    
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void loop(){ 

    if(remrecv == true) remRec();
    serRead();
    stxComm();
    
#if 0
    if(hflag==0xfa){
        hflag= 0;
        ansM= 0;
        //DebugPrint("debig Print 4");
    }
    
    if(ansM==0){
        vPolling();
    }
    if(ansM==1){
        yPolling();
    }
    if(ansM==2){
        ePolling();
    }
    //ansY == false ? yPolling(): vPolling();
    //serRead();
    //stxComm(); 
#endif
}// for loop

//const   byte PROGMEM table[] = {   
//            0x02,
//            "D0Y",
//            0x03,
//            0x02,
//           0
//};
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void ePolling(){
    
    if(count1== 0){
        count1 = 4;
        
        menuE();
        
        dataClr(data,31);             //rxbuff CLR
        //DebugPrint("debig Print e");   
    }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void yPolling(){
        
        if(count1 == 0){  
            count1 = 4;
            
            menuY();
           
            
            ansM=2;            //* Y郢ｧ雋橸ｿｽ�ｽｺ邵ｺ蜉ｱ笳�郢ｧ迚呻ｽｼ�ｽｷ陋ｻ�ｽｶ騾ｧ�ｿｽ邵ｺ�ｽｫ
            //* yStatus();
            dataClr(data,31);               //rxbuff CLR
            //DebugPrint("debig Print y");
        }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void vPolling(){
        
        if(count1 == 0){
            count1 = 4;
            
            menuV();
            
            //* remrecv == true ? remRec(): vStatus();
            dataClr(data,31);               //rxbuff CLR
            //DebugPrint("debig Print v");     
        }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void timerFire(){
    
    if(count1   !=  0) count1--;
    if(count2   !=  0) count2--;           /* LIFE COUNT */
}
/***********************************************************************************************************************
* Function Name: RemComm 32bitTYPE
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void blink(){
        
    ulMicros = micros();
    ulwith = ulMicros - old_ulMicros;
    old_ulMicros = ulMicros;

    if (ulwith > 40000 || ulwith < 400){
        plus = 0;
        uldata = 0;
        
    }else{
        if (ulwith > 1500){
        uldata = uldata | 0x80000000 ; 
    }
    
    if (plus >= 32){
        remrecv =   true ;
        //Serial.print(uldata, HEX);
        remin.rem32   =  uldata;
    }else{
        plus++;
        uldata = uldata >> 1;
    }
  }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 郢晁�後Ε郢晁ｼ斐＜郢晢ｽｪ郢晢ｽｳ郢ｧ�ｽｰ邵ｺ�ｽｮ邵ｺ�ｽｿ邵ｲ竏晢ｿｽ�ｽｦ騾��ｿｽ邵ｺ�ｽｯenqComm()邵ｺ陌腎xComm()邵ｺ�ｽｧ
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void serRead(){
    
    if(Serial.available() > 0){                         //-1邵ｺ迹夲ｽｿ譁絶夢邵ｺ�ｽｦ邵ｺ荳奇ｽ狗ｸｺ荵晢ｽ�
        
        data[g_index]   =   Serial.read();
       
        (rxFlg== 0x03)&&(rxFlg= '!');

        if(data[g_index]    ==0x02 && rxFlg==0){            /*郢ｧ�ｽｹ郢ｧ�ｽｿ郢晢ｽｼ郢晏沺�ｽ､諛ｷ�ｿｽ�ｽｺ*/
            rxFlg   =0x02;
            g_index =0;
            data[g_index]   =0x02;                          /* STX */
        }
        if(data[g_index]    ==0x03 && rxFlg==0x02)rxFlg=0x03;/*郢ｧ�ｽｨ郢晢ｽｳ郢晉判�ｽ､諛ｷ�ｿｽ�ｽｺ*/
            
        //if(data[g_index]    ==0x05 && rxFlg==0){            /*ENQ隶�諛ｷ�ｿｽ�ｽｺ*/
        //    rxFlg   ='?';                                   /* 0x3F */
        //    g_index    =0;
        //    data[g_index]   =0x05;                          /* ENQ */
        //}
        if(data[g_index]== 0x04){
            rxFlg   = '?';
            g_index = 0;
            data[g_index]= 0x04;
        }
        if(data[g_index]== 0xfa){
            hflag= 0xfa;
            //DebugPrint("debig Print fa");
        }
        g_index++;
        g_index &=0x1F;
    }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void stxComm(){
       
    if(rxFlg == '!'){
        rxFlg = 0;
        g_index= 0;
       // for(byte i=0; 0x03!=data[i]; i++){
       //     Serial.write(data[i]);    
       // }
            
        if(data[1]== 'F' && data[2]== 'F'){
            //if(data[3]=='x') ansM= 1;
            //if(data[3]=='i'){
            //    digitalWrite(DE, HIGH);
            //    for(byte i=0; '\0'!=tbuff[i]; i++){
            //        Serial.write(tbuff[i]);          
            //    }
            // ansM=2;
            
            if(data[3]== 'h'){
              digitalWrite(DE,HIGH);
              Serial.write(0x04);
              Serial.flush();
              digitalWrite(DE, LOW);
            }
        }
    }
    if(rxFlg== '?'){
        rxFlg = 0;
        g_index= 0;       
    }
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
byte    makeLrc(byte* d){
        
        byte    tmp;
        byte    index   =   1;
        
        tmp =   d[index];
        do{
            index++;
            tmp     =   tmp    ^   d[index];
        }while(data[index]!=0x03);
        
        return(tmp);
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 0 -> dest[i]
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void    dataClr(byte *dest, byte lim){
    
    for(byte i=0; i< lim; i++){
        dest[i] =   0;
    }
    
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
struct  remcode{
        int     key ;
        void    (*pfunk)(void);
}

const   remcode[17] ={  0xB847,out1set,     //* 1
                        0xF40B,out2set,     //* 2
                        0xF50A,out3set,     //* 3
                        0xF609,out4set,     //* 4
                        0xB44B,out5set,     //* 5
                        0xB54A,out6set,     //* 6
                        
                        0xB649,out1clr,     //* 7
                        0xB748,out2clr,     //* 8
                        0xB24D,out3clr,     //* 9
                        0xB34C,out4clr,     //* 10
                        0xF00F,out5clr,     //* 11
                        0xF10E,out6clr,     //* 12

                        0xB04F,menuY,     //* blue
                        0xB14E,menuE,     //* red
                        0xED12,menuV,     //* green
                        0xEE11,yStatus,   //* yawlo
                        0xF708,ans06      //*
                        };

/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    remRec(){
        
        if(remrecv  ==true){
            //DebugPrint("debig Print 3"); 
            
            remrecv = false;
            
            if(remin.data.code1!=0x3586){                   /* 3586 */
                return;
            }
            
            char    i = select(remin.data.comm1);   //-1邵ｺ譴ｧ諤冗ｹｧ荵敖ｰ郢ｧ蝟ｶhar陜呻ｿｽ
            
            if(i == -1){                             //return(-1) ERR
                return;
            }
            DWD12_HIGH();
            //* softSerial.write(0x02);
                     
            (*remcode[i].pfunk)();
                
            //* softSerial.write(0x03);
            //* softSerial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
            Serial.flush();
            DWD12_LOW();        
            
            //DebugPrint("debig Print 4");  
        }                               
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 郢晢ｿｽ郢ｧ�ｽｹ郢晁ご逡�
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    ledMoni(){
        
        digitalWrite(13, ledout);
        ledout = !ledout; 
}
/***********************************************************************************************************************
* Function Name: 邵ｺ阮呻ｼ�邵ｺ�ｽｧ陞ｳ貅ｯ�ｽ｡蠕後■郢ｧ�ｽ､郢晢ｿｽ
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
#if  0
void     select(int key){

        for(byte i=0;i<17;i++){
            if(remcode[i].key   ==  key){
                (*remcode[i].pfunk)();          //邵ｺ阮呻ｼ�邵ｺ�ｽｧ陞ｳ貅ｯ�ｽ｡�ｿｽ
                break;
            }
        }
}
#endif
/***********************************************************************************************************************
* Function Name: 隰鯉ｽｻ邵ｺ�ｽ｣邵ｺ�ｽｦ陞ｳ貅ｯ�ｽ｡蠕後■郢ｧ�ｽ､郢晢ｿｽ
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
char     select(int key){

        for(byte i=0;i<17;i++){
            if(remcode[i].key   ==  key){
                return(i);                      //隰鯉ｽｻ邵ｺ�ｽ｣邵ｺ�ｽｦ陞ｳ貅ｯ�ｽ｡�ｿｽ      
            }
        }
        return(-1);                             //-1邵ｺ�ｿｽ邵ｺ荵晢ｽ営har陜呻ｿｽ
}                                                                                        
/***********************************************************************************************************************
* Function Name: 
* Description  : 0.625 * 16 = 1uS
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    clkWait(){
         asm volatile(
        
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
            "nop \n"
        );
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
#if  0
void    out1set(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K001"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out2set(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K012"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out3set(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K021"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out4set(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K031"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out5set(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K041"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out6set(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K051"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out1clr(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K000"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out2clr(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K010"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out3clr(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K020"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out4clr(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K030"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);           //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out5clr(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K040"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
void    out6clr(){
        digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0K050"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);             //陷ｿ蛹∬ｦ�邵ｺ蛹ｻ笘�郢晢ｿｽ郢晄ｺ假ｿｽ�ｽｼ
        digitalWrite(DE, LOW);
}
#endif

void    vStatus(){
        //DWD12_HIGH();
        //digitalWrite(DE, HIGH);
        Serial.write(0x02);
        Serial.print(F("D0V"));    //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x21);
        //digitalWrite(DE, LOW);
        //DWD12_LOW();
}
void    yStatus(){
        //asm("SBI 0x05,4");              // 2clock
        //DWD12_HIGH();                 // 3clock
        //digitalWrite(DE, HIGH);       //40clock
        Serial.write(0x02);
        Serial.print(F("D0Y"));    //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x2E);
        //asm("CBI 0x05,4");              // 2clock
        //DWD12_LOW();                 // 3clock
        //digitalWrite(DE, LOW);        //40clock
} 
void    menuE(){
        //digitalWrite(DE, HIGH);       //40clock
        Serial.write(0x05);
        Serial.print(F("F0"));    //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        //digitalWrite(DE, LOW);        //40clock          
}
void    menuV(){
        //digitalWrite(DE, HIGH);       //40clock
        Serial.write(0x02);
        Serial.print(F("F0V"));    //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x23);
        //digitalWrite(DE, LOW);        //40clock                  
}
void    menuY(){
        //digitalWrite(DE, HIGH);       //40clock
        Serial.write(0x02);
        Serial.print(F("F0Y"));    //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x2C);
        //digitalWrite(DE, LOW);        //40clock                  
}
void    ans06(){
        Serial.write(0x06);
}
/***********************************************************************************************************************
* Function Name: 
* Description  : 
* Arguments    : Code
* Return Value : None
***********************************************************************************************************************/
void    out1set(){
        Serial.write(0x02);
        Serial.print(F("F0H001"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0c); 
}
void    out2set(){
        Serial.write(0x02);
        Serial.print(F("F0H011"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0d);
}
void    out3set(){
        Serial.write(0x02);
        Serial.print(F("F0H021"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0e);
}
void    out4set(){
        Serial.write(0x02);
        Serial.print(F("F0H031"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0f);
}
void    out5set(){
        Serial.write(0x02);
        Serial.print(F("F0H041"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x08);
}
void    out6set(){
        Serial.write(0x02);
        Serial.print(F("F0H051"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x09);
}
void    out1clr(){
        Serial.write(0x02);
        Serial.print(F("F0H000"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0d);
}
void    out2clr(){
        Serial.write(0x02);
        Serial.print(F("F0H010"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0c);
}
void    out3clr(){
        Serial.write(0x02);
        Serial.print(F("F0H020"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0f);
}
void    out4clr(){
        Serial.write(0x02);
        Serial.print(F("F0H030"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x0e);
}
void    out5clr(){
        Serial.write(0x02);
        Serial.print(F("F0H040"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x09);
}
void    out6clr(){
        Serial.write(0x02);
        Serial.print(F("F0H050"));  //flash-> RAM郢ｧ蜻茨ｽｶ驛�ｽｲ�ｽｻ邵ｺ蜉ｱ竊醍ｸｺ�ｿｽ
        Serial.write(0x03);
        Serial.write(0x08);
}
