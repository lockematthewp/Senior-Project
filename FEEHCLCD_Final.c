/***********************************************************************************
* INCLUDES
***********************************************************************************/
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"  
#include "nwk_api.h"   
#include "bsp_leds.h" 
#include "bsp_buttons.h"  
#include "bsp_extended.h" 
#include "nwk.h"
#include "stdio.h"
#include "updatedPWM.c"
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_cc8051.h"
#include "ioCCxx10_bitdef.h"
#include "ioCC1110.h"

/***********************************************************************************
* CONSTANTS and DEFINITIONS
***********************************************************************************/
#define SPIN_ABOUT_QUARTER_A_SECOND   NWK_DELAY(250)
#define SPIN_ABOUT_100_MS             NWK_DELAY(100)

#define NUM_TX_RETRIES                3
#define NO_ACK_THRESHOLD              50

#define RSSI_UPPER_THRESHOLD          -40
#define RSSI_LOWER_THRESHOLD          -70

#define MINIMUM_OUTPUT_POWER          0
#define MEDIUM_OUTPUT_POWER           1
#define MAXIMUM_OUTPUT_POWER          2

#define SLEEP_31_25_US_RESOLUTION     0
#define SLEEP_1_MS_RESOLUTION         1
#define SLEEP_32_MS_RESOLUTION        2
#define SLEEP_1_S_RESOLUTION          3

#define MASTER_BUTTON                 1
#define SLAVE_BUTTON                  2
#define BOTH_BUTTONS                  3
#define SLOW_BLINK                    8192
#define MEDIUM_BLINK                  4096
#define FAST_BLINK                    2048

#define Panic_Button                  P2_0

/***********************************************************************************
* LOCAL VARIABLES
***********************************************************************************/
static          linkID_t      sLinkID;
static volatile uint8_t       sSemaphore;
static 		uint8_t       output = 0; 
static		uint8_t       output_lc =0;
static		uint8_t       output_on =0;
static		uint8_t       output_off =0;
static          uint8_t       PGOOD=0;

/***********************************************************************************
* LOCAL FUNCTIONS
***********************************************************************************/
static uint8_t    sRxCallback(linkID_t);
static void       CLCD(addr_t);
static uint32     blinks,blinkRate;

#pragma vector = T3_VECTOR
__interrupt void t3_isr(void)
{
    T3CH1IF = 0;                        //Clears the module interrupt flag.
    T3IF = 0;                           //Clears the CPU interrupt flag.
    blinks++;
    if (blinks>=blinkRate)
    {
        T3CC1= pwm_returnDC();
          
        if(pwm_returnDC()==0xA0)        
            pwm_p1_4_setDC(0xFF);       //turn it off
        else   
            pwm_p1_4_setDC(0xA0);
        blinks=0;
    }       
}

#pragma vector = P1INT_VECTOR
__interrupt void p1_ISR(void)
{
    P1IFG &= ~P1_5;              // Clear status flag for pin
    P1IF = 0;                    // Clear CPU interrupt status flag for P1
    PGOOD=1;
    bsp_PowerMode(POWER_MODE_3);
}

/***********************************************************************************
* @fn          main
*
* @brief       This is the main entry of the SMPL link application. It sets
*              random addresses for the nodes, initalises and runs
*              MASTER and SLAVE tasks sequentially in an endless loop.
*
* @return      none
************************************************************************************/
void main (void)
{
  blinkRate     =SLOW_BLINK;
  uint8  DUTY=0xA0;
  uint8  PERIOD=0xFF;
  BSP_Init();
  pwm_p1_4_init();
  pwm_p1_4_setDC(DUTY);
  pwm_p1_4_setPER(PERIOD);
  
  // Create and set random address for this device. 
  addr_t lAddr;
  BSP_createRandomAddress(&lAddr);      
  SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);

  // Initialize SimpliciTI and provide Callback function 
  SMPL_Init(sRxCallback);

  // Turn on LEDs indicating power on 
  BSP_TURN_ON_LED1();
  BSP_TURN_ON_LED2();

  BSP_SleepFor( POWER_MODE_2, SLEEP_1_MS_RESOLUTION, 1000);

  BSP_TURN_OFF_LED1();
  BSP_TURN_OFF_LED2();
  
  bsp_PowerMode(POWER_MODE_0);  //enter super low power mode until enough power to actually turn on
  
  while(1)
  { 
      if(PGOOD==1)
        CLCD(lAddr);
  }
}

/***********************************************************************************
* @fn          CLCD
*
* @brief       Sends a packet and waits for message from another coupler
*              If no message from another coupler, being to assign coupler
*              Blinking red led indicates packet not acknowledged
*              Adjust output power dynamically
*
* @param       addr_t Address
*
* @return      none
***********************************************************************************/
static void CLCD(addr_t Address)
{
  uint8_t radioMsg[4],coupMsg[4],len, valid,i,last5PktsRcv[5],pktsSent,pktsRecv;
  i=0;
  valid = 1;
  pktsSent=0;
  pktsRecv=0;
  
  //coupler ID
  coupMsg[0]=0x00;     //truck number
  coupMsg[1]=0x00;     //coupler number
  coupMsg[2]=0x00;     //panic ID
  coupMsg[3]=((MRFI_RandomByte() + coupMsg[1])* MRFI_RandomByte()) + pktsSent;//algorithm to prevent network storm
  
  //Build The Inital Radio Message
  radioMsg[0]=coupMsg[0];     //truck number 
  radioMsg[1]=coupMsg[1];     //coupler number
  radioMsg[2]=coupMsg[2];     //panic ID
  radioMsg[3]=coupMsg[3];     //nwkStorm Prevention
  
  SMPL_Send(SMPL_LINKID_USER_UUD, radioMsg, sizeof(radioMsg));  //The key of destiny "who am I?"
  pktsSent++;
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);        //Turn on RX. default is RX off.
  
  while((radioMsg[0]==coupMsg[0] && radioMsg[1]==coupMsg[1] && radioMsg[2]==coupMsg[2] && radioMsg[3]==coupMsg[3] ) && i < 3)      //check a few times [can be modified] (While (msg==default or iterations are lessthan 3)
  {
      NWK_DELAY(10);                                                //wait [10ms]
      SMPL_Receive(SMPL_LINKID_USER_UUD, radioMsg, &len);           //Did we recieve anything? If not, we will begin assigning numbers.
      i++;
  }
  
  SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXIDLE, 0);          //Radio IDLE to save power 

  if(radioMsg[0]==coupMsg[0] && radioMsg[1]==coupMsg[1] && radioMsg[2]==coupMsg[2] && radioMsg[3]==coupMsg[3])     //If no response from coupler, attempt to assign IDs
  {
      coupMsg[0]=1;                                               //You are now truck one
      coupMsg[1]=1;                                               //You are now coupler one
  }

  else        //If response from coupler assign IDs from msg
  {
    coupMsg[0]=radioMsg[0];
    coupMsg[1]=radioMsg[1];
    coupMsg[2]=radioMsg[2];
    last5PktsRcv[pktsRecv]=radioMsg[3];
    pktsRecv++;
  }
  
  //Clear The Radio Msg
  radioMsg[0]=0x00;     //truck number 
  radioMsg[1]=0x00;     //coupler number
  radioMsg[2]=0x00;     //panic ID
  radioMsg[3]=0x00;     //packet ID
  
  while(1)
  {
      if(pktsRecv==4)
        pktsRecv=0;
        
      output = Panic_Button;
      
      //Debounce
      output_off = ~output &output_lc;
      output_on = output & ~output_lc;
      output_lc = output;
      
      if (output_on)  //panic button pressed
      {	
                blinkRate  =MEDIUM_BLINK;             // change to faster duty cycle
                radioMsg[0]=coupMsg[0];			//Truck ID
                radioMsg[1]=coupMsg[1];			//Coupler number
                radioMsg[2]=coupMsg[1];			//Set Panic Coupler ID
                radioMsg[3]=((MRFI_RandomByte() + coupMsg[1])* MRFI_RandomByte()) + pktsSent;//algorithm to prevent network storm
                SMPL_Send(SMPL_LINKID_USER_UUD, radioMsg, sizeof(radioMsg));  //Broadcast Panic
                pktsSent++;
      }
      
      BSP_SleepFor(POWER_MODE_2, SLEEP_1_MS_RESOLUTION, 500);           //Sleep for 500ms
      SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);             //Turn on RX. Radio was idled to save power.
      NWK_DELAY(10);                                                    //wait [10ms]
      SMPL_Receive(SMPL_LINKID_USER_UUD, radioMsg, &len);               //Did anyone talk? If so, time to assign numbers, or send panic states
      SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXIDLE, 0);           //Radio IDLE to save power
      
      for(i = 0; i < 5; i++)
      {
         if (last5PktsRcv[i] == radioMsg[3])
         {
           valid = 0;   //already recieved this packet
         }
         else
         {
           valid = 1;
         }
      }
      
      if (valid)
      {
            if(radioMsg[0]==0x00&&radioMsg[1]==0x00&&radioMsg[2]==0x00&&radioMsg[3]!=0x00)       //did we recieve the default packets?
            {
                last5PktsRcv[pktsRecv]=radioMsg[3];
                pktsRecv++;
                //assign the new coupler
                radioMsg[0]=coupMsg[0];                                       //Same truck
                //INSERT LOGIC FOR DIFFERENT TRUCK
                radioMsg[1]=coupMsg[1]+1;                                     //Coupler number increment by one.
                radioMsg[2]=coupMsg[2];                                       //Same Panic ID ~ May be default
                radioMsg[3]=((MRFI_RandomByte() + coupMsg[1])* MRFI_RandomByte()) + pktsSent; //nwkstorm prevention
                SMPL_Send(SMPL_LINKID_USER_UUD, radioMsg, sizeof(radioMsg));  //send the config  
                pktsSent++;
            }
      
            else if(radioMsg[0]==coupMsg[0] && radioMsg[2]!=0x00) //did we recieve panic and same truck
            {
                blinkRate     =MEDIUM_BLINK;
                SMPL_Send(SMPL_LINKID_USER_UUD, radioMsg, sizeof(radioMsg)); //rebroadcast panic      
            } 
            
            else if(radioMsg[0]==coupMsg[0] && radioMsg[2]==0xFF) //did we recieve panic reset and same truck
            {
                blinkRate     =SLOW_BLINK;
                radioMsg[2]=0xFF;                                            //radioMsg[2]=0xFF is panic reset   
                SMPL_Send(SMPL_LINKID_USER_UUD, radioMsg, sizeof(radioMsg)); //rebroadcast panic reset
            }       
      }
    }
}

/***********************************************************************************
* @fn          sRxCallback
*
* @brief       
*
* @param       lid - link id message receive at
*
* @return      0 - frame left for application to read
*              1 - frame could be overwritten
************************************************************************************/
static uint8_t sRxCallback(linkID_t lid)
{
  if(lid)
  {  
    sSemaphore = 1;
   
    /* Radio IDLE to save power */
    SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXIDLE, 0);
  }
  
  /* Leave frame to be read by application. */
  return 0;
}
