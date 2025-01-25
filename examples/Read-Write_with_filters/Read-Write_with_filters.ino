/*
This example shows how to read filtered CAN messages from the bus and send data out using timers.
*/

#include "STM32_CAN.h"

static CAN_message_t CAN_outMsg_1;
static CAN_message_t CAN_outMsg_2;
static CAN_message_t CAN_outMsg_3;
static CAN_message_t CAN_inMsg;

// This will use PA11/12 pins for CAN1 and set RX-buffer size to 64-messages. TX-buffer size is kept at default 16.
STM32_CAN Can( PA_11, PA_12, RX_SIZE_64, TX_SIZE_16 );

uint8_t Counter;

void SendData()  // Send can messages in 50Hz phase from timer interrupt.
{
  if (Counter >= 255){ Counter = 0;}
  
  // Only the counter value is updated to the 3 messages sent out.
  CAN_outMsg_1.buf[3] =  Counter; 
  Can.write(CAN_outMsg_1);

  CAN_outMsg_2.buf[5] =  Counter;
  Can.write(CAN_outMsg_2);

  CAN_outMsg_3.buf[6] =  Counter;
  Can.write(CAN_outMsg_3);

  Serial.print("Sent: ");
  Serial.println(Counter, HEX);
  Counter++;
}
 
void readCanMessage()  // Read data from CAN bus and print out the messages to serial bus. Note that only message ID's that pass filters are read.
{
  Serial.print("Channel:");
  Serial.print(CAN_inMsg.bus);
  if (CAN_inMsg.flags.extended == false) {
    Serial.print(" Standard ID:");
  }
  else {
    Serial.print(" Extended ID:");
  }
  Serial.print(CAN_inMsg.id, HEX);

#if defined(HAL_FDCAN_MODULE_ENABLED)
    if(CAN_inMsg.flags.fd_rateswitch) {
      Serial.println(" [FD-BRS]");
    }
    else if(CAN_inMsg.flags.fd_frame) {
      Serial.println(" [FD]");
    }
#endif

  Serial.print(" DLC: ");
  Serial.print(CAN_inMsg.len);
  if (CAN_inMsg.flags.remote == false) {
     Serial.print(" buf: ");
    for(int i=0; i<CAN_inMsg.len; i++) {
      Serial.print("0x"); 
      Serial.print(CAN_inMsg.buf[i], HEX); 
      if (i != (CAN_inMsg.len-1))  Serial.print(" ");
    }
    Serial.println();
  } else {
     Serial.println(" Data: REMOTE REQUEST FRAME");
  }
}

void setup(){
  Counter = 0;
  Serial.begin(115200);
  
  Can.begin();

#if defined(HAL_FDCAN_MODULE_ENABLED)
  // Can.setFrameFormat(STM32_CAN::FRAME_FORMAT::CLASSIC);//no FD frames. default
  // Can.setFrameFormat(STM32_CAN::FRAME_FORMAT::FD_NO_BRS);//use FD mode without baudrate switch
  Can.setFrameFormat(STM32_CAN::FRAME_FORMAT::FD_BRS);//use FD mode with baudrate switch
#endif
  // Can.setBaudRate(500000); // 500kbps, no FD switched data rate, even on FD capable hardware
  Can.setBaudRate(500000, 1000000);  //500kbps, use 1Mbps switched data rate (ignored on non-fd peripheral)

  Can.setFilterSingleMask(0, 0x153, 0x1FFFFFFF, AUTO);
  Can.setFilterSingleMask(1, 0x613, 0x1FFFFFFF, AUTO);
  // You can also set that is the ID Standard or Extended
  Can.setFilterSingleMask(2, 0x615, 0x1FFFFFFF, STD);
  Can.setFilterSingleMask(3, 0x1F0, 0x1FFFFFFF, EXT);

  // We set the data that is static for the three different message structs once here.
  CAN_outMsg_1.id = (0x1A5);
  CAN_outMsg_1.len = 8;
  CAN_outMsg_1.buf[0] =  0x03;
  CAN_outMsg_1.buf[1] =  0x41;
  CAN_outMsg_1.buf[2] =  0x11;
  CAN_outMsg_1.buf[3] =  0x00;
  CAN_outMsg_1.buf[4] =  0x00;
  CAN_outMsg_1.buf[5] =  0x00;
  CAN_outMsg_1.buf[6] =  0x00;
  CAN_outMsg_1.buf[7] =  0x00;

  CAN_outMsg_2.id = (0x7E8);
#if defined(HAL_FDCAN_MODULE_ENABLED)
  CAN_outMsg_2.len = 64;
#else
  CAN_outMsg_2.len = 8;
#endif
  CAN_outMsg_2.buf[0] =  0x03;
  CAN_outMsg_2.buf[1] =  0x41;
  CAN_outMsg_2.buf[3] =  0x21;
  CAN_outMsg_2.buf[4] =  0x00;
  CAN_outMsg_2.buf[5] =  0x00;
  CAN_outMsg_2.buf[6] =  0x00;
  CAN_outMsg_2.buf[7] =  0xFF;
#if defined(HAL_FDCAN_MODULE_ENABLED)
  CAN_outMsg_2.flags.fd_frame = true;
#endif

  CAN_outMsg_3.id = (0xA63);
  CAN_outMsg_3.len = 8;
  CAN_outMsg_3.buf[0] =  0x63;
  CAN_outMsg_3.buf[1] =  0x49;
  CAN_outMsg_3.buf[2] =  0x11;
  CAN_outMsg_3.buf[3] =  0x22;
  CAN_outMsg_3.buf[4] =  0x00;
  CAN_outMsg_3.buf[5] =  0x00;
  CAN_outMsg_3.buf[6] =  0x00;
  CAN_outMsg_3.buf[7] =  0x00;
#if defined(HAL_FDCAN_MODULE_ENABLED)
  CAN_outMsg_3.flags.fd_frame = true;
  CAN_outMsg_3.flags.fd_rateswitch = true;
#endif

  // setup hardware timer to send data in 50Hz pace
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *SendTimer = new HardwareTimer(Instance);
  SendTimer->setOverflow(50, HERTZ_FORMAT); // 50 Hz
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  SendTimer->attachInterrupt(1, SendData);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  SendTimer->attachInterrupt(SendData);
#endif
  SendTimer->resume();
}

// main loop
void loop() {
  // The actual code that is being used will be done to main loop as usual.

  // We only read data from CAN bus if there is frames received, so that main code can do it's thing efficiently.
  while (Can.read(CAN_inMsg) ) 
  {
    readCanMessage();
  }
}
