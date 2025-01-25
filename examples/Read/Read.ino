/*
This is simple example to read all data from CAN bus and print it out to serial bus.
*/

#include "STM32_CAN.h"
STM32_CAN Can( PA_11, PA_12 );  //Use PA11/12 pins for CAN1.
//STM32_CAN Can( CAN1, ALT );  //Use PB8/9 pins for CAN1.
//STM32_CAN Can( CAN1, ALT_2 );  //Use PD0/1 pins for CAN1.
//STM32_CAN Can( CAN2, DEF );  //Use PB12/13 pins for CAN2.
//STM32_CAN Can( CAN2, ALT );  //Use PB5/6 pins for CAN2
//STM32_CAN Can( CAN3, DEF );  //Use PA8/15 pins for CAN3.
//STM32_CAN Can( CAN3, ALT );  //Use PB3/4 pins for CAN3

static CAN_message_t CAN_RX_msg;

void setup() {
  Serial.begin(115200);
  Can.begin();

#if defined(HAL_FDCAN_MODULE_ENABLED)
  // Can.setFrameFormat(STM32_CAN::FRAME_FORMAT::CLASSIC);//no FD frames. default
  // Can.setFrameFormat(STM32_CAN::FRAME_FORMAT::FD_NO_BRS);//use FD mode without baudrate switch
  Can.setFrameFormat(STM32_CAN::FRAME_FORMAT::FD_BRS);//use FD mode with baudrate switch
#endif
  // Can.setBaudRate(500000); // 500kbps, no FD switched data rate, even on FD capable hardware
  Can.setBaudRate(500000, 1000000);  //500kbps, use 1Mbps switched data rate (ignored on non-fd peripheral)
}

void loop() {
  if (Can.read(CAN_RX_msg) ) {
    Serial.print("Channel:");
    Serial.print(CAN_RX_msg.bus);
    if (CAN_RX_msg.flags.extended == false) {
      Serial.print(" Standard ID:");
    }
    else {
      Serial.print(" Extended ID:");
    }
    Serial.print(CAN_RX_msg.id, HEX);

#if defined(HAL_FDCAN_MODULE_ENABLED)
    if(CAN_RX_msg.flags.fd_rateswitch) {
      Serial.println(" [FD-BRS]");
    }
    else if(CAN_RX_msg.flags.fd_frame) {
      Serial.println(" [FD]");
    }
#endif

    Serial.print(" DLC: ");
    Serial.print(CAN_RX_msg.len);
    if (CAN_RX_msg.flags.remote == false) {
       Serial.print(" buf: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        Serial.print("0x"); 
        Serial.print(CAN_RX_msg.buf[i], HEX); 
        if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
      }
      Serial.println();
    } else {
       Serial.println(" Data: REMOTE REQUEST FRAME");
    }
  }
}