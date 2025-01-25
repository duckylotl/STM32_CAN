/*
This is simple example to send random data to CAN bus in 20Hz rate, using delay (not recommended in real implementations).
*/

#include "STM32_CAN.h"
STM32_CAN Can( PA_11, PA_12 );  //Use PA11/12 pins for CAN1.
//STM32_CAN Can( CAN1, ALT );  //Use PB8/9 pins for CAN1.
//STM32_CAN Can( CAN1, ALT_2 );  //Use PD0/1 pins for CAN1.
//STM32_CAN Can( CAN2, DEF );  //Use PB12/13 pins for CAN2.
//STM32_CAN Can( CAN2, ALT );  //Use PB5/6 pins for CAN2
//STM32_CAN Can( CAN3, DEF );  //Use PA8/15 pins for CAN3.
//STM32_CAN Can( CAN3, ALT );  //Use PB3/4 pins for CAN3

static CAN_message_t CAN_TX_msg;

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
  uint8_t Counter = 0;
  while(1){
    if (Counter > 255){ Counter = 0;}
    delay(50);
    CAN_TX_msg.id = (0x1A5);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  Counter;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0x00;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    CAN_TX_msg.flags.fd_frame = false;
    CAN_TX_msg.flags.fd_rateswitch = false;
#endif
  
    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x1AC32CF5);
    CAN_TX_msg.flags.extended = 1;  // To enable extended ID.
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[3] =  0x21;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  Counter;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0xFF;

    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0xA63);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x63;
    CAN_TX_msg.buf[1] =  0x49;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  0x22;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  Counter;
    CAN_TX_msg.buf[7] =  0x00;

    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x23);
    CAN_TX_msg.flags.extended = 0;  // Back to standard ID.
#if defined(HAL_FDCAN_MODULE_ENABLED)
    CAN_TX_msg.len = 64;
#else
    CAN_TX_msg.len = 8;
#endif
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  0x33;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  Counter;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    /** 8-51 default 0x00
     * repeat above for last 8 byte */
    CAN_TX_msg.buf[52] =  0x03;
    CAN_TX_msg.buf[53] =  0x41;
    CAN_TX_msg.buf[54] =  0x11;
    CAN_TX_msg.buf[59] =  0x33;
    CAN_TX_msg.buf[60] =  0x00;
    CAN_TX_msg.buf[61] =  0x00;
    CAN_TX_msg.buf[62] =  0x00;
    CAN_TX_msg.buf[63] =  Counter;
    //send as FD frame (needed for len > 8)
    CAN_TX_msg.flags.fd_frame = true;
#endif

    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x55);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x44;
    CAN_TX_msg.buf[2] =  0x31;
    CAN_TX_msg.buf[3] =  0x53;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  Counter;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0x00;
#if defined(HAL_FDCAN_MODULE_ENABLED)
    //also switch baudrate when sending data
    CAN_TX_msg.flags.fd_rateswitch = true;
#endif

    Can.write(CAN_TX_msg);
    Serial.print("Sent: ");
    Serial.println(Counter, HEX);
    Counter++;
  }
}
