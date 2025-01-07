#include "STM32_CAN.h"

#include "core_debug.h"

#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
/** Pointer to CEC_HandleTypeDef structure that contains 
 * the configuration information for the specified CEC.
 * Application have to declare them properly to be able to call
 * the HAL_CEC_IRQHandler().
 */
extern CEC_HandleTypeDef * phcec;
#endif

#if defined(HAL_FDCAN_MODULE_ENABLED) && defined(STM32G0xx) && defined(FDCAN1)
/** On the G0 platform (STM32G0B1xx, STM32G0C1xx) the 2 FDCAN IRQs are shared
 * for both CAN instances. Additionally also shared with a Timer instance each.
 * The HardwareTimer driver from STM32 Arduino core already defines these IRQ
 * handlers, so we can't use them. But it does call the FDCAN IRQ Handler for us
 * if we set the following 2 pointer to a valid handler instance.
 */
FDCAN_HandleTypeDef *phfdcan1 = nullptr;
#ifdef FDCAN2
FDCAN_HandleTypeDef *phfdcan2 = nullptr;
#endif
#endif

#if defined(HAL_CAN_MODULE_ENABLED)
#define STM32_CAN_SINGLE_CAN_FILTER_COUNT 14
#define STM32_CAN_DUAL_CAN_FILTER_COUNT 28
#define STM32_CAN_CAN2_FILTER_OFFSET 14

#elif defined(HAL_FDCAN_MODULE_ENABLED)
#ifdef STM32_FDCAN_MEM_LAYOUT_FIXED
#define STM32_FDCAN_STD_FILTER_COUNT 28
#define STM32_FDCAN_EXT_FILTER_COUNT 8
#endif

#define STM32_FDCAN_SF_Size      (4U)
#define STM32_FDCAN_SF_T_Pos     (30U)
#define STM32_FDCAN_SF_T_Msk     (0x3UL << (STM32_FDCAN_SF_T_Pos))
#define STM32_FDCAN_SF_EC_Pos    (27U)
#define STM32_FDCAN_SF_EC_Msk    (0x7UL << (STM32_FDCAN_SF_EC_Pos))

#define STM32_FDCAN_EF_Size      (8U)
#define STM32_FDCAN_EF_W1_T_Pos  (30U)
#define STM32_FDCAN_EF_W1_T_Msk  (0x3UL << (STM32_FDCAN_EF_W1_T_Pos))
#define STM32_FDCAN_EF_W0_EC_Pos (29U)
#define STM32_FDCAN_EF_W0_EC_Msk (0x7UL << (STM32_FDCAN_EF_W0_EC_Pos))

#endif

#define STM32_CAN_TIMEOUT_VALUE 10U

//Max value, lowest priority
#define MAX_IRQ_PRIO_VALUE ((1UL << __NVIC_PRIO_BITS) - 1UL)

constexpr Baudrate_entry_t STM32_CAN::BAUD_RATE_TABLE_48M[];
constexpr Baudrate_entry_t STM32_CAN::BAUD_RATE_TABLE_45M[];

uint32_t test = 0;

typedef enum {
#if defined(CAN1) || defined(FDCAN1)
  CAN1_INDEX,
#endif
#if defined(CAN2) || defined(FDCAN2)
  CAN2_INDEX,
#endif
#if defined(CAN3) || defined(FDCAN3)
  CAN3_INDEX,
#endif
  CAN_NUM,
  CAN_UNKNOWN = 0xFFFF
} can_index_t;


#if defined(HAL_FDCAN_MODULE_ENABLED)
uint32_t STM32_CAN::commonClockDivRegValue = FDCAN_CLOCK_DIV1;
#endif

static stm32_can_t * canObj[CAN_NUM] = {NULL};

#if defined(HAL_CAN_MODULE_ENABLED)
stm32_can_t *get_can_obj(CAN_HandleTypeDef *hcan)
#elif defined(HAL_FDCAN_MODULE_ENABLED)
stm32_can_t *get_can_obj(FDCAN_HandleTypeDef *hcan)
#endif
{
  stm32_can_t *obj;
  obj = (stm32_can_t *)((char *)hcan - offsetof(stm32_can_t, handle));
  return (obj);
}

#if defined(HAL_CAN_MODULE_ENABLED)
can_index_t get_can_index(CAN_TypeDef *instance)
#elif defined(HAL_FDCAN_MODULE_ENABLED)
can_index_t get_can_index(FDCAN_GlobalTypeDef *instance)
#endif
{
  can_index_t index = CAN_UNKNOWN;
#if defined(HAL_CAN_MODULE_ENABLED)
#if defined(CAN1)
  if (instance == CAN1) {
    index = CAN1_INDEX;
  }
#endif
#if defined(CAN2)
  if (instance == CAN2) {
    index = CAN2_INDEX;
  }
#endif
#if defined(CAN3)
  if (instance == CAN3) {
    index = CAN3_INDEX;
  }
#endif

#elif defined(HAL_FDCAN_MODULE_ENABLED)
#if defined(FDCAN1)
  if (instance == FDCAN1) {
    index = CAN1_INDEX;
  }
#endif
#if defined(FDCAN2)
  if (instance == FDCAN2) {
    index = CAN2_INDEX;
  }
#endif
#if defined(FDCAN3)
  if (instance == FDCAN3) {
    index = CAN3_INDEX;
  }
#endif
#endif
  if (index == CAN_UNKNOWN) {
    Error_Handler();
  }
  return index;
}

bool STM32_CAN::allocatePeripheral()
{
  can_index_t index = get_can_index(_can.handle.Instance);
  if(index >= CAN_NUM)
  {
    return false;
  }
  if(canObj[index])
  {
    //bus already in use by other instance
    Error_Handler();
    return false;
  }
  //register with global, we own this instance now
  canObj[index] = &_can;
  return true;
}

bool STM32_CAN::freePeripheral()
{
  can_index_t index = get_can_index(_can.handle.Instance);
  if(index >= CAN_NUM)
  {
    return false;
  }
  if(canObj[index] == &_can)
  {
    canObj[index] = nullptr;
    _can.handle.Instance = nullptr;
    return true;
  }
  Error_Handler();
  return false;
}

bool STM32_CAN::hasPeripheral()
{
  can_index_t index = get_can_index(_can.handle.Instance);
  if(index >= CAN_NUM)
  {
    return false;
  }
  return canObj[index] == &_can;
}

STM32_CAN::STM32_CAN(uint32_t rx, uint32_t tx, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize)
  : sizeRxBuffer(rxSize), sizeTxBuffer(txSize),
    preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0)
{
  this->rx = digitalPinToPinName(rx);
  this->tx = digitalPinToPinName(tx);
  init();
}

STM32_CAN::STM32_CAN(PinName rx, PinName tx, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize)
  : rx(rx), tx(tx), sizeRxBuffer(rxSize), sizeTxBuffer(txSize),
    preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0)
{
  init();
}

#if defined(HAL_CAN_MODULE_ENABLED)
STM32_CAN::STM32_CAN( CAN_TypeDef* canPort, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize )
  : sizeRxBuffer(rxSize), sizeTxBuffer(txSize),
    preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0)
{
  //get first matching pins from map
  rx = pinmap_find_pin(canPort, PinMap_CAN_RD);
  tx = pinmap_find_pin(canPort, PinMap_CAN_TD);
  init();
}

//lagacy pin config for compatibility
STM32_CAN::STM32_CAN( CAN_TypeDef* canPort, CAN_PINS pins, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize )
  : rx(NC), tx(NC), sizeRxBuffer(rxSize), sizeTxBuffer(txSize),
    preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0)
{
  if (canPort == CAN1)
  {
    switch(pins)
    {
      case DEF:
        rx = PA_11;
        tx = PA_12;
        break;
      case ALT:
        rx = PB_8;
        tx = PB_9;
        break;
      #if defined(__HAL_RCC_GPIOD_CLK_ENABLE)
      case ALT_2:
        rx = PD_0;
        tx = PD_1;
        break;
      #endif
    }
  }
#ifdef CAN2
  else if(canPort == CAN2)
  {
    switch(pins)
    {
      case DEF:
        rx = PB_12;
        tx = PB_13;
        break;
      case ALT:
        rx = PB_5;
        tx = PB_6;
        break;
    }
  }
#endif
#ifdef CAN3
  else if(canPort == CAN3)
  {
    switch(pins)
    {
      case DEF:
        rx = PA_8;
        tx = PA_15;
        break;
      case ALT:
        rx = PB_3;
        tx = PB_4;
        break;
    }
  }
#endif
  init();
}

#elif defined(HAL_FDCAN_MODULE_ENABLED)
STM32_CAN::STM32_CAN( FDCAN_GlobalTypeDef* canPort, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize )
  : sizeRxBuffer(rxSize), sizeTxBuffer(txSize),
    preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0)
{
  //get first matching pins from map
  rx = pinmap_find_pin(canPort, PinMap_CAN_RD);
  tx = pinmap_find_pin(canPort, PinMap_CAN_TD);
  init();
}
#endif

void STM32_CAN::init(void)
{
  _can.__this = (void*)this;
  _can.handle.Instance = nullptr;
  baudrate = 0UL;
  filtersInitialized = false;

  setTimestampCounter(false);
  setTxBufferMode(TX_BUFFER_MODE::FIFO);
  setMode(MODE::NORMAL);
  setAutoRetransmission(true);

#if defined(HAL_CAN_MODULE_ENABLED)
  setAutoBusOffRecovery(false);
  setRxFIFOLock(false);
  _can.handle.Init.AutoWakeUp = DISABLE;

#elif defined(HAL_FDCAN_MODULE_ENABLED)
  #ifndef STM32_FDCAN_HAS_CLOCK_CALIBRATION_UNIT
  _can.handle.Init.ClockDivider = commonClockDivRegValue;
  #endif

  setFrameFormat(CLASSIC);
  setRxFIFOLock(false, false);
  setTransmitPause(false);
  setProtocolException(true);

  setFilterGlobalNonMatching(REJECT, REJECT);
  setFilterGlobalRTR(false, false);

  _can.handle.Init.StdFiltersNbr = STM32_FDCAN_STD_FILTER_COUNT;
  _can.handle.Init.ExtFiltersNbr = STM32_FDCAN_EXT_FILTER_COUNT;
#endif
}

#if defined(HAL_CAN_MODULE_ENABLED)
CAN_TypeDef   * STM32_CAN::getPeripheral()
#elif defined(HAL_FDCAN_MODULE_ENABLED)
FDCAN_GlobalTypeDef * STM32_CAN::getPeripheral()
#endif
{
  #if defined(HAL_CAN_MODULE_ENABLED)
  CAN_TypeDef * canPort_rx = (CAN_TypeDef *) pinmap_peripheral(rx, PinMap_CAN_RD);
  CAN_TypeDef * canPort_tx = (CAN_TypeDef *) pinmap_peripheral(tx, PinMap_CAN_TD);
  #elif defined(HAL_FDCAN_MODULE_ENABLED)
  FDCAN_GlobalTypeDef * canPort_rx = (FDCAN_GlobalTypeDef *) pinmap_peripheral(rx, PinMap_CAN_RD);
  FDCAN_GlobalTypeDef * canPort_tx = (FDCAN_GlobalTypeDef *) pinmap_peripheral(tx, PinMap_CAN_TD);
  #endif
  if ((canPort_rx != canPort_tx && canPort_tx != NP) || canPort_rx == NP)
  {
    //ensure pins relate to same peripheral OR only Rx is set/valid
    // rx only can be used as listen only but needs a 3rd node for valid ACKs

    // do not allow Tx only since that would break arbitration
    return NP;
  }

  #ifdef STM32F1xx
  /** AF remapping on the F1 platform only possible in pairs
   *  Verify that both pins use the same remapping
   *  Only enforced if both pins are used, in Rx only Tx pin is not set to AF*/
  if(canPort_rx != NP && canPort_tx != NP)
  {
    uint32_t rx_func = pinmap_function(rx, PinMap_CAN_RD);
    uint32_t tx_func = pinmap_function(tx, PinMap_CAN_TD);
    uint32_t rx_afnum = STM_PIN_AFNUM(rx_func);
    uint32_t tx_afnum = STM_PIN_AFNUM(tx_func);
    if(rx_afnum != tx_afnum)
    {
      //ERROR
      return NP;
    }
  }
  #endif

  //clear tx pin in case it was set but does not match a peripheral
  if(canPort_tx == NP)
    tx = NC;

  return canPort_rx;
}

/**-------------------------------------------------------------
 *     setup functions
 *     no effect after begin()
 * -------------------------------------------------------------
 */
void STM32_CAN::setIRQPriority(uint32_t preemptPriority, uint32_t subPriority)
{
  //NOTE: limiting the IRQ prio, but not accounting for group setting
  this->preemptPriority = min(preemptPriority, MAX_IRQ_PRIO_VALUE);
  this->subPriority = min(subPriority, MAX_IRQ_PRIO_VALUE);
}

void STM32_CAN::setAutoRetransmission(bool enabled)
{
  _can.handle.Init.AutoRetransmission = enabled ? (ENABLE) : (DISABLE);
}

void STM32_CAN::setRxFIFOLock(bool fifo0locked, bool fifo1locked)
{
#if defined(HAL_CAN_MODULE_ENABLED)
  (void)fifo1locked;
  _can.handle.Init.ReceiveFifoLocked = fifo0locked ? (ENABLE) : (DISABLE);
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  this->fifo0locked = fifo0locked;
  this->fifo1locked = fifo1locked;
#endif
}

void STM32_CAN::setTxBufferMode(TX_BUFFER_MODE mode)
{
  #if defined(HAL_CAN_MODULE_ENABLED)
  _can.handle.Init.TransmitFifoPriority = (FunctionalState)mode;
  #elif defined(HAL_FDCAN_MODULE_ENABLED)
  _can.handle.Init.TxFifoQueueMode = mode;
  #endif
}

void STM32_CAN::setTimestampCounter(bool enabled)
{
#if defined(HAL_CAN_MODULE_ENABLED)
  _can.handle.Init.TimeTriggeredMode = enabled ? (ENABLE) : (DISABLE);
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  timestampCounterEnabled = enabled;
#endif
}

void STM32_CAN::setMode(MODE mode)
{
  _can.handle.Init.Mode = mode;
}

void STM32_CAN::enableLoopBack( bool yes ) {
  setMode(yes ? MODE::LOOPBACK : MODE::NORMAL);
}

void STM32_CAN::enableSilentMode( bool yes ) {
  setMode(yes ? MODE::SILENT : MODE::NORMAL);
}

void STM32_CAN::enableSilentLoopBack( bool yes ) {
  setMode(yes ? MODE::SILENT_LOOPBACK : MODE::NORMAL);
}

#if defined(HAL_CAN_MODULE_ENABLED)
void STM32_CAN::setAutoBusOffRecovery(bool enabled)
{
  _can.handle.Init.AutoBusOff = enabled ? (ENABLE) : (DISABLE);
}
#endif

#if defined(HAL_FDCAN_MODULE_ENABLED)

void STM32_CAN::setFrameFormat(FRAME_FORMAT format)
{
  _can.handle.Init.FrameFormat = format;
}

void STM32_CAN::setTransmitPause(bool enabled)
{
  _can.handle.Init.TransmitPause = enabled ? (ENABLE) : (DISABLE);
}

void STM32_CAN::setProtocolException(bool enabled)
{
  _can.handle.Init.ProtocolException = enabled ? (ENABLE) : (DISABLE);
}

void STM32_CAN::setExtIdAndMask(uint32_t mask)
{
  this->extIdAndMask = mask;
}

void STM32_CAN::setFilterGlobalNonMatching(FILTER_ACTION actionStd, FILTER_ACTION actionExt)
{
  if(actionStd == STORE_FIFO0 || actionStd == STORE_FIFO1 || actionStd == REJECT )
  {
    this->actionStd = actionStd;
  }
  if(actionExt == STORE_FIFO0 || actionExt == STORE_FIFO1 || actionExt == REJECT )
  {
    this->actionExt = actionExt;
  }
}
void STM32_CAN::setFilterGlobalRTR(bool rejectStdRTR, bool rejectExtRTR)
{
  this->rejectStdRTR = rejectStdRTR;
  this->rejectExtRTR = rejectExtRTR;
}

void STM32_CAN::setCommonClockDiv(uint8_t div)
{
  commonClockDivRegValue = (div >> 1) & 0x0FUL; //divider is reg*2, div/1 is 0

  /* in case FDCAN1 was already initialized directly apply to init values
   * Prescaler value will be updated by HAL when FDCAN1 begin() is called. */
  #ifndef STM32_FDCAN_HAS_CLOCK_CALIBRATION_UNIT
  /** if CCU is present, clockdiv get initialized differently */
  if(canObj[CAN1_INDEX])
  {
    canObj[CAN1_INDEX]->handle.Init.ClockDivider = commonClockDivRegValue;
  }
  #endif
  /** Also directly update prescaler value in case FDCAN1 is not running
   * This should allow setting the prescaler value for use with other FDCAN instance than FDCAN1
   * In case its running we can't do anything without disturbing FDCAN1, up to user to restart FDCAN1.
   */
  if((FDCAN1->CCCR & FDCAN_CCCR_INIT))
  {
    /* Enable configuration change */
    SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_CCE);

    #ifdef STM32_FDCAN_HAS_CLOCK_CALIBRATION_UNIT
    /** Bypass clock calibration unit */
    SET_BIT(FDCAN_CCU->CCFG, FDCANCCU_CCFG_BCC);
    /** NOTE: Clock calibration value has CDIV at different location, but with same encoding */
    MODIFY_REG(FDCAN_CCU->CCFG, FDCANCCU_CCFG_CDIV, (commonClockDivRegValue << FDCANCCU_CCFG_CDIV_Pos));
    #else
    FDCAN_CONFIG->CKDIV = commonClockDivRegValue;
    #endif
  }
}

#endif

/**-------------------------------------------------------------
 *     lifecycle functions
 *     setBaudRate may be called before or after begin
 * -------------------------------------------------------------
 */
// Init and start CAN
void STM32_CAN::begin( bool retransmission ) {

  // exit if CAN already is active
  if (_canIsActive) return;

  _can.handle.Instance = getPeripheral();
  if(_can.handle.Instance == NP)
  {
    //impossible pinconfig, done here
    _can.handle.Instance = nullptr;
    return;
  }
  if(!allocatePeripheral())
  {
    //peripheral already in use
    return;
  }

  _canIsActive = true;

  initializeBuffers();
  
  pin_function(rx, pinmap_function(rx, PinMap_CAN_RD));
  if(tx != NC)
  {
    pin_function(tx, pinmap_function(tx, PinMap_CAN_TD));
  }

  // Configure CAN
  #if defined(HAL_CAN_MODULE_ENABLED)
  if (_can.handle.Instance == CAN1)
  {
    //CAN1
    __HAL_RCC_CAN1_CLK_ENABLE();

    #ifdef CAN1_IRQn_AIO
    // NVIC configuration for CAN1 common interrupt
    HAL_NVIC_SetPriority(CAN1_IRQn_AIO, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_IRQn_AIO);
    #else
    #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    /** CAN Tx and Rx0 blocked by USB, only using Rx1 */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn,  preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    #else
    // NVIC configuration for CAN1 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn );
    // NVIC configuration for CAN1 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN1_TX_IRQn,  preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    #endif
    #endif /** else defined(CAN1_IRQn_AIO) */

    _can.bus = 1;
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    //CAN2
    __HAL_RCC_CAN1_CLK_ENABLE(); // CAN1 clock needs to be enabled too, because CAN2 works as CAN1 slave.
    __HAL_RCC_CAN2_CLK_ENABLE();

    // NVIC configuration for CAN2 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn );
    // NVIC configuration for CAN2 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN2_TX_IRQn,  preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);

    _can.bus = 2;
  }
#endif

#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    //CAN3
    __HAL_RCC_CAN3_CLK_ENABLE();

    // NVIC configuration for CAN3 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn );
    // NVIC configuration for CAN3 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN3_TX_IRQn,  preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);

    _can.bus = 3;
  }
#endif

#elif defined(HAL_FDCAN_MODULE_ENABLED)
#if defined(FDCAN1)
  if (_can.handle.Instance == FDCAN1) {
    __HAL_RCC_FDCAN_CLK_ENABLE();

#if defined(STM32G0xx)
    HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_SetPriority(TIM17_FDCAN_IT1_IRQn, preemptPriority, subPriority);
    phfdcan1 = &_can.handle;
    HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
#else
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif
    _can.bus = 1;
  }
#endif
#if defined(FDCAN2)
  else if (_can.handle.Instance == FDCAN2) {
    __HAL_RCC_FDCAN_CLK_ENABLE();

#if defined(STM32G0xx)
    HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_SetPriority(TIM17_FDCAN_IT1_IRQn, preemptPriority, subPriority);
    phfdcan2 = &_can.handle;
    HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
#else
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
#endif
    _can.bus = 2;
  }
#endif
#if defined(FDCAN3)
  else if (_can.handle.Instance == FDCAN3) {
    __HAL_RCC_FDCAN_CLK_ENABLE();

    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_SetPriority(FDCAN3_IT1_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    _can.bus = 3;
  }
#endif
#endif


  setAutoRetransmission(retransmission);
  
  filtersInitialized = false;

  //try to start in case baudrate was set earlier
  if(!calculateBaudrate( baudrate ))
  {
    return;
  }
  start();
}

void STM32_CAN::end()
{
  if(!hasPeripheral())
  {
    return;
  }

  stop();
  
  disableMBInterrupts();

#if defined(HAL_CAN_MODULE_ENABLED)
  if (_can.handle.Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    __HAL_RCC_CAN2_CLK_DISABLE();
    //only disable CAN1 clock if its not used
    if(canObj[CAN1_INDEX] == nullptr)
    {
      __HAL_RCC_CAN1_CLK_DISABLE();
    } 
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    __HAL_RCC_CAN3_CLK_DISABLE();
  }
#endif
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  /** NOTE: all platforms have common clock enable signal,
   * only stop clock if this is the last active instance */
  uint8_t activeCan = 0;
  activeCan += (canObj[CAN1_INDEX] != nullptr) ? 1 : 0;
  #ifdef FDCAN2
  activeCan += (canObj[CAN2_INDEX] != nullptr) ? 1 : 0;
  #endif
  #ifdef FDCAN3
  activeCan += (canObj[CAN3_INDEX] != nullptr) ? 1 : 0;
  #endif
  if(activeCan == 1)
  {
    // we are last active instance, shutdown
    __HAL_RCC_FDCAN_CLK_DISABLE();
  }
#endif

  /** un-init pins, enable tx PULLUP for weak driving of recessive state */
  pin_function(rx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, GPIO_AF_NONE));
  if(tx != NC)
    pin_function(tx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, GPIO_AF_NONE));

  freeBuffers();

  freePeripheral();

  _canIsActive = false;
}

void STM32_CAN::setBaudRate(uint32_t baud)
{
  baudrate = baud;

  if(!hasPeripheral())
  {
    return;
  }

  // Calculate and set baudrate
  if(!calculateBaudrate( baud ))
  {
    return;
  }

  /** just update baudrate regs if running */
#if defined(HAL_CAN_MODULE_ENABLED)
  HAL_CAN_StateTypeDef state = HAL_CAN_GetState(&_can.handle);
  if (  (state == HAL_CAN_STATE_READY)
     || (state == HAL_CAN_STATE_LISTENING))

#elif defined(HAL_FDCAN_MODULE_ENABLED)
  HAL_FDCAN_StateTypeDef state = HAL_FDCAN_GetState(&_can.handle);
  if (  (state == HAL_FDCAN_STATE_READY)
     || (state == HAL_FDCAN_STATE_BUSY))
#endif
  {
    updateBaudrateRegisters();
  }
  else
  {
    start();
  }
}

bool STM32_CAN::updateBaudrateRegisters()
{
  if(!_can.handle.Instance) return false;
  uint32_t tickstart;
  
#if defined(HAL_CAN_MODULE_ENABLED)
  bool wasStarted = !(_can.handle.Instance->MCR & CAN_MCR_INRQ);

  /* Request initialisation */
  SET_BIT(_can.handle.Instance->MCR, CAN_MCR_INRQ);
  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait initialisation acknowledge */
  while ((_can.handle.Instance->MSR & CAN_MSR_INAK) == 0U)
  {
    if ((HAL_GetTick() - tickstart) > STM32_CAN_TIMEOUT_VALUE)
    {
      return false;
    }
  }

    /* Set the bit timing register */
  WRITE_REG(_can.handle.Instance->BTR, (uint32_t)(_can.handle.Init.Mode           |
                                                  _can.handle.Init.SyncJumpWidth  |
                                                  _can.handle.Init.TimeSeg1       |
                                                  _can.handle.Init.TimeSeg2       |
                                                  (_can.handle.Init.Prescaler - 1U)));
  if(wasStarted)
  {
  /* Request leave initialisation */
    CLEAR_BIT(_can.handle.Instance->MCR, CAN_MCR_INRQ);
  }
  
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  bool wasStarted = !(_can.handle.Instance->CCCR & FDCAN_CCCR_INIT);

  /* Request initialisation */
  SET_BIT(_can.handle.Instance->CCCR, FDCAN_CCCR_INIT);

  tickstart = HAL_GetTick();
  /* Wait until the INIT bit into CCCR register is set */
  while ((_can.handle.Instance->CCCR & FDCAN_CCCR_INIT) == 0U)
  {
    /* Check for the Timeout */
    if ((HAL_GetTick() - tickstart) > STM32_CAN_TIMEOUT_VALUE)
    {
      return false;
    }
  }

  /* Enable configuration change */
  SET_BIT(_can.handle.Instance->CCCR, FDCAN_CCCR_CCE);

    /* Set the nominal bit timing register */
  _can.handle.Instance->NBTP = ((((uint32_t)_can.handle.Init.NominalSyncJumpWidth - 1U) << FDCAN_NBTP_NSJW_Pos) | \
                            (((uint32_t)_can.handle.Init.NominalTimeSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos)    | \
                            (((uint32_t)_can.handle.Init.NominalTimeSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos)    | \
                            (((uint32_t)_can.handle.Init.NominalPrescaler - 1U) << FDCAN_NBTP_NBRP_Pos));

  /* If FD operation with BRS is selected, set the data bit timing register */
  if (_can.handle.Init.FrameFormat == FDCAN_FRAME_FD_BRS)
  {
    _can.handle.Instance->DBTP = ((((uint32_t)_can.handle.Init.DataSyncJumpWidth - 1U) << FDCAN_DBTP_DSJW_Pos)  | \
                              (((uint32_t)_can.handle.Init.DataTimeSeg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos)     | \
                              (((uint32_t)_can.handle.Init.DataTimeSeg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos)     | \
                              (((uint32_t)_can.handle.Init.DataPrescaler - 1U) << FDCAN_DBTP_DBRP_Pos));
  }

  if(wasStarted)
  {
    /* Request leave initialisation */
    CLEAR_BIT(_can.handle.Instance->CCCR, FDCAN_CCCR_INIT);
  }
#endif
  return true;
}

void STM32_CAN::start()
{
  // Initializes CAN
#if defined(HAL_CAN_MODULE_ENABLED)
  HAL_CAN_Init( &_can.handle );

#elif defined(HAL_FDCAN_MODULE_ENABLED)
  HAL_FDCAN_Init( &_can.handle );
  HAL_FDCAN_ConfigExtendedIdMask(&_can.handle, extIdAndMask);
  HAL_FDCAN_ConfigGlobalFilter(&_can.handle, this->actionStd, this->actionExt,
    this->rejectStdRTR ? FDCAN_REJECT_REMOTE : FDCAN_FILTER_REMOTE,
    this->rejectExtRTR ? FDCAN_REJECT_REMOTE : FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ConfigRxFifoOverwrite(&_can.handle,
    FDCAN_RX_FIFO0, fifo0locked ? FDCAN_RX_FIFO_BLOCKING : FDCAN_RX_FIFO_OVERWRITE);
  HAL_FDCAN_ConfigRxFifoOverwrite(&_can.handle,
    FDCAN_RX_FIFO1, fifo1locked ? FDCAN_RX_FIFO_BLOCKING : FDCAN_RX_FIFO_OVERWRITE);

  if(timestampCounterEnabled)
    HAL_FDCAN_EnableTimestampCounter(&_can.handle, FDCAN_TIMESTAMP_INTERNAL);
  else
    HAL_FDCAN_DisableTimestampCounter(&_can.handle);

  #ifdef STM32_FDCAN_HAS_CLOCK_CALIBRATION_UNIT
  /** Don't use Clock Calibration Unit, just set the prescaler */
  FDCAN_ClkCalUnitTypeDef CCU_conf;
  CCU_conf.ClockCalibration = FDCAN_CLOCK_CALIBRATION_DISABLE;
  CCU_conf.ClockDivider = commonClockDivRegValue;
  CCU_conf.MinOscClkPeriods = 0;
  CCU_conf.CalFieldLength = FDCAN_CALIB_FIELD_LENGTH_32;
  CCU_conf.TimeQuantaPerBitTime = 4;
  CCU_conf.WatchdogStartValue = 0;
  HAL_FDCAN_ConfigClockCalibration(&_can.handle, &CCU_conf);
  #endif
#endif

  initializeFilters();

  // Start the CAN peripheral
#if defined(HAL_CAN_MODULE_ENABLED)
  HAL_CAN_Start( &_can.handle );
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  HAL_FDCAN_Start( &_can.handle );
#endif

  // Activate CAN notifications
#if defined(HAL_CAN_MODULE_ENABLED)
  #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  HAL_CAN_ActivateNotification( &_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
  #else
  HAL_CAN_ActivateNotification( &_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification( &_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  #endif
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  HAL_FDCAN_ActivateNotification( &_can.handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0 /*ignored arg for this event*/);
  HAL_FDCAN_ActivateNotification( &_can.handle, FDCAN_IT_TX_FIFO_EMPTY, 0 /*ignored arg for this event*/);
  #endif
}

void STM32_CAN::stop()
{
#if defined(HAL_CAN_MODULE_ENABLED)
  #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  HAL_CAN_DeactivateNotification( &_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
  #else
  HAL_CAN_DeactivateNotification( &_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_DeactivateNotification( &_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  #endif
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  HAL_FDCAN_DeactivateNotification( &_can.handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  HAL_FDCAN_DeactivateNotification( &_can.handle, FDCAN_IT_TX_FIFO_EMPTY);
#endif

  /** Calls Stop internally, clears all errors */
  #if defined(HAL_CAN_MODULE_ENABLED)
  HAL_CAN_DeInit( &_can.handle );
  #elif defined(HAL_FDCAN_MODULE_ENABLED)
  HAL_FDCAN_DeInit( &_can.handle );
  #endif
}


/**-------------------------------------------------------------
 *     post begin(), setup filters, data transfer
 * -------------------------------------------------------------
 */

#if defined(HAL_CAN_MODULE_ENABLED)
bool STM32_CAN::write(CAN_message_t &CAN_tx_msg, bool sendMB)
{
  bool ret = true;
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;
  if(!_can.handle.Instance) return false;

  #if !defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB)
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  #endif

  if (CAN_tx_msg.flags.extended == 1) // Extended ID when CAN_tx_msg.flags.extended is 1
  {
      TxHeader.ExtId = CAN_tx_msg.id;
      TxHeader.IDE   = CAN_ID_EXT;
  }
  else // Standard ID otherwise
  {
      TxHeader.StdId = CAN_tx_msg.id;
      TxHeader.IDE   = CAN_ID_STD;
  }

  if (CAN_tx_msg.flags.remote == 1) // Remote frame when CAN_tx_msg.flags.remote is 1
  {
    TxHeader.RTR   = CAN_RTR_REMOTE;
    TxHeader.DLC   = 0;
  }
  else{
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = CAN_tx_msg.len;
  }

  TxHeader.TransmitGlobalTime = DISABLE;

  if(HAL_CAN_AddTxMessage( &_can.handle, &TxHeader, CAN_tx_msg.buf, &TxMailbox) != HAL_OK)
  {
    /* in normal situation we add up the message to TX ring buffer, if there is no free TX mailbox. But the TX mailbox interrupt is using this same function
    to move the messages from ring buffer to empty TX mailboxes, so for that use case, there is this check */
    if(sendMB != true)
    {
      if( addToRingBuffer(txRing, CAN_tx_msg) == false )
      {
        ret = false; // no more room
      }
    }
    else { ret = false; }
  }

  #if !defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB)
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  #endif
  return ret;
}

#elif defined(HAL_FDCAN_MODULE_ENABLED)

static const uint8_t DLCtoLength[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static uint8_t lengthToDLC(uint8_t length)
{
  if(length <= 8)
  {
    return length;
  }
  else if(length <= 24)
  {
    return  8 + ((length-5) >> 2);
  }
  else
  {
    return (12 + ((length-1) >> 4)) & 0x0FUL;
  }
}

bool STM32_CAN::write(CAN_message_t &CAN_tx_msg, bool sendMB)
{
  bool ret = true;
  FDCAN_TxHeaderTypeDef TxHeader;
  if(!_can.handle.Instance) return false;

  __HAL_FDCAN_DISABLE_IT(&_can.handle, FDCAN_IT_TX_FIFO_EMPTY);

  if (CAN_tx_msg.flags.extended)
  {
      TxHeader.Identifier = CAN_tx_msg.id & 0x1FFFFFFFU;
      TxHeader.IdType   = FDCAN_EXTENDED_ID;
  }
  else
  {
      TxHeader.Identifier = CAN_tx_msg.id & 0x7FF;
      TxHeader.IdType   = FDCAN_STANDARD_ID;
  }

  if (CAN_tx_msg.flags.remote)
  {
    TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
    TxHeader.DataLength  = 0;
  }
  else{
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength  = lengthToDLC(min(CAN_tx_msg.len, (uint8_t)8UL));
  }

  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if(HAL_FDCAN_AddMessageToTxFifoQ( &_can.handle, &TxHeader, CAN_tx_msg.buf) != HAL_OK)
  {
    /* in normal situation we add up the message to TX ring buffer, if there is no free TX mailbox. But the TX mailbox interrupt is using this same function
    to move the messages from ring buffer to empty TX mailboxes, so for that use case, there is this check */
    if(sendMB != true)
    {
      if( addToRingBuffer(txRing, CAN_tx_msg) == false )
      {
        ret = false; // no more room
      }
    }
    else { ret = false; }
  }
  __HAL_FDCAN_ENABLE_IT(&_can.handle, FDCAN_IT_TX_FIFO_EMPTY);
  return ret;
}
#endif /* elif defined(HAL_FDCAN_MODULE_ENABLED) */

bool STM32_CAN::read(CAN_message_t &CAN_rx_msg)
{
  bool ret;
  if(!_can.handle.Instance) return false;

#if defined(HAL_CAN_MODULE_ENABLED)
  #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
  #else
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  #endif
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  __HAL_FDCAN_DISABLE_IT(&_can.handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#endif

  ret = removeFromRingBuffer(rxRing, CAN_rx_msg);

#if defined(HAL_CAN_MODULE_ENABLED)
  #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
  #else
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  #endif
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  __HAL_FDCAN_ENABLE_IT(&_can.handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#endif
  return ret;
}


#if defined(HAL_CAN_MODULE_ENABLED)

uint8_t STM32_CAN::getFilterBankCount(IDE std_ext)
{
  (void)std_ext;
  if(_can.handle.Instance == nullptr) return 0;
  #ifdef CAN2
  if(_can.handle.Instance == CAN1)
  {
    return STM32_CAN_CAN2_FILTER_OFFSET;
  }
  if(_can.handle.Instance == CAN2)
  {
    return STM32_CAN_DUAL_CAN_FILTER_COUNT - STM32_CAN_CAN2_FILTER_OFFSET;
  }
  #endif
  return STM32_CAN_SINGLE_CAN_FILTER_COUNT;
}

static uint32_t format32bitFilter(uint32_t id, IDE std_ext, bool mask)
{
  uint32_t id_reg;
  if (std_ext == AUTO)
  {
    std_ext = (id <= 0x7FF) ? STD : EXT;
  }
  if (std_ext == STD)
  {
    id <<= 18;
  }
  id_reg = id << 3;
  //set IDE bit
  if (mask || std_ext == EXT)
  {
    id_reg |= (1 << 2);
  }
  return id_reg;
}

static uint32_t format16bitFilter(uint32_t id, IDE std_ext, bool mask)
{
  uint32_t id_reg;
  if (std_ext == AUTO)
  {
    std_ext = (id <= 0x7FF) ? STD : EXT;
  }
  if (std_ext == STD)
  {
    id <<= 18;
  }
  //set STID
  id_reg =  (id >> (18-5)) & 0xFFE0UL;
  //set EXTI [17:15]
  id_reg |= (id >> (18-3)) & 0x003UL;
  //set IDE bit
  if (mask || std_ext == EXT)
  {
    id_reg |= (1 << 3);
  }
  return id_reg;
}

bool STM32_CAN::setFilter(uint8_t bank_num, bool enabled, FILTER_ACTION action)
{
  CAN_TypeDef *can_ip = _can.handle.Instance;
  if(!_can.handle.Instance) return false;
  /** CAN2 shares filter banks with CAN1
   * Driver allocates equal amount to each
   * Filter Banks located at CAN1 base address
  */
  #ifdef CAN2
  if(_can.handle.Instance == CAN2)
  {
    can_ip = CAN1;
    bank_num += STM32_CAN_CAN2_FILTER_OFFSET;
  }
  #endif

  uint32_t filternbrbitpos = (uint32_t)1 << (bank_num & 0x1FU);

  /* Initialisation mode for the filter */
  SET_BIT(can_ip->FMR, CAN_FMR_FINIT);
  
  /* Filter Deactivation */
  CLEAR_BIT(can_ip->FA1R, filternbrbitpos);

  /* Filter FIFO assignment */
  switch (action)
  {
    case FILTER_ACTION::STORE_FIFO0:
      CLEAR_BIT(can_ip->FFA1R, filternbrbitpos);
      break;
    case FILTER_ACTION::STORE_FIFO1:
      SET_BIT(can_ip->FFA1R, filternbrbitpos);
      break;
  }

  /* Filter activation */
  if(enabled)
  {
    SET_BIT(can_ip->FA1R, filternbrbitpos);
  }
  /* Leave the initialisation mode for the filter */
  CLEAR_BIT(can_ip->FMR, CAN_FMR_FINIT);
  return true;
}

bool STM32_CAN::setFilterSingleMask(uint8_t bank_num, uint32_t id, uint32_t mask, IDE std_ext, FILTER_ACTION action, bool enabled)
{
  uint32_t id_reg   = format32bitFilter(id,   std_ext, false);
  uint32_t mask_reg = format32bitFilter(mask, std_ext, true);
  return setFilterRaw(bank_num, id_reg, mask_reg, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, action, enabled);
}

bool STM32_CAN::setFilterDualID(uint8_t bank_num, uint32_t id1, uint32_t id2, IDE std_ext1, IDE std_ext2, FILTER_ACTION action, bool enabled)
{
  uint32_t id   = format32bitFilter(id1, std_ext1, false);
  uint32_t mask = format32bitFilter(id2, std_ext2, false);
  return setFilterRaw(bank_num, id, mask, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_32BIT, action, enabled);
}

bool STM32_CAN::setFilterDualMask(uint8_t bank_num, uint32_t id1, uint32_t mask1, IDE std_ext1, uint32_t id2, uint32_t mask2, IDE std_ext2, FILTER_ACTION action, bool enabled)
{
  uint32_t id   = (uint32_t)format16bitFilter(id1, std_ext1, false) | (((uint32_t)format16bitFilter(mask1, std_ext1, true)) << 16);
  uint32_t mask = (uint32_t)format16bitFilter(id2, std_ext2, false) | (((uint32_t)format16bitFilter(mask2, std_ext2, true)) << 16);
  return setFilterRaw(bank_num, id, mask, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_16BIT, action, enabled);
}

bool STM32_CAN::setFilterQuadID(uint8_t bank_num, uint32_t id1, IDE std_ext1, uint32_t id2, IDE std_ext2, uint32_t id3, IDE std_ext3, uint32_t id4, IDE std_ext4, FILTER_ACTION action, bool enabled)
{
  uint32_t id   = (uint32_t)format16bitFilter(id1, std_ext1, false) | (((uint32_t)format16bitFilter(id2, std_ext2, false)) << 16);
  uint32_t mask = (uint32_t)format16bitFilter(id3, std_ext3, false) | (((uint32_t)format16bitFilter(id4, std_ext4, false)) << 16);
  return setFilterRaw(bank_num, id, mask, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_16BIT, action, enabled);
}

bool STM32_CAN::setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, IDE std_ext, uint32_t filter_mode, uint32_t filter_scale, uint32_t fifo)
{
  /** NOTE: legacy, this function only implemented 32 bit scaling mode in mask mode, other modes will be broken*/
  if(filter_scale != CAN_FILTERSCALE_32BIT)
  {
    core_debug("WARNING: legacy function only implements 32 bit filter scale. Filter will be broken!\n");
  }
  if(filter_scale != CAN_FILTERMODE_IDMASK)
  {
    core_debug("WARNING: legacy function only implements ID Mask mode. Filter will be broken!\n");
  }
  /** re-implement broken implementation for legacy behaviour */
  uint32_t id_reg   = format32bitFilter(filter_id, std_ext, false);
  uint32_t mask_reg = format32bitFilter(mask,      std_ext, true);
  FILTER_ACTION action = (fifo==CAN_FILTER_FIFO0) ? FILTER_ACTION::STORE_FIFO0 : FILTER_ACTION::STORE_FIFO1;
  return !setFilterRaw(bank_num, id_reg, mask_reg, filter_mode, filter_scale, action);
}

bool STM32_CAN::setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask, uint32_t filter_mode, uint32_t filter_scale, FILTER_ACTION action, bool enabled)
{
  CAN_FilterTypeDef sFilterConfig;
  if(!_can.handle.Instance) return false;

  sFilterConfig.FilterBank = bank_num;
  sFilterConfig.FilterMode = filter_mode;
  sFilterConfig.FilterScale = filter_scale;
  #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  if(action == FILTER_ACTION::STORE_FIFO0)
  {
    core_debug("WARNING: RX0 IRQ is blocked by USB Driver. Events only handled by polling and RX1 events!\n");
  }
  #endif
  switch (action)
  {
    case FILTER_ACTION::STORE_FIFO0:
      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      break;
    case FILTER_ACTION::STORE_FIFO1:
      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
      break;
  }
  sFilterConfig.FilterActivation = enabled ? ENABLE : DISABLE;

  sFilterConfig.FilterIdLow      = id   & 0xFFFFUL;
  sFilterConfig.FilterIdHigh     = id   >> 16;
  sFilterConfig.FilterMaskIdLow  = mask & 0xFFFFUL;
  sFilterConfig.FilterMaskIdHigh = mask >> 16;

  #ifdef CAN2
  sFilterConfig.SlaveStartFilterBank = STM32_CAN_CAN2_FILTER_OFFSET;
  if(_can.handle.Instance == CAN2)
  {
    sFilterConfig.FilterBank += STM32_CAN_CAN2_FILTER_OFFSET;
    if(sFilterConfig.FilterBank >= STM32_CAN_DUAL_CAN_FILTER_COUNT)
      return false;
  }
  else
  #endif
  if(sFilterConfig.FilterBank >= STM32_CAN_SINGLE_CAN_FILTER_COUNT)
  {
    return false;
  }
  // Enable filter
  return (HAL_CAN_ConfigFilter( &_can.handle, &sFilterConfig ) == HAL_OK);
}

#elif defined(HAL_FDCAN_MODULE_ENABLED)

uint8_t STM32_CAN::getFilterBankCount(IDE std_ext)
{
  if(_can.handle.Instance == nullptr) return 0;
  if(std_ext == STD)
    return _can.handle.Init.StdFiltersNbr;
  else if(std_ext == EXT)
    return _can.handle.Init.ExtFiltersNbr;
  else
    return 0;
}


bool STM32_CAN::setFilter(uint8_t bank_num, bool enabled, FILTER_ACTION action, IDE std_ext)
{
  uint32_t * pFilter;
  uint32_t config = FDCAN_FILTER_DISABLE;
  if(!_can.handle.Instance) return false;
  if(enabled)
  {
    switch (action)
    {
    case STORE_FIFO0:
      config = FDCAN_FILTER_TO_RXFIFO0;
      break;
    case STORE_FIFO1:
      config = FDCAN_FILTER_TO_RXFIFO1;
      break;
    case REJECT:
      config = FDCAN_FILTER_REJECT;
      break;
    case HIGH_PRIORITY:
      config = FDCAN_FILTER_HP;
      break;
    case STORE_FIFO0_HIGH_PRIORITY:
      config = FDCAN_FILTER_TO_RXFIFO0_HP;
      break;
    case STORE_FIFO1_HIGH_PRIORITY:
      config = FDCAN_FILTER_TO_RXFIFO1_HP;
      break;
    default:
      config = FDCAN_FILTER_DISABLE;
      break;
    }
  }

  switch (std_ext)
  {
    case STD:
      if( bank_num >= _can.handle.Init.StdFiltersNbr) return false;
      pFilter = (uint32_t *)(_can.handle.msgRam.StandardFilterSA + (bank_num * STM32_FDCAN_SF_Size));
      pFilter[0] = (pFilter[0] & ~(STM32_FDCAN_SF_EC_Msk))
                | ((config << (STM32_FDCAN_SF_EC_Pos)) & STM32_FDCAN_SF_EC_Msk);
      break;
    case EXT:
      if( bank_num >= _can.handle.Init.ExtFiltersNbr) return false;
      pFilter = (uint32_t *)(_can.handle.msgRam.ExtendedFilterSA + (bank_num * STM32_FDCAN_EF_Size));
      pFilter[0] = (pFilter[0] & ~(STM32_FDCAN_EF_W0_EC_Msk))
                | ((config << (STM32_FDCAN_EF_W0_EC_Pos)) & STM32_FDCAN_EF_W0_EC_Msk);
      break;
    default:
      return false;
  }
  return true;
}

bool STM32_CAN::setFilterSingleMask(uint8_t bank_num, uint32_t id, uint32_t mask, IDE std_ext, FILTER_ACTION action, bool enabled)
{
  return setFilterRaw(bank_num, id, mask, std_ext, FDCAN_FILTER_MASK, action, enabled);
}

/** NOTE: FDCAN does not support dual filter with different id types. std_ext1 has to be equal to std_ext2 */
bool STM32_CAN::setFilterDualID(uint8_t bank_num, uint32_t id1, uint32_t id2, IDE std_ext1, IDE std_ext2, FILTER_ACTION action, bool enabled)
{
  bool std1 = (std_ext1 == STD || (std_ext1 == AUTO && id1 <= 0x7FFUL));
  bool std2 = (std_ext2 == STD || (std_ext2 == AUTO && id2 <= 0x7FFUL));
  if(std1 != std2)
  {
    core_debug("ERROR: Dual ID Filter have to use same ID type!\n");
  }
  return setFilterRaw(bank_num, id1, id2, std_ext1, FDCAN_FILTER_DUAL, action, enabled);
}

bool STM32_CAN::setFilterRange(uint8_t bank_num, uint32_t id1, uint32_t id2, IDE std_ext, FILTER_ACTION action, bool enabled, bool nEIDM)
{
  bool std = (std_ext == STD || (std_ext == AUTO && id1 <= 0x7FFUL && id2 <= 0x7FFUL));
  if(std && nEIDM)
  {
    core_debug("ERROR: Can not apply extended ID and mask register with standard ids!\n");
    nEIDM = false;
  }
  return setFilterRaw(bank_num, id1, id2, std_ext, nEIDM ? FDCAN_FILTER_RANGE_NO_EIDM : FDCAN_FILTER_RANGE, action, enabled);
}

bool STM32_CAN::setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask, IDE std_ext, uint32_t filter_type, FILTER_ACTION action, bool enabled)
{
  FDCAN_FilterTypeDef sFilterConfig;
  if(!_can.handle.Instance) return false;

  sFilterConfig.FilterIndex = bank_num;
  sFilterConfig.FilterType = filter_type;

  sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
  if(enabled) {
    switch(action)
    {
      case STORE_FIFO0:
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        break;
      case STORE_FIFO1:
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        break;
      case REJECT:
        sFilterConfig.FilterConfig = FDCAN_FILTER_REJECT;
        break;
      case HIGH_PRIORITY:
        sFilterConfig.FilterConfig = FDCAN_FILTER_HP;
        break;
      case STORE_FIFO0_HIGH_PRIORITY:
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
        break;
      case STORE_FIFO1_HIGH_PRIORITY:
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
        break;
    }
  }

  if (std_ext == STD || (std_ext == AUTO && id <= 0x7FFUL))
  {
    if( bank_num >= _can.handle.Init.StdFiltersNbr) return false;
    /** Unapplicable filter type for standard filters, set to normal range */
    if(sFilterConfig.FilterType == FDCAN_FILTER_RANGE_NO_EIDM)
    {
      sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    }
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    /** Mask relevant bits in case user supplies wider mask than allowed (HAL does not mask) */
    sFilterConfig.FilterID1 = id   & 0x7FFUL;
    sFilterConfig.FilterID2 = mask & 0x7FFUL;
  }
  else
  {
    if( bank_num >= _can.handle.Init.ExtFiltersNbr) return false;
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    /** Mask relevant bits in case user supplies wider mask than allowed (HAL does not mask) */
    sFilterConfig.FilterID1 = id   & 0x1FFFFFFFUL;
    sFilterConfig.FilterID2 = mask & 0x1FFFFFFFUL;
  }

  return (HAL_FDCAN_ConfigFilter( &_can.handle, &sFilterConfig ) == HAL_OK);
}

#endif /* elif defined(HAL_FDCAN_MODULE_ENABLED) */


/**-------------------------------------------------------------
 *     Teensy FlexCAN compatibility functions
 * -------------------------------------------------------------
 */

void STM32_CAN::setMBFilter(CAN_BANK bank_num, CAN_FLTEN input)
{
  setFilter(bank_num, (input == ACCEPT_ALL));
}

void STM32_CAN::setMBFilter(CAN_FLTEN input)
{
#if defined(HAL_CAN_MODULE_ENABLED)
  for (uint8_t bank_num = 0 ; bank_num < STM32_CAN_SINGLE_CAN_FILTER_COUNT ; bank_num++)
  {
    setFilter(bank_num, (input == ACCEPT_ALL));
  }
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  /** NOTE: this does overwrite the filter action with default, but this lagecy API
   *  does not allow setting FIFO target, so default would be uses anyway*/
  for (uint8_t bank_num = 0 ; bank_num < _can.handle.Init.StdFiltersNbr ; bank_num++)
  {
    setFilter(bank_num, (input == ACCEPT_ALL));
  }
  for (uint8_t bank_num = 0 ; bank_num < _can.handle.Init.ExtFiltersNbr ; bank_num++)
  {
    setFilter(bank_num, (input == ACCEPT_ALL), CAN_FILTER_DEFAULT_ACTION, EXT);
  }
#endif
}

bool STM32_CAN::setMBFilterProcessing(CAN_BANK bank_num, uint32_t filter_id, uint32_t mask, IDE std_ext)
{
  // just convert the MB number enum to bank number.
  return !setFilterSingleMask(uint8_t(bank_num), filter_id, mask, std_ext);
}

bool STM32_CAN::setMBFilter(CAN_BANK bank_num, uint32_t id1, IDE std_ext)
{
  // by setting the mask to 0x1FFFFFFF we only filter the ID set as Filter ID.
  return !setFilterSingleMask(uint8_t(bank_num), id1, 0x1FFFFFFF, std_ext);
}

bool STM32_CAN::setMBFilter(CAN_BANK bank_num, uint32_t id1, uint32_t id2, IDE std_ext)
{
  return !setFilterDualID(uint8_t(bank_num), id1, id2, std_ext, std_ext);
}

void STM32_CAN::initializeFilters()
{
  if(filtersInitialized) return;
  filtersInitialized = true;

#if defined(HAL_CAN_MODULE_ENABLED)

  /** Let everything in by default */
  setFilterRaw(0, 0UL, 0UL, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT,
    FILTER_ACTION::CAN_FILTER_DEFAULT_ACTION, true);

  /** turn off all other filters that might sill be setup from before */
  for (uint8_t bank_num = 1 ; bank_num < STM32_CAN_SINGLE_CAN_FILTER_COUNT ; bank_num++)
  {
    setFilter(bank_num, false);
  }

#elif defined(HAL_FDCAN_MODULE_ENABLED)
  /** Let everything in by default */
  setFilterSingleMask(0, 0UL, 0UL, STD);
  setFilterSingleMask(0, 0UL, 0UL, EXT);

  /** turn off all other filters that might sill be setup from before */
  for (uint8_t bank_num = 1 ; bank_num < _can.handle.Init.StdFiltersNbr ; bank_num++)
  {
    setFilter(bank_num, false);
  }
  for (uint8_t bank_num = 1 ; bank_num < _can.handle.Init.ExtFiltersNbr ; bank_num++)
  {
    setFilter(bank_num, false, CAN_FILTER_DEFAULT_ACTION, EXT);
  }
#endif
}


void STM32_CAN::initializeBuffers()
{
    if(isInitialized()) { return; }

    // set up the transmit and receive ring buffers
    if(tx_buffer==0)
    {
      tx_buffer=new CAN_message_t[sizeTxBuffer];
    }
    initRingBuffer(txRing, tx_buffer, sizeTxBuffer);

    if(rx_buffer==0)
    {
      rx_buffer=new CAN_message_t[sizeRxBuffer];
    }
    initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}

void STM32_CAN::freeBuffers()
{
  txRing.head = 0;
  txRing.tail = 0;
  txRing.buffer = nullptr;
  delete[] tx_buffer;
  tx_buffer = nullptr;

  rxRing.head = 0;
  rxRing.tail = 0;
  rxRing.buffer = nullptr;
  delete[] rx_buffer;
  rx_buffer = nullptr;
}

void STM32_CAN::initRingBuffer(RingbufferTypeDef &ring, volatile CAN_message_t *buffer, uint32_t size)
{
    ring.buffer = buffer;
    ring.size = size;
    ring.head = 0;
    ring.tail = 0;
}

bool STM32_CAN::addToRingBuffer(RingbufferTypeDef &ring, const CAN_message_t &msg)
{
    uint16_t nextEntry;
    nextEntry =(ring.head + 1) % ring.size;

    // check if the ring buffer is full
    if(nextEntry == ring.tail)
	{
        return(false);
    }

    // add the element to the ring */
    memcpy((void *)&ring.buffer[ring.head],(void *)&msg, sizeof(CAN_message_t));

    // bump the head to point to the next free entry
    ring.head = nextEntry;

    return(true);
}
bool STM32_CAN::removeFromRingBuffer(RingbufferTypeDef &ring, CAN_message_t &msg)
{
    // check if the ring buffer has data available
    if(isRingBufferEmpty(ring) == true)
    {
        return(false);
    }

    // copy the message
    memcpy((void *)&msg,(void *)&ring.buffer[ring.tail], sizeof(CAN_message_t));

    // bump the tail pointer
    ring.tail =(ring.tail + 1) % ring.size;
    return(true);
}

bool STM32_CAN::isRingBufferEmpty(RingbufferTypeDef &ring)
{
    if(ring.head == ring.tail)
	{
        return(true);
    }

    return(false);
}

uint32_t STM32_CAN::ringBufferCount(RingbufferTypeDef &ring)
{
    int32_t entries;
    entries = ring.head - ring.tail;

    if(entries < 0)
    {
        entries += ring.size;
    }
    return((uint32_t)entries);
}

#if defined(HAL_CAN_MODULE_ENABLED)
void STM32_CAN::setBaudRateValues(uint16_t prescaler, uint8_t timeseg1,
                                  uint8_t timeseg2, uint8_t sjw)
{
  uint32_t _SyncJumpWidth = 0;
  uint32_t _TimeSeg1 = 0;
  uint32_t _TimeSeg2 = 0;
  uint32_t _Prescaler = 0;

  /* the CAN specification (v2.0) states that SJW shall be programmable between
   * 1 and min(4, timeseg1)... the bxCAN documentation doesn't mention this
   */
  if (sjw > 4) sjw = 4;
  if (sjw > timeseg1) sjw = timeseg1;

  switch (sjw)
  {
    case 0:
    case 1:
      _SyncJumpWidth = CAN_SJW_1TQ;
      break;
    case 2:
      _SyncJumpWidth = CAN_SJW_2TQ;
      break;
    case 3:
      _SyncJumpWidth = CAN_SJW_3TQ;
      break;
    case 4:
    default: /* limit to 4 */
      _SyncJumpWidth = CAN_SJW_4TQ;
      break;
  }

  switch (timeseg1)
  {
    case 1:
      _TimeSeg1 = CAN_BS1_1TQ;
      break;
    case 2:
      _TimeSeg1 = CAN_BS1_2TQ;
      break;
    case 3:
      _TimeSeg1 = CAN_BS1_3TQ;
      break;
    case 4:
      _TimeSeg1 = CAN_BS1_4TQ;
      break;
    case 5:
      _TimeSeg1 = CAN_BS1_5TQ;
      break;
    case 6:
      _TimeSeg1 = CAN_BS1_6TQ;
      break;
    case 7:
      _TimeSeg1 = CAN_BS1_7TQ;
      break;
    case 8:
      _TimeSeg1 = CAN_BS1_8TQ;
      break;
    case 9:
      _TimeSeg1 = CAN_BS1_9TQ;
      break;
    case 10:
      _TimeSeg1 = CAN_BS1_10TQ;
      break;
    case 11:
      _TimeSeg1 = CAN_BS1_11TQ;
      break;
    case 12:
      _TimeSeg1 = CAN_BS1_12TQ;
      break;
    case 13:
      _TimeSeg1 = CAN_BS1_13TQ;
      break;
    case 14:
      _TimeSeg1 = CAN_BS1_14TQ;
      break;
    case 15:
      _TimeSeg1 = CAN_BS1_15TQ;
      break;
    case 16:
      _TimeSeg1 = CAN_BS1_16TQ;
      break;
    default:
      // should not happen
      _TimeSeg1 = CAN_BS1_1TQ;
      break;
  }

  switch (timeseg2)
  {
    case 1:
      _TimeSeg2 = CAN_BS2_1TQ;
      break;
    case 2:
      _TimeSeg2 = CAN_BS2_2TQ;
      break;
    case 3:
      _TimeSeg2 = CAN_BS2_3TQ;
      break;
    case 4:
      _TimeSeg2 = CAN_BS2_4TQ;
      break;
    case 5:
      _TimeSeg2 = CAN_BS2_5TQ;
      break;
    case 6:
      _TimeSeg2 = CAN_BS2_6TQ;
      break;
    case 7:
      _TimeSeg2 = CAN_BS2_7TQ;
      break;
    case 8:
      _TimeSeg2 = CAN_BS2_8TQ;
      break;
    default:
      // should not happen
      _TimeSeg2 = CAN_BS2_1TQ;
      break;
  }
  _Prescaler = prescaler;

  _can.handle.Init.SyncJumpWidth = _SyncJumpWidth;
  _can.handle.Init.TimeSeg1 = _TimeSeg1;
  _can.handle.Init.TimeSeg2 = _TimeSeg2;
  _can.handle.Init.Prescaler = _Prescaler;
}

#elif defined(HAL_FDCAN_MODULE_ENABLED)
void STM32_CAN::setBaudRateValues(uint16_t prescaler, uint8_t timeseg1, uint8_t timeseg2, uint8_t sjw)
{
  _can.handle.Init.NominalPrescaler = prescaler;
  _can.handle.Init.NominalSyncJumpWidth = sjw;
  _can.handle.Init.NominalTimeSeg1 = timeseg1;
  _can.handle.Init.NominalTimeSeg2 = timeseg2;
  /** NOTE: Bitrate switching not supported yet. Values are unuesd in non switched modes. */
  _can.handle.Init.DataPrescaler = 1;
  _can.handle.Init.DataSyncJumpWidth = 1;
  _can.handle.Init.DataTimeSeg1 = 1;
  _can.handle.Init.DataTimeSeg2 = 1;
}
#endif

template <typename T, size_t N>
bool STM32_CAN::lookupBaudrate(int baud, const T(&table)[N]) {
  for (size_t i = 0; i < N; i++) {
    if (baud != (int)table[i].baudrate) {
      continue;
    }

    /* for the best chance at interoperability, use the widest SJW possible */
    setBaudRateValues(table[i].prescaler, table[i].timeseg1, table[i].timeseg2, 4);
    return true;
  }

  return false;
}

bool STM32_CAN::calculateBaudrate(int baud)
{
  uint8_t bs1;
  uint8_t bs2;
  uint16_t prescaler;

  const uint32_t frequency = getCanPeripheralClock();

  if (frequency == 48000000) {
    if (lookupBaudrate(baud, BAUD_RATE_TABLE_48M)) return true;
  } else if (frequency == 45000000) {
    if (lookupBaudrate(baud, BAUD_RATE_TABLE_45M)) return true;
  }

  /* this loop seeks a precise baudrate match, with the sample point positioned
   * at between ~75-95%. the nominal bit time is produced from N time quanta,
   * running at the prescaled clock rate (where N = 1 + bs1 + bs2). this algorithm
   * prefers the lowest prescaler (most time quanter per bit).
   *
   * many configuration sets can be discarded due to an out-of-bounds sample point,
   * or being unable to reach the desired baudrate.
   *
   * for the best chance at interoperability, we use the widest SJW possible.
   *
   * for more details + justification, see: https://github.com/pazi88/STM32_CAN/pull/41
   */
#if defined(HAL_CAN_MODULE_ENABLED)
  for (prescaler = 1; prescaler <= 1024; prescaler += 1)
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  for (prescaler = 1; prescaler <= 512; prescaler += 1)
#endif
  {
    const uint32_t can_freq = frequency / prescaler;
    const uint32_t baud_min = can_freq / (1 + 5 + 16);

    /* skip all prescaler values that can't possibly achieve the desired baudrate */
    if (baud_min > baud) continue;

    for (bs2 = 1; bs2 <= 5; bs2 += 1) {
      for (bs1 = (bs2 * 3) - 1; bs1 <= 16; bs1 += 1) {
        const uint32_t baud_cur = can_freq / (1 + bs1 + bs2);

        if (baud_cur != baud) continue;

        setBaudRateValues(prescaler, bs1, bs2, 4);
        return true;
      }
    }
  }

  /* uhoh, failed to calculate an acceptable baud rate... */
  return false;
}

uint32_t STM32_CAN::getCanPeripheralClock()
{
#if defined(HAL_CAN_MODULE_ENABLED)
  //All bxCAN get clocked by APB1 / PCLK1
  return HAL_RCC_GetPCLK1Freq();
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  #ifdef STM32_FDCAN_HAS_CLOCK_CALIBRATION_UNIT
  uint32_t div = (FDCAN_CCU->CCFG & FDCANCCU_CCFG_CDIV) >> FDCANCCU_CCFG_CDIV_Pos;
  #else
  uint32_t div = (FDCAN_CONFIG->CKDIV & FDCAN_CKDIV_PDIV_Msk) >> FDCAN_CKDIV_PDIV_Pos;
  #endif
  //decode register value to devider
  if(!div)
    div = 1;
  else
    div <<= 1;

  return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / div;
#endif
}


#if defined(HAL_CAN_MODULE_ENABLED)

void STM32_CAN::enableMBInterrupts()
{
  if (_can.handle.Instance == CAN1)
  {
    #ifdef CAN1_IRQn_AIO
    HAL_NVIC_EnableIRQ(CAN1_IRQn_AIO);
    #else
    #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    #else
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    #endif
    #endif /** else defined(CAN1_IRQn_AIO) */
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
  }
#endif
}

void STM32_CAN::disableMBInterrupts()
{
  if (_can.handle.Instance == CAN1)
  {
    #ifdef CAN1_IRQn_AIO
    #if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
    //only disable if cec instance is not set/used
    if(!phcec)
    #endif
    {
      HAL_NVIC_DisableIRQ(CAN1_IRQn_AIO);
    }
    #else
    #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    #else
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    #endif
    #endif /** else defined(CAN1_IRQn_AIO) */
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    HAL_NVIC_DisableIRQ(CAN3_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
  }
#endif
}

#elif defined(HAL_FDCAN_MODULE_ENABLED)

#if defined(STM32G0xx)
void STM32_CAN::enableMBInterrupts()
{
  if (_can.handle.Instance == FDCAN1)
  {
    phfdcan1 = &_can.handle;
    HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
  }
#ifdef FDCAN2
  else if (_can.handle.Instance == FDCAN2)
  {
    phfdcan2 = &_can.handle;
    HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
  }
#endif
}

void STM32_CAN::disableMBInterrupts()
{
  if (_can.handle.Instance == FDCAN1)
  {
    phfdcan1 = nullptr;
    /** NOTE: If timer is used and de-init with timerHandleDeinit() 
     * this IRQ is disabled even if phfdcan1 is still set, crippeling the FDCAN.
     * we try to be nice here and only disable IRQ if nothing else is using it anymore */
    if(!HardwareTimer_Handle[TIMER16_INDEX] && !phfdcan2)
      HAL_NVIC_DisableIRQ(TIM16_FDCAN_IT0_IRQn);
  }
#ifdef FDCAN2
  else if (_can.handle.Instance == FDCAN2)
  {
    phfdcan2 = nullptr;
    if(!HardwareTimer_Handle[TIMER16_INDEX] && !phfdcan1)
      HAL_NVIC_DisableIRQ(TIM16_FDCAN_IT0_IRQn);
  }
#endif
}

#else /* defined(STM32G0xx) */

void STM32_CAN::enableMBInterrupts()
{
  if (_can.handle.Instance == FDCAN1)
  {
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
#ifdef FDCAN2
  else if (_can.handle.Instance == FDCAN2)
  {
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
#endif
#ifdef FDCAN3
  else if (_can.handle.Instance == FDCAN3)
  {
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
  }
#endif
}

void STM32_CAN::disableMBInterrupts()
{
  if (_can.handle.Instance == FDCAN1)
  {
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
#ifdef FDCAN2
  else if (_can.handle.Instance == FDCAN2)
  {
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
#endif
#ifdef FDCAN3
  else if (_can.handle.Instance == FDCAN3)
  {
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
  }
#endif
}
#endif /* else defined(STM32G0xx) */
#endif /* defined(HAL_FDCAN_MODULE_ENABLED) */


void STM32_CAN::enableFIFO(bool status)
{
  //Nothing to do here. The FIFO is on by default. This is just to work with code made for Teensy FlexCan.
  (void) status;
}

/* Interrupt functions
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
#if defined(HAL_CAN_MODULE_ENABLED)
// There is 3 TX mailboxes. Each one has own transmit complete callback function, that we use to pull next message from TX ringbuffer to be sent out in TX mailbox.
extern "C" void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  stm32_can_t * canObj = get_can_obj(CanHandle);
  STM32_CAN * _can = (STM32_CAN *)canObj->__this;
  CAN_message_t txmsg;

  if (_can->removeFromRingBuffer(_can->txRing, txmsg))
  {
    _can->write(txmsg, true);
  }
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  stm32_can_t * canObj = get_can_obj(CanHandle);
  STM32_CAN * _can = (STM32_CAN *)canObj->__this;
  CAN_message_t txmsg;

  if (_can->removeFromRingBuffer(_can->txRing, txmsg))
  {
    _can->write(txmsg, true);
  }
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  stm32_can_t * canObj = get_can_obj(CanHandle);
  STM32_CAN * _can = (STM32_CAN *)canObj->__this;
  CAN_message_t txmsg;

  if (_can->removeFromRingBuffer(_can->txRing, txmsg))
  {
    _can->write(txmsg, true);
  }
}

// This is called by RX0_IRQHandler when there is message at RX FIFO0 buffer
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
#else
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
#endif
{
  stm32_can_t * canObj = get_can_obj(CanHandle);
  STM32_CAN * _can = (STM32_CAN *)canObj->__this;
  CAN_message_t rxmsg;
  CAN_RxHeaderTypeDef   RxHeader;
  //bool state = Disable_Interrupts();

  // move the message from RX FIFO0 to RX ringbuffer
  #if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  const uint32_t fifo = CAN_RX_FIFO1;
  #else
  const uint32_t fifo = CAN_RX_FIFO0;
  #endif
  do
  {
    if (HAL_CAN_GetRxMessage( CanHandle, fifo, &RxHeader, rxmsg.buf ) == HAL_OK)
    {
      if ( RxHeader.IDE == CAN_ID_STD )
      {
        rxmsg.id = RxHeader.StdId;
        rxmsg.flags.extended = 0;
      }
      else
      {
        rxmsg.id = RxHeader.ExtId;
        rxmsg.flags.extended = 1;
      }

      rxmsg.flags.remote = RxHeader.RTR;
      rxmsg.mb           = RxHeader.FilterMatchIndex;
      rxmsg.timestamp    = RxHeader.Timestamp;
      rxmsg.len          = RxHeader.DLC;

      rxmsg.bus = canObj->bus;
      _can->addToRingBuffer(_can->rxRing, rxmsg);
    }
  } while(HAL_CAN_GetRxFifoFillLevel(CanHandle, fifo));
  //Enable_Interrupts(state);
}

#elif defined(HAL_FDCAN_MODULE_ENABLED)

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *FDCanHandle, uint32_t BufferIndexes)
{
  (void)BufferIndexes;

  stm32_can_t * canObj = get_can_obj(FDCanHandle);
  STM32_CAN * _can = (STM32_CAN *)canObj->__this;

  CAN_message_t txmsg;
  if (_can->removeFromRingBuffer(_can->txRing, txmsg))
  {
    _can->write(txmsg, true);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *FDCanHandle, uint32_t RxFifo0ITs)
{
  CAN_message_t rxmsg;
  FDCAN_RxHeaderTypeDef   RxHeader;
  stm32_can_t * canObj = get_can_obj(FDCanHandle);
  STM32_CAN * _can = (STM32_CAN *)canObj->__this;
  rxmsg.bus = canObj->bus;

  if(RxFifo0ITs & FDCAN_IR_RF0N) //New message
  {

    do
    {
      if (HAL_FDCAN_GetRxMessage(FDCanHandle, FDCAN_RX_FIFO0, &RxHeader, rxmsg.buf) == HAL_OK)
      {
        if ( RxHeader.IdType == FDCAN_STANDARD_ID )
        {
          rxmsg.id = RxHeader.Identifier;
          rxmsg.flags.extended = 0;
        }
        else
        {
          rxmsg.id = RxHeader.Identifier;
          rxmsg.flags.extended = 1;
        }

        rxmsg.flags.remote = RxHeader.RxFrameType == FDCAN_REMOTE_FRAME;
        rxmsg.mb           = RxHeader.FilterIndex;
        rxmsg.timestamp    = RxHeader.RxTimestamp;
        /** NOTE: DataLength contains DLC code not length in bytes */
        rxmsg.len          = DLCtoLength[RxHeader.DataLength];

        _can->addToRingBuffer(_can->rxRing, rxmsg);
      }
    } while(HAL_FDCAN_GetRxFifoFillLevel(FDCanHandle, FDCAN_RX_FIFO0));
  }
}

#endif /* defined(HAL_FDCAN_MODULE_ENABLED) */


#if defined(HAL_CAN_MODULE_ENABLED)

#ifdef CAN1_IRQHandler_AIO
extern "C" void CAN1_IRQHandler_AIO(void)
{
  if(canObj[CAN1_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
  #if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
  if(phcec)
  {
    HAL_CEC_IRQHandler(phcec);
  }
  #endif
}

#else

// RX IRQ handlers
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
/** If USB blocks TX and RX0 IRQs, will use RX1 by default*/
extern "C" void CAN1_RX1_IRQHandler(void)
#else
extern "C" void CAN1_RX0_IRQHandler(void)
#endif
{
  if(canObj[CAN1_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
}

#ifdef CAN2
extern "C" void CAN2_RX0_IRQHandler(void)
{
  if(canObj[CAN2_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
  }
}
#endif
#ifdef CAN3
extern "C" void CAN3_RX0_IRQHandler(void)
{
  if(canObj[CAN3_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
  }
}
#endif

// TX IRQ handlers
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
/** If USB blocks TX and RX0 IRQs, need to poll for Tx events*/
extern "C" void STM32_CAN_Poll_IRQ_Handler(void)
#else
extern "C" void CAN1_TX_IRQHandler(void)
#endif
{
  if(canObj[CAN1_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
}

#ifdef CAN2
extern "C" void CAN2_TX_IRQHandler(void)
{
  if(canObj[CAN2_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
  }
}
#endif
#ifdef CAN3
extern "C" void CAN3_TX_IRQHandler(void)
{
  if(canObj[CAN3_INDEX]) {
    HAL_CAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
  }
}
#endif

#endif /* CAN1_IRQHandler_AIO */

#elif defined(HAL_FDCAN_MODULE_ENABLED)

#if defined(STM32G0xx)
    /** NOTE: no irq handlers defined here. They are defined already in
     * HardwareTimer.cpp of STM32 Arduino core
     * since they are shared with 2 Timer peripherals.
     * Those already take care of calling the HAL handlers.
     */

#else /* defined(STM32G0xx) */

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
  if(canObj[CAN1_INDEX]) {
    HAL_FDCAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
}
extern "C" void FDCAN1_IT1_IRQHandler(void)
{
  if(canObj[CAN1_INDEX]) {
    HAL_FDCAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
}
#ifdef FDCAN2
extern "C" void FDCAN2_IT0_IRQHandler(void)
{
  if(canObj[CAN2_INDEX]) {
    HAL_FDCAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
  }
}
extern "C" void FDCAN2_IT1_IRQHandler(void)
{
  if(canObj[CAN2_INDEX]) {
    HAL_FDCAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
  }
}
#endif
#ifdef FDCAN3
extern "C" void FDCAN3_IT0_IRQHandler(void)
{
  if(canObj[CAN3_INDEX]) {
    HAL_FDCAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
  }
}
extern "C" void FDCAN3_IT1_IRQHandler(void)
{
  if(canObj[CAN3_INDEX]) {
    HAL_FDCAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
  }
}
#endif

#endif /* else defined(STM32G0xx) */

#endif /* defined(HAL_FDCAN_MODULE_ENABLED) */
