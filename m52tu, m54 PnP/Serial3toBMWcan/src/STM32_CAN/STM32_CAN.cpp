/* -------------------------------------------------------------------------------------------
 simpel CAN driver for stm32 devices using HAL

 Before you can use this driver you need to enable the CAN module in the HAL configuration file,
 for a stm32f103 blue board you have to find the stm32f1xx_hal_conf.h 
 file and uncomment #define HAL_CAN_MODULE_ENABLED
 
 This can be used in a C++ projects by replacing the
 #include <Arduino.h> with the include for the used HAL for example;  
 #include <stm32f1xx_hal.h>
 
 Inspired by the following giving me the input needed to make this work:
 https://github.com/jiauka/stm32Can
 https://gist.github.com/Arman92/154e2540847b32c44c29
 https://github.com/collin80
 st's CAN examples and documentation
 
---------------------------------------------------------------------------------------------*/


#include "STM32_CAN.h"

static CAN_HandleTypeDef     hcan1;
static CAN_HandleTypeDef     hcan2;

// instantiate the canbus adapter(s)
stm32Can Can1( &hcan1, 1 );

#ifdef CAN2
stm32Can Can2( &hcan2, 2 );
#endif

/* Constructor */
stm32Can::stm32Can( CAN_HandleTypeDef* pCanHandle, int portNumber ) {

  if (_canIsActive) { return; }

  n_pCanHandle = pCanHandle;
  
  sizeRxBuffer=SIZE_RX_BUFFER; //default value, use setRxBufferSize to change it before begin
  sizeTxBuffer=SIZE_TX_BUFFER; //default value, use setTxBufferSize to change it before begin

  _portNumber = portNumber;
}

/* Init and start CAN */
void stm32Can::begin( bool UseAltPins ) {
    init( n_pCanHandle, UseAltPins );
}

/* Init and start CAN */
void stm32Can::init( CAN_HandleTypeDef* CanHandle, bool UseAltPins ) {

  /* exit if CAN already is active */
  if ( _canIsActive ) return;
  
  _canIsActive = true;
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  DEBUG(Serial.begin(115200));
  DEBUG(Serial.println("stm32Can:Begin"));
  if ( _portNumber == 1 ) {
  DEBUG(Serial.println("CAN1"));
  }
  
  initializeBuffers();
  
  /* Configure CAN **************************************************/
  /* Struct init*/
  if ( _portNumber == 1 )
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    if(UseAltPins) {
      /* Enable AFIO clock and remap CAN PINs to PB_8 and PB_9*/
	  #if defined(STM32F1xx)
      __HAL_RCC_AFIO_CLK_ENABLE();
      __HAL_AFIO_REMAP_CAN1_2();
      #endif
      /* CAN1 RX GPIO pin configuration */
      GPIO_InitStruct.Pin = GPIO_PIN_8;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    } 
    else {
	  #if defined(STM32F1xx)
      __HAL_RCC_AFIO_CLK_ENABLE();
	  #endif
      /* CAN1 RX GPIO pin configuration */
      GPIO_InitStruct.Pin = GPIO_PIN_11;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_12;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    /*##-3- Configure the NVIC #################################################*/
    /* NVIC configuration for CAN1 Reception complete interrupt */
    HAL_NVIC_SetPriority( CAN1_RX0_IRQn, 4, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX0_IRQn );
	/* NVIC configuration for CAN1 Transmission complete interrupt */
    HAL_NVIC_SetPriority( CAN1_TX_IRQn, 3, 0 );
    HAL_NVIC_EnableIRQ( CAN1_TX_IRQn );
    
    CanHandle->Instance = CAN1;
  }
#ifdef CAN2  
  else
  {
     /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
	#if defined(STM32F1xx)
    __HAL_RCC_CAN2_CLK_ENABLE();

    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
    #endif
    if(UseAltPins) {
      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**CAN2 GPIO Configuration    
      PB5     ------> CAN2_RX
      PB6     ------> CAN2_TX 
      */
      GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    } 
    else {
      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**CAN2 GPIO Configuration    
      PB12     ------> CAN2_RX
      PB13     ------> CAN2_TX 
      */
      GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    /*##-3- Configure the NVIC #################################################*/
    /* NVIC configuration for CAN2 Reception complete interrupt */
    HAL_NVIC_SetPriority( CAN2_RX0_IRQn, 4, 0 );
    HAL_NVIC_EnableIRQ( CAN2_RX0_IRQn );
	/* NVIC configuration for CAN2 Transmission complete interrupt */
    HAL_NVIC_SetPriority( CAN2_TX_IRQn, 4, 0 );
    HAL_NVIC_EnableIRQ( CAN2_TX_IRQn );
    
    CanHandle->Instance = CAN2;
  }
#endif

  CanHandle->Init.TimeTriggeredMode = DISABLE;
  CanHandle->Init.AutoBusOff = DISABLE;
  CanHandle->Init.AutoWakeUp = DISABLE;
  CanHandle->Init.AutoRetransmission  = DISABLE;
  CanHandle->Init.ReceiveFifoLocked  = DISABLE;
  CanHandle->Init.TransmitFifoPriority = ENABLE;
  CanHandle->Init.Mode = CAN_MODE_NORMAL;
}

void stm32Can::setBaudRate( uint32_t baudrate ) {

  CAN_FilterTypeDef sFilterConfig;
  /* Calculate and set baudrate */
  calculateBaudrate( n_pCanHandle, baudrate );

  /*Initializes CAN */
  HAL_CAN_Init( n_pCanHandle );

  /* CAN filter init */
   if ( _portNumber == 1 ) {
	sFilterConfig.FilterBank = 0; // Can1 0 to 13
   } else {
	sFilterConfig.FilterBank = 14; // Can2 14 to 27
   }
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14; // Define that filter bank from 14 to 27 are for Can2, this is not relevant for devices with only one CAN
  
  if (HAL_CAN_ConfigFilter( n_pCanHandle, &sFilterConfig ) != HAL_OK)
  {
    /* Filter configuration Error */
    //Error_Handler();
    DEBUG(Serial.println("stm32Can - error configuring filter"));
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start( n_pCanHandle ) != HAL_OK)
  {
    /* Start Error */
    //Error_Handler();
    DEBUG(Serial.println("stm32Can - error starting"));
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification( n_pCanHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    //Error_Handler();
    DEBUG(Serial.println("stm32Can - error activating int"));
  }
  /* Activate CAN TX notification */
  if (HAL_CAN_ActivateNotification( n_pCanHandle, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* Notification Error */
    //Error_Handler();
    DEBUG(Serial.println("stm32Can - error activating TX int"));
  }
  DEBUG(Serial.println("stm32Can - CAN startet")); 
}

bool stm32Can::write(CAN_message_t &msg, bool wait_sent) {
  bool ret = true;
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;
 
  __HAL_CAN_DISABLE_IT(n_pCanHandle, CAN_IT_TX_MAILBOX_EMPTY);

  if ( msg.flags.extended == 1 ) // Extended ID when msg.flags.extended is 1
  {
	  TxHeader.ExtId = msg.id;
	  TxHeader.IDE   = CAN_ID_EXT;
  }
  else // Standard ID otherwise
  {
	  TxHeader.StdId = msg.id;
	  TxHeader.IDE   = CAN_ID_STD;
  }
  
  TxHeader.RTR   = CAN_RTR_DATA;
  TxHeader.DLC   = msg.len;
  TxHeader.TransmitGlobalTime = DISABLE;

//  if ( wait_sent ) {
//      /* Start the Transmission process */
//     if ( HAL_CAN_AddTxMessage( n_pCanHandle, &TxHeader, msg.buf, &TxMailbox) != HAL_OK )
//      {
//        /* Transmission request Error */
//        ret = false;
//      }
//  }
//  else
//  {
//    uint8_t prio = (uint8_t) ((msg.id >> 26) & 0x7);
    if( HAL_CAN_AddTxMessage( n_pCanHandle, &TxHeader, msg.buf, &TxMailbox) != HAL_OK ) {
      if( Can1.addToRingBuffer(Can1.txRing, msg)==false ) {
        DEBUG(Serial.println("TX full"));
        ret= false;; // no more room
      }
    }
//  }
  __HAL_CAN_ENABLE_IT(n_pCanHandle, CAN_IT_TX_MAILBOX_EMPTY);
  return ret;
}

bool stm32Can::read(CAN_message_t &msg) {
  bool ret;
  __HAL_CAN_DISABLE_IT( n_pCanHandle, CAN_IT_RX_FIFO0_MSG_PENDING ); 
  ret = removeFromRingBuffer(rxRing, msg);
  __HAL_CAN_ENABLE_IT( n_pCanHandle, CAN_IT_RX_FIFO0_MSG_PENDING ); 
  return ret;
}

bool stm32Can::readdebug(CAN_message_t &msg) {
  bool ret;
  CAN_RxHeaderTypeDef   RxHeader;

  /* For debug usage - this can be used to read a message if interupt not is working */
  if (HAL_CAN_GetRxMessage( &hcan1, CAN_RX_FIFO0, &RxHeader, msg.buf ) == HAL_OK)
  {
    if ( RxHeader.IDE == CAN_ID_STD )
	{
		msg.id = RxHeader.StdId;
		msg.flags.extended = 0;
	}
	else
	{
		msg.id = RxHeader.ExtId;
		msg.flags.extended = 1;
	}
    msg.flags.remote = RxHeader.RTR;
    msg.priority     = RxHeader.FilterMatchIndex;
    msg.timestamp    = RxHeader.Timestamp;
    msg.len          = RxHeader.DLC;
    ret = true;
  }
  else
  {
    ret = false;
  }

  uint32_t test;

  /* print the timing parameters */
  DEBUG(Serial.println( "Debug - timing settings" ));
    
  test = (uint32_t) n_pCanHandle->Init.SyncJumpWidth;
  DEBUG(Serial.print( "SyncJumpWidth: " ));
  DEBUG(Serial.println( test ));
    
  test = (uint32_t) n_pCanHandle->Init.TimeSeg1;
  DEBUG(Serial.print( "TimeSeg1: " ));
  DEBUG(Serial.println( test ));
    
  test = (uint32_t) n_pCanHandle->Init.TimeSeg2;
  DEBUG(Serial.print( "TimeSeg2: " ));
  DEBUG(Serial.println( test ));
    
  test = (uint32_t) n_pCanHandle->Init.Prescaler;
  DEBUG(Serial.print( "Prescaler: " ));
  DEBUG(Serial.println( test ));
      
  return ret;
}

uint32_t stm32Can::available( void ) 
{ 
  return ringBufferCount(rxRing); 
};

bool stm32Can::setFilter( uint32_t FilterID, uint32_t FilterMask, uint8_t FilterBank, bool IDStdOrExt )
{	
	CAN_FilterTypeDef sFilterConfig;
	
	// Define filter
	sFilterConfig.FilterBank = FilterBank;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;

	switch ( IDStdOrExt ) {
		case IDStd: // Standard
			sFilterConfig.FilterIdHigh = (uint16_t) (FilterID << 5);
			sFilterConfig.FilterIdLow = 0;
			sFilterConfig.FilterMaskIdHigh = (uint16_t) (FilterMask << 5);
			sFilterConfig.FilterMaskIdLow = CAN_ID_EXT;
			break;
		case IDExt: // Extended
			sFilterConfig.FilterIdLow = (uint16_t) (FilterID << 3);
			sFilterConfig.FilterIdLow |= CAN_ID_EXT;
			sFilterConfig.FilterIdHigh = (uint16_t) (FilterID >> 13);
			sFilterConfig.FilterMaskIdLow = (uint16_t) (FilterMask << 3);
			sFilterConfig.FilterMaskIdLow |= CAN_ID_EXT;
			sFilterConfig.FilterMaskIdHigh = (uint16_t) (FilterMask >> 13);
			break;
	}
 
	// Enable filter
	if (HAL_CAN_ConfigFilter( n_pCanHandle, &sFilterConfig ) != HAL_OK)
	{
		/* Filter configuration Error */
		DEBUG(Serial.println("stm32Can - error configuring filter"));

		return 1;
	}
	else
	{
		return 0;
	}
}

void stm32Can::initializeBuffers() {
    if( isInitialized() )
      return;
  
    // set up the transmit and receive ring buffers
    if(tx_buffer==0)
      tx_buffer=new CAN_message_t[sizeTxBuffer];
      
    initRingBuffer(txRing, tx_buffer, sizeTxBuffer);

    if(rx_buffer==0)
      rx_buffer=new CAN_message_t[sizeRxBuffer];

    initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}

void stm32Can::initRingBuffer(RingbufferTypeDef &ring, volatile CAN_message_t *buffer, uint32_t size)
{
    ring.buffer = buffer;
    ring.size = size;
    ring.head = 0;
    ring.tail = 0;
}

bool stm32Can::addToRingBuffer (RingbufferTypeDef &ring, const CAN_message_t &msg)
{
    uint16_t nextEntry;

    nextEntry =(ring.head + 1) % ring.size;

    /* check if the ring buffer is full */

    if(nextEntry == ring.tail) {
        return(false);
    }

    /* add the element to the ring */

    memcpy((void *)&ring.buffer[ring.head],(void *)&msg, sizeof(CAN_message_t));

    /* bump the head to point to the next free entry */

    ring.head = nextEntry;

    return(true);
}

/*
 * \brief Remove a CAN message from the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to fill in.
 *
 * \retval true if a message was removed, false if the ring is empty.
 *
 */

bool stm32Can::removeFromRingBuffer(RingbufferTypeDef &ring, CAN_message_t &msg)
{

    /* check if the ring buffer has data available */

    if(isRingBufferEmpty(ring) == true) {
        return(false);
    }

    /* copy the message */

    memcpy((void *)&msg,(void *)&ring.buffer[ring.tail], sizeof(CAN_message_t));

    /* bump the tail pointer */

    ring.tail =(ring.tail + 1) % ring.size;

    return(true);
}

/*
 * \brief Check if the specified ring buffer is empty.
 *
 * \param ring - ring buffer to use.
 *
 * \retval true if the ring contains data, false if the ring is empty.
 *
 */

bool stm32Can::isRingBufferEmpty(RingbufferTypeDef &ring)
{
    if(ring.head == ring.tail) {
        return(true);
    }

    return(false);
}

/*
 * \brief Count the number of entries in the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 *
 * \retval a count of the number of elements in the ring buffer.
 *
 */

uint32_t stm32Can::ringBufferCount(RingbufferTypeDef &ring)
{
    int32_t entries;

    entries = ring.head - ring.tail;

    if(entries < 0) {
        entries += ring.size;
    }

    return((uint32_t)entries);
}

void stm32Can::calculateBaudrate( CAN_HandleTypeDef *CanHandle, int baudrate )
{
  int sjw = 1;
  int bs1 = 1;
  int bs2 = 1;
  int prescaler = 1;
  uint32_t _SyncJumpWidth;
  uint32_t _TimeSeg1;
  uint32_t _TimeSeg2;
  uint32_t _Prescaler;
  
  bool shouldBrake = false;
  
  uint32_t frequency = getAPB1Clock();
  
  for (; sjw <= 4 && !shouldBrake; )
  {
    for (; prescaler <= 1024 && !shouldBrake; )
    {
      for (; bs2 <= 8 && !shouldBrake; )
      {
        for (; bs1 <= 16 && !shouldBrake; )
        {
          int calcBaudrate = (int)(frequency / (prescaler * (sjw + bs1 + bs2)));
          
          if (calcBaudrate == baudrate)
          {
            if (sjw == 1)
              _SyncJumpWidth = CAN_SJW_1TQ;
            else if (sjw == 2)
              _SyncJumpWidth = CAN_SJW_2TQ;
            else if (sjw == 3)
              _SyncJumpWidth = CAN_SJW_3TQ;
            else if (sjw == 4)
              _SyncJumpWidth = CAN_SJW_4TQ;

            if (bs1 == 1)
              _TimeSeg1 = CAN_BS1_1TQ;
            else if (bs1 == 2)
              _TimeSeg1 = CAN_BS1_2TQ;
            else if (bs1 == 3)
              _TimeSeg1 = CAN_BS1_3TQ;
            else if (bs1 == 4)
              _TimeSeg1 = CAN_BS1_4TQ;
            else if (bs1 == 5)
              _TimeSeg1 = CAN_BS1_5TQ;
            else if (bs1 == 6)
              _TimeSeg1 = CAN_BS1_6TQ;
            else if (bs1 == 7)
              _TimeSeg1 = CAN_BS1_7TQ;
            else if (bs1 == 8)
              _TimeSeg1 = CAN_BS1_8TQ;
            else if (bs1 == 9)
              _TimeSeg1 = CAN_BS1_9TQ;
            else if (bs1 == 10)
              _TimeSeg1 = CAN_BS1_10TQ;
            else if (bs1 == 11)
              _TimeSeg1 = CAN_BS1_11TQ;
            else if (bs1 == 12)
              _TimeSeg1 = CAN_BS1_12TQ;
            else if (bs1 == 13)
              _TimeSeg1 = CAN_BS1_13TQ;
            else if (bs1 == 14)
              _TimeSeg1 = CAN_BS1_14TQ;
            else if (bs1 == 15)
              _TimeSeg1 = CAN_BS1_15TQ;
            else if (bs1 == 16)
              _TimeSeg1 = CAN_BS1_16TQ;

            if (bs2 == 1)
              _TimeSeg2 = CAN_BS2_1TQ;
            else if (bs2 == 2)
              _TimeSeg2 = CAN_BS2_2TQ;
            else if (bs2 == 3)
              _TimeSeg2 = CAN_BS2_2TQ;
            else if (bs2 == 4)
              _TimeSeg2 = CAN_BS2_2TQ;

            _Prescaler = prescaler;

            shouldBrake = true;
          }
          bs1++;
        }
        if (!shouldBrake)
        {
          bs2++;
        }
      }
      if (!shouldBrake)
      {
        bs1 = 1;
        bs2 = 1;
        prescaler++;
      }
    }
    if (!shouldBrake)
    {
      bs1 = 1;
      sjw++;
    }
  }

  CanHandle->Init.SyncJumpWidth = _SyncJumpWidth;
  CanHandle->Init.TimeSeg1 = _TimeSeg1;
  CanHandle->Init.TimeSeg2 = _TimeSeg2;
  CanHandle->Init.Prescaler = _Prescaler;
}

uint32_t stm32Can::getAPB1Clock()
{
  RCC_ClkInitTypeDef clkInit;
  uint32_t flashLatency;
  HAL_RCC_GetClockConfig(&clkInit, &flashLatency);

  uint32_t hclkClock = HAL_RCC_GetHCLKFreq();
  uint8_t clockDivider = 1;
  if (clkInit.APB1CLKDivider == RCC_HCLK_DIV1)
    clockDivider = 1;
  if (clkInit.APB1CLKDivider == RCC_HCLK_DIV2)
    clockDivider = 2;
  if (clkInit.APB1CLKDivider == RCC_HCLK_DIV4)
    clockDivider = 4;
  if (clkInit.APB1CLKDivider == RCC_HCLK_DIV8)
    clockDivider = 8;
  if (clkInit.APB1CLKDivider == RCC_HCLK_DIV16)
    clockDivider = 16;

  uint32_t apb1Clock = hclkClock / clockDivider;

  return apb1Clock;
}

void stm32Can::enableMBInterrupts()
{
    if (n_pCanHandle->Instance == CAN1) 
    {
      HAL_NVIC_EnableIRQ( CAN1_TX_IRQn );
    }
#ifdef CAN2
    else
    {
      HAL_NVIC_EnableIRQ( CAN2_TX_IRQn );
    }
#endif
}

void stm32Can::disableMBInterrupts()
{
    if (n_pCanHandle->Instance == CAN1) 
    {
      HAL_NVIC_DisableIRQ( CAN1_TX_IRQn );
    }
#ifdef CAN2
    else
    {
      HAL_NVIC_DisableIRQ( CAN2_TX_IRQn );
    }
#endif
}

/* Interupt functions */
extern "C" void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  DEBUG(Serial.println( "MB0 Empty int" ));
  uint32_t TxMailbox;
  CAN_message_t txmsg;
  CAN_TxHeaderTypeDef TxHeader;
  if ( Can1.removeFromRingBuffer(Can1.txRing, txmsg) )
  {
    if ( txmsg.flags.extended == 1 ) // Extended ID when msg.flags.extended is 1
    {
	    TxHeader.ExtId = txmsg.id;
	    TxHeader.IDE   = CAN_ID_EXT;
    }
    else // Standard ID otherwise
    {
	    TxHeader.StdId = txmsg.id;
	    TxHeader.IDE   = CAN_ID_STD;
    }
  
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = txmsg.len;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(CanHandle, &TxHeader, txmsg.buf, &TxMailbox) != HAL_OK;	
  }
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  DEBUG(Serial.println( "MB1 Empty int" ));
  uint32_t TxMailbox;
  CAN_message_t txmsg;
  CAN_TxHeaderTypeDef TxHeader;
  if ( Can1.removeFromRingBuffer(Can1.txRing, txmsg) )
  {
    if ( txmsg.flags.extended == 1 ) // Extended ID when msg.flags.extended is 1
    {
	    TxHeader.ExtId = txmsg.id;
	    TxHeader.IDE   = CAN_ID_EXT;
    }
    else // Standard ID otherwise
    {
	    TxHeader.StdId = txmsg.id;
	    TxHeader.IDE   = CAN_ID_STD;
    }
  
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = txmsg.len;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(CanHandle, &TxHeader, txmsg.buf, &TxMailbox) != HAL_OK;	
  }
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *CanHandle )
{
  DEBUG(Serial.println( "MB2 Empty int" ));
  uint32_t TxMailbox;
  CAN_message_t txmsg;
  CAN_TxHeaderTypeDef TxHeader;
  if ( Can1.removeFromRingBuffer(Can1.txRing, txmsg) )
  {
    if ( txmsg.flags.extended == 1 ) // Extended ID when msg.flags.extended is 1
    {
	    TxHeader.ExtId = txmsg.id;
	    TxHeader.IDE   = CAN_ID_EXT;
    }
    else // Standard ID otherwise
    {
	    TxHeader.StdId = txmsg.id;
	    TxHeader.IDE   = CAN_ID_STD;
    }
  
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = txmsg.len;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(CanHandle, &TxHeader, txmsg.buf, &TxMailbox) != HAL_OK;	
  }
}
/**
  * @brief  Rx Fifo 0 message pending callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *CanHandle )
{
  DEBUG(Serial.println( "RxFifo0 int" ));
  CAN_message_t rxmsg;
  CAN_RxHeaderTypeDef   RxHeader;
  
  /* Get RX message */
  if (HAL_CAN_GetRxMessage( CanHandle, CAN_RX_FIFO0, &RxHeader, rxmsg.buf ) == HAL_OK)
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
    rxmsg.priority     = RxHeader.FilterMatchIndex;
    rxmsg.timestamp    = RxHeader.Timestamp;
    rxmsg.len          = RxHeader.DLC;

    if (CanHandle->Instance == CAN1) 
    {
      Can1.addToRingBuffer(Can1.rxRing, rxmsg);
    }
#ifdef CAN2
    else
    {
      Can2.addToRingBuffer(Can2.rxRing, rxmsg);
    }
#endif
  }
}

void stm32Can::enableLoopBack( bool yes ) {
if ( yes ) { n_pCanHandle->Init.Mode = CAN_MODE_LOOPBACK; }	
else { n_pCanHandle->Init.Mode = CAN_MODE_NORMAL; }
}

void stm32Can::enableFIFO(bool status)
{
  //Nothing to do here. The FIFO is on by default.
}

	
/**
* @brief  functions handles CAN RX0 interrupt request.
* @param  None
* @retval None
*/
extern "C" void CAN1_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan1 );
}

#ifdef CAN2
extern "C" void CAN2_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan2 );
}
#endif

/**
* @brief  functions handles CAN TX interrupt request.
* @param  None
* @retval None
*/
extern "C" void CAN1_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan1 );
}

#ifdef CAN2
extern "C" void CAN2_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan2 );
}
#endif

