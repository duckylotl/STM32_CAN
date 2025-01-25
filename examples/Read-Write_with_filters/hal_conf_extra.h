#if defined(STM32G0xx) || defined(STM32G4xx) || defined(STM32H5xx) || defined(STM32U5xx) || defined(STM32L5xx) || defined(STM32H7xx)

    #if !defined(HAL_FDCAN_MODULE_ENABLED)
        #define HAL_FDCAN_MODULE_ENABLED
    #endif

#else

    #if !defined(HAL_CAN_MODULE_ENABLED)
        #define HAL_CAN_MODULE_ENABLED

        #if defined(USBCON) && defined(STM32F1xx)
            /** NOTE: On the F1 platform CAN and USB may not be used at the same time.
             * To still allow a program to be build that may use both
             * but not at the same time this workaround may be enabled.
             * Since USB driver is using the shared IRQ handlers, the CAN driver has no access to them.
             * To handle Tx events call
             * 
             * STM32_CAN_Poll_IRQ_Handler()
             * 
             * frequently
             */
            // #define STM32_CAN_USB_WORKAROUND_POLLING
        #endif

        #if defined(USBCON) && defined(STM32F3xx)
            /** NOTE: On F3 platform CAN and USB share IRQ by default.
             *  Since USB driver is using the shared IRQ handlers, the CAN driver has no access to them.
             * 
             *  Below define maps the USB IRQs to alternate IRQ vectors, 
             *  so USB and CAN IRQs are no longer shared
             */
            // #define USE_USB_INTERRUPT_REMAPPED
        #endif

    #endif
#endif