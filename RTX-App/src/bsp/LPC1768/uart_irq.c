/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTX LAB  
 *
 *        Copyright 2020-2022 Yiqing Huang and NXP Semiconductors
 *                          All rights reserved.
 *---------------------------------------------------------------------------
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice and the following disclaimer.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/**************************************************************************//**
 * @file        uart_irq.c
 * @brief       UART IRQ handler. It receives input char through RX interrupt
 *              and then writes a string containing the input char through
 *              TX interrupts. 
 *              
 * @version     V1.2022.05
 * @authors     Yiqing Huang and NXP Semiconductors
 * @date        2022 May
 *****************************************************************************/

#include "k_inc.h"
#include "k_rtx.h"



uint8_t g_send_char = 0;                    // main() read/write this flag
uint8_t g_char_in;                          // main() read this var
uint8_t g_tx_irq = 0;                       // initial TX irq is off
uint8_t g_switch_flag = FALSE;
U8 g_send_buf[32];
U8 g_recv_buf[UART_MBX_SIZE]; // what's the size?
RTX_MSG_HDR *gp_recv_buf;
int recv_start = 1;
int UART_RECV = 0;

/**************************************************************************//**
 * @brief   initializes the n_uart interrupts
 * @note    it only supports UART0. It can be easily extended to support UART1 IRQ.
 * The step number in the comments matches the item number in Section 14.1 on pg 298
 * of LPC17xx_UM
 *****************************************************************************/
int uart_irq_init(int n_uart) {

    LPC_UART_TypeDef *pUart;

    if ( n_uart ==0 ) {
        /*
        Steps 1 & 2: system control configuration.
        Under CMSIS, system_LPC17xx.c does these two steps
         
        -----------------------------------------------------
        Step 1: Power control configuration. 
                See table 46 pg63 in LPC17xx_UM
        -----------------------------------------------------
        Enable UART0 power, this is the default setting
        done in system_LPC17xx.c under CMSIS.
        Enclose the code for your refrence
        //LPC_SC->PCONP |= BIT(3);
    
        -----------------------------------------------------
        Step2: Select the clock source. 
               Default PCLK=CCLK/4 , where CCLK = 100MHZ.
               See tables 40 & 42 on pg56-57 in LPC17xx_UM.
        -----------------------------------------------------
        Check the PLL0 configuration to see how XTAL=12.0MHZ 
        gets to CCLK=100MHZin system_LPC17xx.c file.
        PCLK = CCLK/4, default setting after reset.
        Enclose the code for your reference
        //LPC_SC->PCLKSEL0 &= ~(BIT(7)|BIT(6));    
            
        -----------------------------------------------------
        Step 5: Pin Ctrl Block configuration for TXD and RXD
                See Table 79 on pg108 in LPC17xx_UM.
        -----------------------------------------------------
        Note this is done before Steps3-4 for coding purpose.
        */
        
        /* Pin P0.2 used as TXD0 (Com0) */
        LPC_PINCON->PINSEL0 |= (1 << 4);  
        
        /* Pin P0.3 used as RXD0 (Com0) */
        LPC_PINCON->PINSEL0 |= (1 << 6);  

        pUart = (LPC_UART_TypeDef *) LPC_UART0;     
        
    } else if ( n_uart == 1) {
        
        /* see Table 79 on pg108 in LPC17xx_UM */ 
        /* Pin P2.0 used as TXD1 (Com1) */
        LPC_PINCON->PINSEL4 |= (2 << 0);

        /* Pin P2.1 used as RXD1 (Com1) */
        LPC_PINCON->PINSEL4 |= (2 << 2);          

        pUart = (LPC_UART_TypeDef *) LPC_UART1;
        
    } else {
        return 1; /* not supported yet */
    } 
    
    /*
    -----------------------------------------------------
    Step 3: Transmission Configuration.
            See section 14.4.12.1 pg313-315 in LPC17xx_UM 
            for baud rate calculation.
    -----------------------------------------------------
    */
    
    /* Step 3a: DLAB=1, 8N1 */
    pUart->LCR = UART_8N1; /* see uart.h file */ 

    /* Step 3b: 115200 baud rate @ 25.0 MHZ PCLK */
    pUart->DLM = 0; /* see table 274, pg302 in LPC17xx_UM */
    pUart->DLL = 9;    /* see table 273, pg302 in LPC17xx_UM */
    
    /* FR = 1.507 ~ 1/2, DivAddVal = 1, MulVal = 2
       FR = 1.507 = 25MHZ/(16*9*115200)
       see table 285 on pg312 in LPC_17xxUM
    */
    pUart->FDR = 0x21;       
    
 

    /*
    ----------------------------------------------------- 
    Step 4: FIFO setup.
           see table 278 on pg305 in LPC17xx_UM
    -----------------------------------------------------
        enable Rx and Tx FIFOs, clear Rx and Tx FIFOs
    Trigger level 0 (1 char per interrupt)
    */
    
    pUart->FCR = 0x07;

    /* Step 5 was done between step 2 and step 4 a few lines above */

    /*
    ----------------------------------------------------- 
    Step 6 Interrupt setting and enabling
    -----------------------------------------------------
    */
    /* Step 6a: 
       Enable interrupt bit(s) wihtin the specific peripheral register.
           Interrupt Sources Setting: RBR, THRE or RX Line Stats
       See Table 50 on pg73 in LPC17xx_UM for all possible UART0 interrupt sources
       See Table 275 on pg 302 in LPC17xx_UM for IER setting 
    */
    /* disable the Divisior Latch Access Bit DLAB=0 */
    pUart->LCR &= ~(BIT(7)); 
    
    /* enable RBR and RLS interrupts */
    pUart->IER = IER_RBR | IER_RLS; 
    
    /* Step 6b: set up UART0/1 IRQ priority */    
    NVIC_SetPriority(UART0_IRQn, 0x10);
    NVIC_SetPriority(UART1_IRQn, 0x10);
    
    /* Step 6c: enable the UART interrupt from the system level */
    
    if ( n_uart == 0 ) {
        NVIC_EnableIRQ(UART0_IRQn); /* CMSIS function */
    } else if ( n_uart == 1 ) {
        NVIC_EnableIRQ(UART1_IRQn); /* CMSIS function */
    } else {
        return 1; /* not supported yet */
    }
    /* Step 7: For Lab 3, uart0 irq has a mailbox. Initialize it here. */
    // I think I'm wrong, not need to do so as the special mailbox for UART 
    // seems to be created in k_tsk_init, calling mailbox_init_all?
    return 0;
}

/**
 * @brief: CMSIS ISR for UART0 IRQ Handler
 */

void UART0_IRQHandler(void)
{
    uint8_t IIR_IntId;        /* Interrupt ID from IIR */          
    LPC_UART_TypeDef *pUart = (LPC_UART_TypeDef *)LPC_UART0;
    
#ifdef DEBUG_1
    uart1_put_string("Entering c_UART0_IRQHandler\r\n");
#endif // DEBUG_1

    /* Reading IIR automatically acknowledges the interrupt */
    IIR_IntId = (pUart->IIR) >> 1 ; /* skip pending bit in IIR */ 
    if (IIR_IntId & IIR_RDA) { /* Receive Data Available */
        
        /* Read UART. Reading RBR will clear the interrupt */
        int char_in = pUart->RBR;
#ifdef DEBUG_0
        printf("Reading a char = %c \r\n", char_in);
#endif /* DEBUG_0 */ 
        if (g_send_char == 0 && !g_tx_irq) {
            g_char_in = char_in;
#ifdef DEBUG_0
            printf("char %c gets processed\r\n", g_char_in);
#endif /* DEBUG_0 */ 
            
            U8* buf = &g_send_buf[0];
            RTX_MSG_HDR* ptr = (RTX_MSG_HDR*) buf;
            ptr->length = MSG_HDR_SIZE + 1;
            ptr->sender_tid = TID_UART;
            ptr->type = KEY_IN;
            buf += MSG_HDR_SIZE;
            *buf = g_char_in;
            k_send_msg_nb(TID_KCD, (const void*)ptr);
            // ***** DO NOT DELETE THE PRINTF BELOW *****
            printf("");          
        }
#ifdef ECE350_P3       
        /* setting the g_continue_flag */
        if ( g_char_in == 's' ) {
            g_switch_flag = 1; 
        } else {
            g_switch_flag = 0;
        }
#endif
    } else if (IIR_IntId & IIR_THRE) {
        uint8_t char_out;
        int index = 0;

        UART_RECV = 1;
        k_recv_msg_nb(g_recv_buf, UART_MBX_SIZE);
        gp_recv_buf = (RTX_MSG_HDR*) g_recv_buf;
        UART_RECV = 0;

        int str_len = gp_recv_buf->length - MSG_HDR_SIZE;
        
        // This covers 2 cases:
        // 1. A single char is always printed by cdisp, so just check for special case, 'carriage enter'.
        // In this case, we write '\n' to the console for better formating.
        if (str_len == 1 && g_recv_buf[MSG_HDR_SIZE] == '\r') {
            char_out = '\n';
            while(!(pUart->LSR & LSR_THRE)) { } // referred from uart_polling.c
            pUart->THR = char_out;
        }
        // 2. A string, so print the rest of the chars.
        // notice it's str_len - 2, because we don't want to print out the \0 at the end of the string.
        for (int i = 0; i < str_len - 2; i++)
        {
            char_out = g_recv_buf[MSG_HDR_SIZE + i + 1];
            while(!(pUart->LSR & LSR_THRE)) { } // referred from uart_polling.c
            pUart->THR = char_out;
        }

        // Finally, reset parameters.
        pUart->IER &= ~IER_THRE; // There are no data to transfer because we've finished sending a message to the terminal. 
        g_tx_irq = 0; // set it back to 0, so that the handler could receive new keyboard inputs.
        gp_recv_buf = NULL; // should we do this?
        for (int i = 0; i < UART_MBX_SIZE; i++) // reset the buffer.
        {
            g_recv_buf[i] = 0;
        }         
    } else {  /* not implemented yet */
        #ifdef DEBUG_0
            uart1_put_string("Should not get here!\r\n");
        #endif /* DEBUG_0 */
        return;
    }    
    
    // when interrupt handling is done, if new scheduling decision is made
    // then do context switching to the newly selected task
#ifdef ECE350_P3
    if ( g_switch_flag == 1 ) {
        k_tsk_yield();
    }

    // Always call scheduler at the end of the IRQ
    

#endif // ECE350_P3
}
/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
