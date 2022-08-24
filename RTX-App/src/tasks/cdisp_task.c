/**
 * @brief The Console Display Task Template File 
 * @note  The file name and the function name can be changed
 * @see   k_tasks.h
 */

#include "rtx.h"
#include "k_inc.h"
#include "k_msg.h"

extern uint8_t g_send_char;                    // main() read/write this flag
extern uint8_t g_char_in;                          // main() read this var
extern uint8_t g_tx_irq;                       // initial TX irq is off
extern uint8_t g_switch_flag;
extern U8 g_send_buf[32];
extern U8 g_recv_buf[UART_MBX_SIZE]; // what's the size?
extern int recv_start;
int count = 0;

void task_cdisp(void)
{
    mbx_create(CON_MBX_SIZE);

    char msg_buf[CON_MBX_SIZE];
    RTX_MSG_HDR* msg;

    LPC_UART_TypeDef* pUart = (LPC_UART_TypeDef*) LPC_UART0;

    while (TRUE) {
        recv_msg(msg_buf, CON_MBX_SIZE);
        msg = (RTX_MSG_HDR*)msg_buf;
        if (msg->type == DISPLAY) {
            if (!g_tx_irq) {
                pUart->THR = msg_buf[MSG_HDR_SIZE];
                msg->sender_tid = TID_CON;
                send_msg(TID_UART, msg);
                // ***** DO NOT DELETE THIS FOR LOOP BELOW *****
                for (int i = 0; i < 10000; i++)
                {
                    count ++;
                }
                g_tx_irq = 1;
                pUart->IER |= IER_THRE;
            }
        }
    }
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */

