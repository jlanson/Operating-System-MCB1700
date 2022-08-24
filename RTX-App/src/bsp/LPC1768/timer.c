/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTX LAB  
 *
 *  Copyright 2020-2022 NXP Semiconductors, Thomas Reidemeister and Yiqing Huang
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
 * @file        timer.c
 * @brief       timer.c - Timer example code. Tiemr IRQ is invoked every 500 us 
 *              
 * @version     V1.2021.07
 * @authors     NXP Semiconductors, Thomas Reidemeister and Yiqing Huang
 * @date        2021 JUL
 *****************************************************************************/


#include "timer.h"
#include "k_task.h"
#include "queue.h"

#define BIT(X) ( 1UL << (X) )


volatile uint32_t g_timer_count = 0; // increment every 500 us
time_usec prev_usec = 0;

/**
 * @brief: initialize timer IRQ. Only timer 0 is supported
 */
uint32_t timer_irq_init(uint8_t n_timer) 
{
    LPC_TIM_TypeDef *pTimer;
    if (n_timer == TIMER0) {
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
        //LPC_SC->PCONP |= BIT(1);
    
        -----------------------------------------------------
        Step2: Select the clock source, 
               default PCLK=CCLK/4 , where CCLK = 100MHZ.
               See tables 40 & 42 on pg56-57 in LPC17xx_UM.
        -----------------------------------------------------
        Check the PLL0 configuration to see how XTAL=12.0MHZ 
        gets to CCLK=100MHZ in system_LPC17xx.c file.
        PCLK = CCLK/4, default setting in system_LPC17xx.c.
        Enclose the code for your reference
        //LPC_SC->PCLKSEL0 &= ~(BIT(3)|BIT(2));    

        -----------------------------------------------------
        Step 3: Pin Ctrl Block configuration. 
                Optional, not used in this example
                See Table 82 on pg110 in LPC17xx_UM 
        -----------------------------------------------------
        */
        pTimer = (LPC_TIM_TypeDef *) LPC_TIM0;

    } else { /* other timer not supported yet */
        return 1;
    }

    /*
    -----------------------------------------------------
    Step 4: Interrupts configuration
    -----------------------------------------------------
    */

    /* Step 4.1: Prescale Register PR setting 
       CCLK = 100 MHZ, PCLK = CCLK/4 = 25 MHZ
       2*(6249 + 1)*(1/25) * 10^(-6) s = 10^(-3) s = 500 us = 0.5 ms
       TC (Timer Counter) toggles b/w 0 and 1 every 6250 PCLKs
       see MR setting below 
    */
    pTimer->PR = 6249;  

    /* Step 4.2: MR setting, see section 21.6.7 on pg496 of LPC17xx_UM. */
    pTimer->MR0 = 1;

    /* Step 4.3: MCR setting, see table 429 on pg496 of LPC17xx_UM.
       Interrupt on MR0: when MR0 mathches the value in the TC, 
                         generate an interrupt.
       Reset on MR0: Reset TC if MR0 mathches it.
    */
    pTimer->MCR = BIT(0) | BIT(1);

    g_timer_count = 0;

    /* Step 4.4: set up TIMER0 IRQ priority */    
    NVIC_SetPriority(TIMER0_IRQn, 0x10);
    
    /* Step 4.5: CSMSIS enable timer0 IRQ */
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Step 4.6: Enable the TCR. See table 427 on pg494 of LPC17xx_UM. */
    pTimer->TCR = 1;

    return 0;
}

time_usec tick_to_usec(TM_TICK* tk) {
    return (time_usec)(tk->tc * 1000000 + tk->pc / 100);
}

/**
 * @brief: use CMSIS ISR for TIMER0 IRQ Handler
 */
 
void TIMER0_IRQHandler(void)
{
    /* ack inttrupt, see section  21.6.1 on pg 493 of LPC17XX_UM */
    LPC_TIM0->IR = BIT(0);  

     // get current time
    TM_TICK curr_tick;
    get_tick(&curr_tick, TIMER1);
    time_usec curr_usec = tick_to_usec(&curr_tick);

    int inc = 1;

    if (curr_usec - prev_usec >= RTX_TICK_SIZE * 2) { // account for any missed ticks
        inc = (curr_usec - prev_usec) / MIN_PERIOD;
    }
    prev_usec = curr_usec;
    g_timer_count += inc;

    // execute real time tasks
    for (int tid = 0; tid < MAX_TASKS; tid++) {
        if (g_tcbs[tid].prio == PRIO_RT) { // check state of rt tasks
            switch (g_tcbs[tid].state)
            {
            case DORMANT:
            case UNINITIALIZED:
            case RUNNING:
                continue;
            default: // READY, RUNNING, SUSPENDED
                if (rt_tasks[tid].deadline <= g_timer_count) { // deadline missed
                    if (g_tcbs[tid].state == SUSPENDED) {
                        g_tcbs[tid].state = READY;
                    } else if (g_tcbs[tid].state == READY) { // there is a rt task ready to run
                        pq_pop_rt(ready_pq, &g_tcbs[tid]);
                    }

                    while (rt_tasks[tid].deadline < g_timer_count) { // this shouldn't happen; adding just in case
                        rt_tasks[tid].deadline += rt_tasks[tid].period;
                    }

                    pq_push_rt(ready_pq, &g_tcbs[tid]);
                }
                break;
            }
        }
    }

    // check for current running task
    if (gp_current_task->prio != PRIO_RT && peek_rt(ready_pq)) {
        pq_push_front(ready_pq, gp_current_task);
    } else { // current running task is rt; check for preemption
        if (rt_tasks[gp_current_task->tid].deadline < g_timer_count) {
            // deadline already missed; keep running
            return;
        }
        if (peek_rt(ready_pq) 
            && rt_tasks[gp_current_task->tid].deadline > rt_tasks[peek_rt(ready_pq)->tid].deadline) {
            // preempted because current task has a later deadline
            pq_push_rt(ready_pq, gp_current_task);
            //gp_current_task->state = READY;
        } else {
            return;
        }
    }

    // make new scheduling decision
    // gp_current_task->state = READY;
    k_tsk_run_new();
    
    return;

    // get current time
    // TM_TICK *curr_tick;
    // get_tick(curr_tick, TIMER1);
    // time_usec curr_usec = tick_to_usec(curr_tick);

    // int inc = 1;

    // if (curr_usec - prev_usec >= RTX_TICK_SIZE * 2) { // account for any missed ticks
    //     // Why divide by MIN_PERIOD?
    //     // Divide TICK_SIZE to know how many number of ticks should be added to g_timer_count.
    //     inc = (curr_usec - prev_usec) / RTX_TICK_SIZE;
    // }
    // prev_usec = curr_usec;
    // g_timer_count += inc;

    // execute real time tasks
//     for (int tid = 0; tid < MAX_TASKS; tid++) {
//         if (g_tcbs[tid].prio == PRIO_RT) { // check state of rt tasks
//             switch (g_tcbs[tid].state)
//             {
//             case DORMANT: break;
//             case UNINITIALIZED: break;
//             case RUNNING: break;
//                 // continue;
//             default: // READY and SUSPENDED

//             // 1. If 

//             // 1. If it happens to be a suspended RT task's start of next period, release it.
//             // Also, update this ready RT's deadline.
//                 if (rt_tasks[tid].deadline == g_timer_count) {
//                     if (g_tcbs[tid].state == SUSPENDED) {
//                         g_tcbs[tid].state = READY;
//                         pq_push_rt(ready_pq, &g_tcbs[tid]);
//                         rt_tasks[tid].deadline += rt_tasks[tid].period;
//                     }
//                 }

//                 if (rt_tasks[tid].deadline < g_timer_count) { // deadline missed
//                     if (g_tcbs[tid].state == SUSPENDED) {
//                         g_tcbs[tid].state = READY;
//                         pq_push_rt(ready_pq, &g_tcbs[tid]);
//                     } else if (g_tcbs[tid].state == READY) { // there is a rt task ready to run
//                     // What's the point of this pop?
//                         // pq_pop_rt(ready_pq, &g_tcbs[gp_current_task->tid]);
//                     }

//                 // Update deadline. Why shouldn't happen?
//                     while (rt_tasks[tid].deadline < g_timer_count) { // this shouldn't happen; adding just in case
//                         rt_tasks[tid].deadline += rt_tasks[tid].period;
//                     }

//                     // pq_push_rt(ready_pq, &g_tcbs[tid]);
//                 }

//                 if (rt_tasks[tid].deadline > g_timer_count) {
//                     // do nothing.
//                 }
                
//                 break;
//             }
//         }
//     }

//     // check for current running task
//     if (gp_current_task->prio != PRIO_RT) {
// // Why?
//         pq_push_front(ready_pq, gp_current_task);
//     } else { // current running task is rt; check for preemption

//     // deadline already missed; keep running and don't schedule
//         if (rt_tasks[gp_current_task->tid].deadline < g_timer_count) {
//             // deadline already missed; keep running
//             return;
//         }
//     // if still before deadline, check if other RT should preempt the running RT.
//         // 1. preempted because current task has a later deadline
//         if (peek_rt(ready_pq) 
//             && rt_tasks[gp_current_task->tid].deadline > rt_tasks[peek_rt(ready_pq)->tid].deadline) {
//             gp_current_task->state = READY;
//             pq_push_rt(ready_pq, gp_current_task);
//         } 
//         // 2. If no other RT in RT ready queue, the running RT must keep running.
//         // Or if running RT still has shortest deadline, keeps running.
//         else {
//             return;
//         }
//     }

    // make new scheduling decision
    // gp_current_task->state = READY;
    // k_tsk_run_new();
}


/**************************************************************************//**
 * @brief       Setting up Timer1&2 as free-running counter. No interrupt is fired.
 *              The timer peripheral clock speed is set to the cpu clock speed.
 *              The prescale counter increments every 10 nano seconds.
 * @param       uint8_t n_timer: 1 for timer1, 2 for timer2, other timers are not supported.
 * @return      0 on success and non-zero on failure
 *              The timer couner increments every 1 second.
 * @details     The timer has two counters. One is PC, the prescale counter;
 *              the other is TC, the timer counter.
 *              The PC incrementes every PCLCK (peripheral clock) cycle.
 *              When PC reaches the pre-set value in PR (prescale register),
 *              the TC increments by one and the PC gets re-set to zero.
 *              When TC reaches the pre-set value in MCR0, the match register, 
 *              it resets to zero.
 *              The Timer2 PCLCK is set to CCLCK (100 MHZ cpu clock).
 *              The PR is set to (100000000 - 1) so that every 1 second 
 *              the TC is incremetned by one.
 *****************************************************************************/

uint32_t timer_freerun_init(uint8_t n_timer) 
{
    LPC_TIM_TypeDef *pTimer;
    
    switch (n_timer) {
        case TIMER1:
            pTimer = (LPC_TIM_TypeDef *) LPC_TIM1;
            /*------------------------------------------------------------------------*
            * Power setting of TIMER1 
            * See Table 46 on page 63 of LPC17xx user's manual
            *------------------------------------------------------------------------*/
            
            LPC_SC->PCONP |= BIT(2);
            
           /*------------------------------------------------------------------------*
            * PCLK = CCLK, can also be set in system_LPC17xx.c by using configuratio wizard 
            * We set explicitly here to overwrite the system_LPC17xx.c configuration so that
            * we do not need to modify student's system_LPC17xx.c file
            * See Tables 40 and 42 on page 56 of LPC17xx user's manual
            *------------------------------------------------------------------------*/
            
            LPC_SC->PCLKSEL0 |= BIT(4);
            LPC_SC->PCLKSEL0 &= ~BIT(5);
            break;
        case TIMER2:
            pTimer = (LPC_TIM_TypeDef *) LPC_TIM2;
            /*------------------------------------------------------------------------*
            * Power setting of TIMER2 
            * See Table 46 on page 63 of LPC17xx user's manual
            *------------------------------------------------------------------------*/
            
            LPC_SC->PCONP |= BIT(22);
            
           /*------------------------------------------------------------------------*
            * PCLK = CCLK, can also be set in system_LPC17xx.c by using configuratio wizard 
            * We set explicitly here to overwrite the system_LPC17xx.c configuration so that
            * we do not need to modify student's system_LPC17xx.c file
            * See Tables 41 and 42 on page 56 of LPC17xx user's manual
            *------------------------------------------------------------------------*/
            
            LPC_SC->PCLKSEL1 |= BIT(12);
            LPC_SC->PCLKSEL1 &= ~BIT(13);
            break;
        default:
            return 1;
    }
    
   /*------------------------------------------------------------------------*
    * Step 1. Prescale Register PR setting 
    * CCLK = 100 MHZ, PCLK = CCLK
    * Prescale counter increments every PCLK tick: (1/100)* 10^(-6) s = 10 ns
    * TC increments every (PR + 1) PCLK cycles
    * TC increments every (MR0 + 1) * (PR + 1) PCLK cycles 
    *------------------------------------------------------------------------*/
     
    pTimer->PR = 100000000 - 1; /* increment timer counter every 1*10^8 PCLK ticks, which is 1 sec */ 

   /*------------------------------------------------------------------------* 
    * Step 2: MR setting, see section 21.6.7 on pg496 of LPC17xx_UM.
    * Effectively, using timer2 as a counter to measure time, 
    * there is no overflow in TC in about 100 years 
    *------------------------------------------------------------------------*/
    pTimer->MR0 = 0xFFFFFFFF;  

   /*------------------------------------------------------------------------*  
    * Step 3: MCR setting, see table 429 on pg496 of LPC17xx_UM.
    * Reset on MR0: Reset TC if MR0 mathches it. No interrupt triggered on match.
    *------------------------------------------------------------------------*/
    pTimer->MCR = BIT(1);
   /*------------------------------------------------------------------------* 
    * Step 4: Enable the counter. See table 427 on pg494 of LPC17xx_UM.
    *------------------------------------------------------------------------*/
    
    pTimer->TCR = 1;

    return 0;
}

/**************************************************************************//**
 * @brief   	obtain the current PC and TC readings
 *          
 * @return      0 on success and non-zero on failure
 * @param[out]  struct ae_tick *tk, structure to return PC and TC readings
 * @param[in]   uinit8_t n_timer, the timer index, 2 for timer2
 * @attention   tk memory should be allocated by the caller.
 *****************************************************************************/

int get_tick(TM_TICK *tk, uint8_t n_timer) 
{
    LPC_TIM_TypeDef *pTimer;
    
    switch (n_timer) {
        case TIMER0:
            pTimer = (LPC_TIM_TypeDef *) LPC_TIM0;
            break;
        case TIMER1:
            pTimer = (LPC_TIM_TypeDef *) LPC_TIM1;
            break;
        case TIMER2:
            pTimer = (LPC_TIM_TypeDef *) LPC_TIM2;
            break;
        case TIMER3:
            pTimer = (LPC_TIM_TypeDef *) LPC_TIM3;
            break;
        default:
            return -1;
    }
    
    tk->tc = pTimer->TC;
    tk->pc = pTimer->PC;
    
    return 0;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
