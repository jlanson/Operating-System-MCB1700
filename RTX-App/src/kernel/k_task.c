/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTOS LAB
 *
 *                     Copyright 2020-2022 Yiqing Huang
 *                          All rights reserved.
 *
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
 ****************************************************************************
 */

/**************************************************************************//**
 * @file        k_task.c
 * @brief       task management C file
 * @version     V1.2021.05
 * @authors     Yiqing Huang
 * @date        2021 MAY
 *
 * @attention   assumes NO HARDWARE INTERRUPTS
 * @details     The starter code shows one way of implementing context switching.
 *              The code only has minimal sanity check.
 *              There is no stack overflow check.
 *              The implementation assumes only three simple tasks and
 *              NO HARDWARE INTERRUPTS.
 *              The purpose is to show how context switch could be done
 *              under stated assumptions.
 *              These assumptions are not true in the required RTX Project!!!
 *              Understand the assumptions and the limitations of the code before
 *              using the code piece in your own project!!!
 *
 *****************************************************************************/


#include "k_inc.h"
#include "k_task.h"
#include "k_rtx.h"

PQ g_ready_PQ = {0};
PQ* ready_pq = &g_ready_PQ;
unsigned int yielding = 0;
unsigned int setting = 0;
unsigned int exiting = 0;
unsigned int creating = 0;

TASK_RT rt_tasks[MAX_TASKS];

/*
 *==========================================================================
 *                            GLOBAL VARIABLES
 *==========================================================================
 */

TCB             *gp_current_task = NULL;    // the current RUNNING task
TCB             g_tcbs[MAX_TASKS];          // an array of TCBs
//TASK_INIT       g_null_task_info;           // The null task info
U32             g_num_active_tasks = 0;     // number of non-dormant tasks

/*---------------------------------------------------------------------------
The memory map of the OS image may look like the following:
                   RAM1_END-->+---------------------------+ High Address
                              |                           |
                              |                           |
                              |       MPID_IRAM1          |
                              |   (for user space heap  ) |
                              |                           |
                 RAM1_START-->|---------------------------|
                              |                           |
                              |  unmanaged free space     |
                              |                           |
&Image$$RW_IRAM1$$ZI$$Limit-->|---------------------------|-----+-----
                              |         ......            |     ^
                              |---------------------------|     |
                              |      PROC_STACK_SIZE      |  OS Image
              g_p_stacks[2]-->|---------------------------|     |
                              |      PROC_STACK_SIZE      |     |
              g_p_stacks[1]-->|---------------------------|     |
                              |      PROC_STACK_SIZE      |     |
              g_p_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |                           |  OS Image
                              |---------------------------|     |
                              |      KERN_STACK_SIZE      |     |                
    g_k_stacks[MAX_TASKS-1]-->|---------------------------|     |
                              |                           |     |
                              |     other kernel stacks   |     |                              
                              |---------------------------|     |
                              |      KERN_STACK_SIZE      |  OS Image
              g_k_stacks[2]-->|---------------------------|     |
                              |      KERN_STACK_SIZE      |     |                      
              g_k_stacks[1]-->|---------------------------|     |
                              |      KERN_STACK_SIZE      |     |
              g_k_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |---------------------------|     |
                              |        TCBs               |  OS Image
                      g_tcbs->|---------------------------|     |
                              |        global vars        |     |
                              |---------------------------|     |
                              |                           |     |          
                              |                           |     |
                              |        Code + RO          |     |
                              |                           |     V
                 IRAM1_BASE-->+---------------------------+ Low Address
    
---------------------------------------------------------------------------*/ 

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */


/**************************************************************************//**
 * @brief   scheduler, pick the TCB of the next to run task
 *
 * @return  TCB pointer of the next to run task
 * @post    gp_curret_task is updated
 * @note    you need to change this one to be a priority scheduler
 *
 *****************************************************************************/

TCB *scheduler(void)
{
    if (peek(ready_pq)->prio == PRIO_NULL) {
        return &g_tcbs[TID_NULL];
    }

// // If Suspended RT calls scheduler, must pop from ready queue to run, cuz suspended RT is NOT running anymore.
//     if (gp_current_task->state == SUSPENDED ||gp_current_task->state == BLK_RECV || gp_current_task->state == BLK_SEND) {
//         return pop(ready_pq);
//     }

// // EDF Scheduler.
//     if(gp_current_task->prio == peek(ready_pq)->prio && peek(ready_pq)->prio == 0 && 
//     rt_tasks[peek(ready_pq)->tid].deadline < rt_tasks[gp_current_task->tid].deadline) {
//         return pop(ready_pq);
//     }

//     if (gp_current_task->prio <= peek(ready_pq)->prio && yielding == 0 && setting == 0 && exiting == 0 && creating == 0){
//         return gp_current_task;
//     }

//     if (yielding == 1) {
//         yielding = 0;
//     }

//     if (exiting == 1){
//         exiting = 0;
//     }

    return pop(ready_pq);
}

/**
 * @brief initialzie the first task in the system
 */
void k_tsk_init_first(TASK_INIT *p_task)
{
    p_task->prio         = PRIO_NULL;
    p_task->priv         = 0;
    p_task->tid          = TID_NULL;
    p_task->ptask        = &task_null;
    p_task->u_stack_size = PROC_STACK_SIZE;
}

/**
 * @brief initialzie the kcd task
 */
void k_tsk_init_kcd(TASK_INIT *p_task)
{
    p_task->prio         = HIGH;
    p_task->priv         = 0;
    p_task->tid          = TID_KCD;
    p_task->ptask        = &task_kcd;
    p_task->u_stack_size = PROC_STACK_SIZE;
}

/**
 * @brief initialzie the console display task
 */
void k_tsk_init_cdisp(TASK_INIT *p_task)
{
    p_task->prio         = HIGH;
    p_task->priv         = 1;
    p_task->tid          = TID_CON;
    p_task->ptask        = &task_cdisp;
    p_task->u_stack_size = PROC_STACK_SIZE;
}

/**
 * @brief initialzie the wall clock display task
 */
void k_tsk_init_wclock(TASK_INIT *p_task)
{
    p_task->prio         = HIGH;
    p_task->priv         = 0; //unpriviledged !!!
    p_task->tid          = TID_WCLCK;
    p_task->ptask        = &task_wall_clock;
    p_task->u_stack_size = PROC_STACK_SIZE;
}

/**************************************************************************//**
 * @brief       initialize all boot-time tasks in the system,
 *
 *
 * @return      RTX_OK on success; RTX_ERR on failure
 * @param       task_info   boot-time task information structure pointer
 * @param       num_tasks   boot-time number of tasks
 * @pre         memory has been properly initialized
 * @post        none
 * @see         k_tsk_create_first
 * @see         k_tsk_create_new
 *****************************************************************************/

int k_tsk_init(TASK_INIT *task, int num_tasks)
{
    if (num_tasks > MAX_TASKS - 1) {
        return RTX_ERR;
    }

    g_k_stacks =  k_mpool_alloc(MPID_IRAM2,MAX_TASKS*(KERN_STACK_SIZE>>2)+2000);

    // init task queue
    pq_init(ready_pq);

    // init real time task array.
    for (int i = 0; i < MAX_TASKS; i++)
    {
        rt_tasks[i].deadline = 0;
        rt_tasks[i].period = 0;
        rt_tasks[i].start = 0;
        rt_tasks[i].rt_mbx_size = 0;
    }
    

    // set all tasks' state to UNINITIALIZED
    for (int i = 0; i < MAX_TASKS; i++) {
        g_tcbs[i].state = UNINITIALIZED;
    }

    mailbox_init_all();
    
    TASK_INIT taskinfo;
    k_tsk_init_first(&taskinfo);
    if ( k_tsk_create_new(&taskinfo, &g_tcbs[TID_NULL], TID_NULL) == RTX_OK ) {
        g_num_active_tasks = 1;
        gp_current_task = &g_tcbs[TID_NULL];
    } else {
        g_num_active_tasks = 0;
        return RTX_ERR;
    }

    TASK_INIT kcd_taskinfo;
    k_tsk_init_kcd(&kcd_taskinfo);
    if ( k_tsk_create_new(&kcd_taskinfo, &g_tcbs[TID_KCD], TID_KCD) == RTX_OK ) {
        g_num_active_tasks++;
        // pq_push_back(ready_pq, &g_tcbs[TID_KCD])
    } else {
        return RTX_ERR;
    }
    
    TASK_INIT cdisp_taskinfo;
    k_tsk_init_cdisp(&cdisp_taskinfo);
    if ( k_tsk_create_new(&cdisp_taskinfo, &g_tcbs[TID_CON], TID_CON) == RTX_OK ) {
        g_num_active_tasks++;
        // pq_push_back(ready_pq, &g_tcbs[TID_CON])
    } else {
        return RTX_ERR;
    }

    // TASK_INIT wclck_taskinfo;
    // k_tsk_init_wclock(&wclck_taskinfo);
    // if ( k_tsk_create_new(&wclck_taskinfo, &g_tcbs[TID_WCLCK], TID_WCLCK) == RTX_OK ) {
    //     g_num_active_tasks++;
    //     // pq_push_back(ready_pq, &g_tcbs[TID_CON])
    // } else {
    //     return RTX_ERR;
    // }
    
    // create the rest of the tasks
    for ( int i = 0; i < num_tasks; i++ ) {
        TCB *p_tcb = &g_tcbs[i+1];
        if (k_tsk_create_new(&task[i], p_tcb, i+1) == RTX_OK) {
            g_num_active_tasks++;
        }
    }

    // let scheduler start the first running task
    return k_tsk_run_new();
}

/**************************************************************************//**
 * @brief       initialize a new task in the system,
 *              one dummy kernel stack frame, one dummy user stack frame
 *
 * @return      RTX_OK on success; RTX_ERR on failure
 * @param       p_taskinfo  task initialization structure pointer
 * @param       p_tcb       the tcb the task is assigned to
 * @param       tid         the tid the task is assigned to
 *
 * @details     From bottom of the stack,
 *              we have user initial context (xPSR, PC, SP_USR, uR0-uR3)
 *              then we stack up the kernel initial context (kLR, kR4-kR12, PSP, CONTROL)
 *              The PC is the entry point of the user task
 *              The kLR is set to SVC_RESTORE
 *              20 registers in total
 * @note        YOU NEED TO MODIFY THIS FILE!!!
 *****************************************************************************/
int k_tsk_create_new(TASK_INIT *p_taskinfo, TCB *p_tcb, task_t tid)
{
    extern U32 SVC_RTE;

    U32 *usp;
    U32 *ksp;

    if (p_taskinfo == NULL || p_tcb == NULL)
    {
        return RTX_ERR;
    }

    p_tcb->tid   = tid;
    p_tcb->state = READY;
    p_tcb->prio  = p_taskinfo->prio;
    p_tcb->priv  = p_taskinfo->priv;
    
    /*---------------------------------------------------------------
     *  Step1: allocate user stack for the task
     *         stacks grows down, stack base is at the high address
     * ATTENTION: you need to modify the following three lines of code
     *            so that you use your own dynamic memory allocator
     *            to allocate variable size user stack.
     * -------------------------------------------------------------*/
    
    // dynamically allocate memory for user stack
    int allocated_stack_size = PROC_STACK_SIZE;  // minimum user stack size
    while (allocated_stack_size < p_taskinfo->u_stack_size) {
        allocated_stack_size = allocated_stack_size << 1;
    }
    usp = k_mpool_alloc(MPID_IRAM2, allocated_stack_size);
    if (usp == NULL) {
        errno = ENOMEM;
        return RTX_ERR;
    }

    // set tcb properties
    p_tcb->u_stack_base = (U32 *)((char *)usp + allocated_stack_size);
    p_tcb->u_stack_size = (U32)allocated_stack_size;
    p_tcb->ptask = p_taskinfo->ptask;

    /*-------------------------------------------------------------------
     *  Step2: create task's thread mode initial context on the user stack.
     *         fabricate the stack so that the stack looks like that
     *         task executed and entered kernel from the SVC handler
     *         hence had the exception stack frame saved on the user stack.
     *         This fabrication allows the task to return
     *         to SVC_Handler before its execution.
     *
     *         8 registers listed in push order
     *         <xPSR, PC, uLR, uR12, uR3, uR2, uR1, uR0>
     * -------------------------------------------------------------*/

    // if kernel task runs under SVC mode, then no need to create user context stack frame for SVC handler entering
    // since we never enter from SVC handler in this case

    usp = p_tcb->u_stack_base;
    //usp should be set to the stack base before we start to write things to it below, right?
    *(--usp) = INITIAL_xPSR;             // xPSR: Initial Processor State
    *(--usp) = (U32) (p_taskinfo->ptask);// PC: task entry point

    // uR14(LR), uR12, uR3, uR2, uR1, uR0, 6 registers
    for ( int j = 0; j < 6; j++ ) {

#ifdef DEBUG_0
        *(--usp) = 0xDEADAAA0 + j;
#else
        *(--usp) = 0x0;
#endif
    }

    p_tcb->usp = usp;

    // allocate kernel stack for the task
    ksp = k_alloc_k_stack(tid);
    if ( ksp == NULL ) {
        return RTX_ERR;
    }
    // Check if this is the correct one from k_alloc.
    p_tcb->m_stack_base = ksp;
    // TODO: is this the right location to compute this? i.e. is it okay to use this ksp to compute?
    //I think I've fixed this so no worries


    /*---------------------------------------------------------------
     *  Step3: create task kernel initial context on kernel stack
     *
     *         12 registers listed in push order
     *         <kLR, kR4-kR12, PSP, CONTROL>
     * -------------------------------------------------------------*/
    
    // Check if the line below is necessary.
    // ksp = p_tcb->m_stack_base;
    *(--ksp) = (U32) (&SVC_RTE);
    // kernel stack R4 - R12, 9 registers
#define NUM_REGS 9    // number of registers to push
      for ( int j = 0; j < NUM_REGS; j++) {
#ifdef DEBUG_0
        *(--ksp) = 0xDEADCCC0 + j;
#else
        *(--ksp) = 0x0;
#endif
    }

    // put user sp on to the kernel stack
    *(--ksp) = (U32) usp;

    // save control register so that we return with correct access level
    if (p_taskinfo->priv == 1) {  // privileged 
        *(--ksp) = __get_CONTROL() & ~BIT(0); 
    } else {                      // unprivileged
        *(--ksp) = __get_CONTROL() | BIT(0);
    }

    p_tcb->msp = ksp;

    // Always push to pq after creating a new task.
    pq_push_back(ready_pq, p_tcb);
    return RTX_OK;
}

/**************************************************************************//**
 * @brief       switching kernel stacks of two TCBs
 * @param       p_tcb_old, the old tcb that was in RUNNING
 * @return      RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre         gp_current_task is pointing to a valid TCB
 *              gp_current_task->state = RUNNING
 *              gp_crrent_task != p_tcb_old
 *              p_tcb_old == NULL or p_tcb_old->state updated
 * @note        caller must ensure the pre-conditions are met before calling.
 *              the function does not check the pre-condition!
 * @note        The control register setting will be done by the caller
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @attention   CRITICAL SECTION
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *****************************************************************************/
__asm void k_tsk_switch(TCB *p_tcb_old)
{
        PRESERVE8
        EXPORT  K_RESTORE
        
        PUSH    {R4-R12, LR}                // save general pupose registers and return address
        MRS     R4, CONTROL                 
        MRS     R5, PSP
        PUSH    {R4-R5}                     // save CONTROL, PSP
        STR     SP, [R0, #TCB_MSP_OFFSET]   // save SP to p_old_tcb->msp
K_RESTORE
        LDR     R1, =__cpp(&gp_current_task)
        LDR     R2, [R1]
        LDR     SP, [R2, #TCB_MSP_OFFSET]   // restore msp of the gp_current_task
        POP     {R4-R5}
        MSR     PSP, R5                     // restore PSP
        MSR     CONTROL, R4                 // restore CONTROL
        ISB                                 // flush pipeline, not needed for CM3 (architectural recommendation)
        POP     {R4-R12, PC}                // restore general purpose registers and return address
}


__asm void k_tsk_start(void)
{
        PRESERVE8
        B K_RESTORE
}

/**************************************************************************//**
 * @brief       run a new thread. The caller becomes READY and
 *              the scheduler picks the next ready to run task.
 * @return      RTX_ERR on error and zero on success
 * @pre         gp_current_task != NULL && gp_current_task == RUNNING
 * @post        gp_current_task gets updated to next to run task
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @attention   CRITICAL SECTION
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *****************************************************************************/
int k_tsk_run_new(void)
{
    TCB *p_tcb_old = NULL;
    
    if (gp_current_task == NULL) {
        return RTX_ERR;
    }

    p_tcb_old = gp_current_task;
    gp_current_task = scheduler();
    
    if ( gp_current_task == NULL  ) {
        gp_current_task = p_tcb_old;        // revert back to the old task
        return RTX_ERR;
    }

    // at this point, gp_current_task != NULL and p_tcb_old != NULL
    if (gp_current_task != p_tcb_old) {
        /*if(p_tcb_old->prio <= gp_current_task->prio){
            gp_current_task = p_tcb_old;
        }else{*/
        gp_current_task->state = RUNNING;   // change state of the to-be-switched-in  tcb
        if(p_tcb_old->state == RUNNING){
            p_tcb_old->state = READY;           // change state of the to-be-switched-out tcb
        }
        
        k_tsk_switch(p_tcb_old);            // switch kernel stacks
        //}
    }

    return RTX_OK;
}

 
/**************************************************************************//**
 * @brief       yield the cpu
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:        gp_current_task != NULL &&
 *              gp_current_task->state = RUNNING
 * @post        gp_current_task gets updated to next to run task
 * @note:       caller must ensure the pre-conditions before calling.
 *****************************************************************************/
int k_tsk_yield(void)
{
    if (gp_current_task->prio == PRIO_RT) { // do nothing on rt task
        // errno = EPERM;
        // return RTX_ERR;
        // Is this change neccessary?
        return RTX_OK;
    }

    if (gp_current_task != NULL && gp_current_task->state == RUNNING) {
        if (gp_current_task->prio < peek(ready_pq)->prio){
            return RTX_OK;
        }
        yielding = 1;
        pq_push_back(ready_pq, gp_current_task);
        int result = 0;
        result = k_tsk_run_new();
        yielding = 0;
        return result;
    }
    return RTX_ERR;
}

/**
 * @brief   get task identification
 * @return  the task ID (TID) of the calling task
 */
task_t k_tsk_gettid(void)
{
    return gp_current_task->tid;
}

/*
 *===========================================================================
 *                             TO BE IMPLEMETED IN LAB2
 *===========================================================================
 */


/**************************************************************************//**
 * @brief       creates user task
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:        N/A
 * @post        a new task TASK is created and added to global task list 
 *****************************************************************************/
int k_tsk_create(task_t *task, void (*task_entry)(void), U8 prio, U32 stack_size)
{
#ifdef DEBUG_0
    printf("k_tsk_create: entering...\n\r");
    printf("task = 0x%x, task_entry = 0x%x, prio=%d, stack_size = %d\n\r", task, task_entry, prio, stack_size);
#endif /* DEBUG_0 */

    // sanity checks first
    // ENOMEM There is not enough memory to support the operation.

    // EAGAIN The system has reached maximum number of tasks.
    if (g_num_active_tasks == MAX_TASKS) {
        errno = EAGAIN;
        return RTX_ERR;
    }
    // EINVAL The prio value is not valid.
    if (prio != LOWEST && prio != LOW && prio != MEDIUM && prio != HIGH) {
        errno = EINVAL;
        return RTX_ERR;
    }
    // other errors: invalid task / task entry / task id
    // task always point to 0 at the beginning. Could remove the 2 conditions below.
    if (task == NULL || task_entry == NULL) {
        errno = EINVAL;
        return RTX_ERR;
    }

    // find the first uninitialized task in g_tcbs
    task_t tid = 0;
    for (int i = 1; i < MAX_TASKS; i++) {
        if (g_tcbs[i].state == UNINITIALIZED || g_tcbs[i].state == DORMANT) {
            tid = i;
            break;
        }
    }

    TASK_INIT task_init_info;
    task_init_info.prio = prio;
    task_init_info.priv = 0; // user
    task_init_info.ptask = task_entry;
    task_init_info.tid = tid;
    task_init_info.u_stack_size = stack_size;

    if (RTX_OK == k_tsk_create_new(&task_init_info, &g_tcbs[tid], tid)) {
        g_num_active_tasks++;
    } else {
        #ifdef DEBUG_0
            printf("k_tsk_create: FAILED...\n\r");
        #endif
        return RTX_ERR;
    }
    #ifdef DEBUG_0
        printf("k_tsk_create: SUCCESS...\n\r");
    #endif


    pq_push_front(ready_pq, gp_current_task);
    *task = tid;
    creating = 1;
    int result = 0;
    result = k_tsk_run_new();
    creating = 0;
    return result;
}

void k_tsk_exit(void) 
{
#ifdef DEBUG_0
    printf("k_tsk_exit: entering...\n\r");
#endif /* DEBUG_0 */

    if (gp_current_task->tid == TID_NULL ) return; // exit from NULL task?

    if (RTX_ERR == k_mpool_dealloc(MPID_IRAM2, (void *)((char *)gp_current_task->u_stack_base - gp_current_task->u_stack_size)))
    {
        #ifdef DEBUG_0
            printf("k_tsk_exit: failed to dealloc user stack for task...\n\r");
        #endif /* DEBUG_0 */
        // This function does not return. Page 58 of manual.
        // return RTX_ERR;
    }

    // If there are tasks on the waiting list, we have to unblock them 
    task_t calling_tid = gp_current_task->tid == TID_UART ? UART_MAILBOX_ID : gp_current_task->tid;
    while (mailboxes[calling_tid].waiting_q.size > 0) 
    {
        // Unblock these tasks
        task_t waiting_tid = waiting_pq_peek(&(mailboxes[calling_tid].waiting_q));
        waiting_pq_pop(&(mailboxes[calling_tid].waiting_q));
        g_tcbs[waiting_tid].state = READY;
        pq_push_back(ready_pq, &(g_tcbs[waiting_tid]));
        // Then, make sure their sending actions fail.

    }
    
    

    mailbox_free(gp_current_task->tid);

    gp_current_task->state = DORMANT;
    g_num_active_tasks--;
    // lose the ptr if set to null?
    // gp_current_task->m_stack_base = NULL;
    gp_current_task->u_stack_size = 0;
    gp_current_task->usp = NULL;

    exiting = 1;
    k_tsk_run_new();
    exiting = 0;

    return;
}

int k_tsk_set_prio(task_t task_id, U8 prio) 
{
#ifdef DEBUG_0
    printf("k_tsk_set_prio: entering...\n\r");
    printf("task_id = %d, prio = %d.\n\r", task_id, prio);
#endif /* DEBUG_0 */

    // sanity checks first
    //NULL task is allowed to be 'changed' to PRIO_NULL though nothing really need to be done.
    //check https://piazza.com/class/l2ahaqd6n9c6nk?cid=186 for details
    if (task_id < TID_NULL || task_id >= MAX_TASKS ||
        (prio != LOWEST && prio != LOW && prio != MEDIUM && prio != HIGH && prio != PRIO_NULL) ||
        ((prio == PRIO_NULL && task_id != TID_NULL) || (prio != PRIO_NULL && task_id == TID_NULL))){
        errno = EINVAL;
        return RTX_ERR;
    }

    if (gp_current_task->priv == 0 && g_tcbs[task_id].priv == 1) {
        // current task is unprivileged and cannot modify target task's priority
        errno = EPERM;
        return RTX_ERR;
    }

    // return if there is no need to modify state
    // the situation regarding NULL task as described above should also fall into here
    if (g_tcbs[task_id].prio == prio) {
        return RTX_OK;
    }

    // real-time task priority cannot be changed.
    if (g_tcbs[task_id].prio == PRIO_RT) {
        errno = EPERM;
        return RTX_ERR;
    }

    // non-real-time task priority cannot be changed to RT_PRIO
    if (prio == PRIO_RT) {
        errno = EPERM;
        return RTX_ERR;
    }

    switch (g_tcbs[task_id].state)//should be state, not prio
    {
    case UNINITIALIZED:
        return RTX_OK;
    case READY:

// Task B new prio > current RUNNING task A's prio. Preempt.
        if (prio < gp_current_task->prio) {
            pq_push_front(ready_pq, gp_current_task);
            pq_modify_priority(ready_pq, &g_tcbs[task_id], prio);
            return k_tsk_run_new();
        }

// Task B new prio <= current RUNNING task A's prio. 
        if (prio >= gp_current_task->prio) {
            return pq_modify_priority(ready_pq, &g_tcbs[task_id], prio);
        }

    case RUNNING: 
        if (gp_current_task->tid != task_id) { // cannot have two different running tasks
            errno = EINVAL;
            return RTX_ERR;
        }

        if (prio <= g_tcbs[task_id].prio) { // RUNNING task gets a higher prio, so must keep running.
            gp_current_task->prio = prio;
            return RTX_OK;
        } else {
            if (prio < peek(ready_pq)->prio) { // gets lower prio, but still higher than all tasks in queue.
                gp_current_task->prio = prio;
                return RTX_OK;
            } else {
                gp_current_task->prio = prio; // Update the running tasks's prio.
                setting = 1;
                pq_push_back(ready_pq, gp_current_task);
                int result = 0;
                result = k_tsk_run_new();
                setting = 0;
                return result;
            }
        }
        // return RTX_OK;
    case DORMANT:
        return RTX_OK;
    //nothing to be done for DORMANT and UNINITIALIZED state task, just return 0
    case BLK_RECV:
        g_tcbs[task_id].prio = prio;   
        return RTX_OK;
    case BLK_SEND:
        g_tcbs[task_id].prio = prio;
		waiting_pq_move_to_end(&mailboxes[g_tcbs[task_id].waiting_pq_tid].waiting_q, task_id);
        // need to change the waiting list.
        return RTX_OK;
    default:
        errno = EINVAL;
        return RTX_OK;
    }
}

/**
 * @brief   Retrieve task internal information 
 */
int k_tsk_get(task_t tid, RTX_TASK_INFO *buffer)
{
#ifdef DEBUG_0
    printf("k_tsk_get: entering...\n\r");
    printf("tid = %d, buffer = 0x%x.\n\r", tid, buffer);
#endif /* DEBUG_0 */    
    // sanity checks first
    if (buffer == NULL) {
        errno = EFAULT;
        return RTX_ERR;
    }
    if (tid >= MAX_TASKS || tid < TID_NULL) { //also check if tid < 0 here?
        errno = EINVAL;
        return RTX_ERR;
    }
    
    buffer->tid           = tid;
    buffer->prio          = g_tcbs[tid].prio;
    buffer->u_stack_size  = g_tcbs[tid].u_stack_size;
    buffer->priv          = g_tcbs[tid].priv;
    buffer->ptask         = g_tcbs[tid].ptask;
    buffer->k_sp          = (U32)g_tcbs[tid].msp;
    buffer->k_sp_base     = (U32)g_tcbs[tid].m_stack_base;
    buffer->k_stack_size  = KERN_STACK_SIZE;
    buffer->state         = g_tcbs[tid].state;
    buffer->u_sp          = (U32)g_tcbs[tid].usp;
    buffer->u_sp_base     = (U32)g_tcbs[tid].u_stack_base;

    if (k_tsk_gettid() == tid) {
        buffer->u_sp = __get_PSP();
        buffer->k_sp = __get_MSP();
    }

    return RTX_OK;     
}

// returns the number of non-DORMANT tasks in the system
int k_tsk_ls(task_t *buf, size_t count){
#ifdef DEBUG_0
    printf("k_tsk_ls: buf=0x%x, count=%u\r\n", buf, count);
#endif /* DEBUG_0 */
    // sanity checks first
    if (buf == NULL || count == 0) {
        errno = EFAULT;
        return RTX_ERR;
    }

    int final_count = ((U32)count < g_num_active_tasks) ? count : g_num_active_tasks;

    // store non-dormant task IDs (increasing order) in buf
    int buf_index = 0;
    int counter = 0;
    for (task_t i = 0; i < (task_t)MAX_TASKS; i++) {
        if (counter == final_count) {
            break;
        }
        if (g_tcbs[i].state != DORMANT) {
            *(buf+buf_index) = i;
            buf_index++;
            counter++;
        }
    }
    
    return final_count;
}

/*
 *==========================================================================
 *                            LAB4 REAL TIME FUNCTIONS
 *==========================================================================
 */

time_usec tval_to_usec(TIMEVAL* tv) {
    return (time_usec)(tv->sec * 1000000 + tv->usec);
}

void usec_to_tval(TIMEVAL* tv, time_usec t) {
    tv->usec = t % 1000000;
    tv->sec = t / 1000000;
}


int k_rt_tsk_set(TIMEVAL *p_tv)
{
#ifdef DEBUG_0
    printf("k_rt_tsk_set: p_tv = 0x%x\r\n", p_tv);
#endif /* DEBUG_0 */

    // check if calling task is already real time
    if (gp_current_task->prio == PRIO_RT) {
        errno = EPERM;
        return RTX_ERR;
    }

    // check if period is zero or is not a multiple of RTX_TICK_SIZE * MIN_PERIOD
    if (tval_to_usec(p_tv) == 0 
        || tval_to_usec(p_tv) % (RTX_TICK_SIZE * MIN_PERIOD) != 0) {
            errno = EINVAL;
            return RTX_ERR;
    }

    // Not enough memory to support the operation.

    gp_current_task->prio = PRIO_RT;

    // register real time task info
    rt_tasks[gp_current_task->tid].start = g_timer_count;
    rt_tasks[gp_current_task->tid].period = tval_to_usec(p_tv) / RTX_TICK_SIZE;
    rt_tasks[gp_current_task->tid].deadline = g_timer_count + tval_to_usec(p_tv) / RTX_TICK_SIZE;

    //gp_current_task->state = READY;
    // pq_push_rt(ready_pq, gp_current_task);

    k_tsk_run_new();

    return RTX_OK;   
}

int k_rt_tsk_susp(void)
{
#ifdef DEBUG_0
    printf("k_rt_tsk_susp: entering\r\n");
#endif /* DEBUG_0 */

    // check if calling task is not real time
    if (gp_current_task->prio != PRIO_RT) {
        errno = EPERM;
        return RTX_ERR;
    }

    // Not enough memory to support the operation.

    if (rt_tasks[gp_current_task->tid].deadline >= g_timer_count) {
        // the finished job has met the deadline; simply suspend
        gp_current_task->state = SUSPENDED;
        k_tsk_run_new();
        return RTX_OK;
    } else { // missed deadline; release right away.
        do { 
        rt_tasks[gp_current_task->tid].deadline += rt_tasks[gp_current_task->tid].period;
        } while (rt_tasks[gp_current_task->tid].deadline < g_timer_count);

        //gp_current_task->state = READY;
        pq_push_rt(ready_pq, gp_current_task);
        k_tsk_run_new();
        return RTX_OK;
    }  
}

int k_rt_tsk_get(task_t tid, TIMEVAL *buffer)
{
#ifdef DEBUG_0
    printf("k_rt_tsk_get: entering...\n\r");
    printf("tid = %d, buffer = 0x%x.\n\r", tid, buffer);
#endif /* DEBUG_0 */    
    if (buffer == NULL) {
        return RTX_ERR;
    }   

    // calling task is not real time
    if (gp_current_task->prio != PRIO_RT) {
        errno = EPERM;
        return RTX_ERR;
    }

    // task identified by tid is not a real-time task
    if (g_tcbs[tid].prio != PRIO_RT) {
        errno = EINVAL;
        return RTX_ERR;
    }
    
    usec_to_tval(buffer, rt_tasks[tid].period * RTX_TICK_SIZE);
    
    return RTX_OK;
}
/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */

