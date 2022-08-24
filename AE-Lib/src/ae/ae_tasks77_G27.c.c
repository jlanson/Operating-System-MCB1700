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
 * @file        ae_tasks100.c
 * @brief       P1 public testing suite 100
 *
 * @version     V1.2022.05
 * @authors     Yiqing Huang
 * @date        2022 May
 * *
 *****************************************************************************/

#include "ae_tasks.h"
#include "uart_polling.h"
#include "printf.h"
#include "ae.h"
#include "ae_util.h"
#include "ae_tasks_util.h"

/*
 *===========================================================================
 *                             MACROS
 *===========================================================================
 */
    
#define NUM_TESTS       1       // number of tests
#define NUM_INIT_TASKS  1       // number of tasks during initialization

/*
 *===========================================================================
 *                             GLOBAL VARIABLES 
 *===========================================================================
 */

TASK_INIT    g_init_tasks[NUM_INIT_TASKS];
const char   PREFIX[]      = "TS1     ";
const char   PREFIX_LOG[]  = "TS1-LOG ";
const char   PREFIX_LOG2[] = "TS1-LOG2";

AE_XTEST     g_ae_xtest;                // test data, re-use for each test
AE_CASE      g_ae_cases[NUM_TESTS];
AE_CASE_TSK  g_tsk_cases[NUM_TESTS];

task_t priv_task;

void set_ae_init_tasks (TASK_INIT **pp_tasks, int *p_num)
{
    *p_num = NUM_INIT_TASKS;
    *pp_tasks = g_init_tasks;
    set_ae_tasks(*pp_tasks, *p_num);
}

// initial task configuration
void set_ae_tasks(TASK_INIT *tasks, int num)
{
    for (int i = 0; i < num; i++ ) {                                                 
        tasks[i].u_stack_size = PROC_STACK_SIZE;    
        tasks[i].prio = HIGH;
        tasks[i].priv = 1;
    }
    tasks[0].priv  = 1;
    tasks[0].ptask = &priv_task1;
    
    init_ae_tsk_test();
}

void init_ae_tsk_test(void)
{
    g_ae_xtest.test_id = 0;
    g_ae_xtest.index = 0;
    g_ae_xtest.num_tests = NUM_TESTS;
    g_ae_xtest.num_tests_run = 0;
    
    for ( int i = 0; i< NUM_TESTS; i++ ) {
        g_tsk_cases[i].p_ae_case = &g_ae_cases[i];
        g_tsk_cases[i].p_ae_case->results  = 0x0;
        g_tsk_cases[i].p_ae_case->test_id  = i;
        g_tsk_cases[i].p_ae_case->num_bits = 0;
        g_tsk_cases[i].pos = 0;  // first avaiable slot to write exec seq tid
        // *_expt fields are case specific, deligate to specific test case to initialize
    }
    printf("%s: START\r\n", PREFIX);
}

void update_ae_xtest(int test_id)
{
    g_ae_xtest.test_id = test_id;
    g_ae_xtest.index = 0;
    g_ae_xtest.num_tests_run++;
}

void gen_req0(int test_id)
{
    g_tsk_cases[test_id].p_ae_case->num_bits = 5;  
    g_tsk_cases[test_id].p_ae_case->results = 0;
    g_tsk_cases[test_id].p_ae_case->test_id = test_id;
    g_tsk_cases[test_id].len = 16; // assign a value no greater than MAX_LEN_SEQ
    g_tsk_cases[test_id].pos_expt = 0; // N/A for P1 tests
       
    update_ae_xtest(test_id);
}

void task0(void){
    char* buf[MSG_HDR_SIZE+8];

    RTX_MSG_HDR *msg = (void*)buf;

    sprintf((char*)(msg+1), "%d ", tsk_gettid());

    msg->type= 0;

    msg->length = MSG_HDR_SIZE + 4;
    msg->sender_tid= tsk_gettid();
    
    send_msg(priv_task, msg);

    tsk_exit();
}

int test0_start(int test_id){


    int retval;
    int result;

    
    priv_task = tsk_gettid();
    gen_req0(0);

    //test 0-0 Creating a mailbox for the priv task
    U8 *p_index    = &(g_ae_xtest.index);
    *p_index = 0;
    strcpy(g_ae_xtest.msg, "creating a mailbox for priv task");
    if(mbx_create((8 + MSG_HDR_SIZE)*3) != -1){
        result = 1;
    }else{
        result = 0;
    }
    
    process_sub_result(test_id, *p_index, result);  
    
    //test 0-1 Trying to create a second mailbox 
    (*p_index)++;
    strcpy(g_ae_xtest.msg, "creating a mailbox when one already exists");
    if(mbx_create((8 + MSG_HDR_SIZE)*3) != -1 && errno == EEXIST){
        result =0;
        errno = 0;
    }else{
        result =1;
    }
    process_sub_result(test_id, *p_index, result);  

    //test 0-2 Sending an empty message to itslef
    strcpy(g_ae_xtest.msg, "Sending a msg to its own mailbox");
    (*p_index)++;

    char buf[MSG_HDR_SIZE+8];
    RTX_MSG_HDR *msg = (void*)buf;
    sprintf((char*)(msg+1), "000");
    msg->length = MSG_HDR_SIZE + 8;
    msg->type = 0;
    msg->sender_tid = tsk_gettid();
    
    if(send_msg(tsk_gettid(), msg) == 0){
        result = 1;
    }else{
        result =0;
    }
    process_sub_result(test_id, *p_index, result); 

    //test 0-3 Filling up the mailbox and creating tasks 
    strcpy(g_ae_xtest.msg, "Filing up mailbox and creating tasks");
    (*p_index)++;
    send_msg(tsk_gettid(), msg);
    send_msg(tsk_gettid(), msg);

    task_t tids[2];

    if(tsk_create(&tids[0], &task0, MEDIUM, 512 ) != 0){
        result = 0;
    }else{
        result =1;
    }
    tsk_create(&tids[0], &task0, MEDIUM, 512); 
    process_sub_result(test_id, *p_index, result); 

    //test 0-4 Receiving all the messages
    (*p_index)++;
    strcpy(g_ae_xtest.msg, "Receiving all the messages created");
    tsk_set_prio(tsk_gettid(), LOWEST);

    int recv_buf[MSG_HDR_SIZE +8];
    recv_msg_nb(recv_buf, MSG_HDR_SIZE+8);
    recv_msg_nb(recv_buf, MSG_HDR_SIZE+8);
    recv_msg_nb(recv_buf, MSG_HDR_SIZE+8);
    if(recv_msg_nb(recv_buf, MSG_HDR_SIZE+8) != 0){
        result = 0;
    }else{
        result = 1;
    }
    process_sub_result(test_id, *p_index, result); 
}

void priv_task1(void)
{
    // Test ID is 0
    test0_start(0);
    test_exit();
}
