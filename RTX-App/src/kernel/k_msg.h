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
 * @file        k_msg.h
 * @brief       kernel message passing header file
 *
 * @version     V1.2021.06
 * @authors     Yiqing Huang
 * @date        2021 JUN
 *****************************************************************************/

 
#ifndef K_MSG_H_
#define K_MSG_H_

#include "k_inc.h"

typedef struct ring_buffer {
    U32 head;
    U32 tail;
    size_t len; // currently occupied space.
    size_t capacity; // total size, i.e. user defined mailbox size.
    size_t count; // number of messages. E.g. msg1, msg2, msg3 ...
    void* data;
} RING_BUFFER;

typedef struct mailbox {
    BOOL initialized;
    size_t size;
    RING_BUFFER ring_buf;
    WAITING_PQ waiting_q; // change this.
    size_t capacity;
    void* msg_buf;
} MAILBOX;

extern MAILBOX mailboxes[MAX_TASKS+1];  // tasks' mailboxes (+1 for UART)
#define UART_MAILBOX_ID   10   // the last mailbox is reserved for UART communication
extern const void* blocked_messages[MAX_TASKS+1];  // messaged blocked on send; blocked_messaged[tid] is the blocked msg sent by tid

int k_mbx_create    (size_t size);
int k_send_msg      (task_t receiver_tid, const void *buf);
int k_send_msg_nb   (task_t receiver_tid, const void *buf);
int k_recv_msg      (void *buf, size_t len);
int k_recv_msg_nb   (void *buf, size_t len);
int k_mbx_ls        (task_t *buf, size_t count);
int k_mbx_get       (task_t tid);

int ring_buf_init(RING_BUFFER* rbuf, size_t capacity);
size_t ring_buf_remaining(RING_BUFFER* rbuf);
int ring_buf_add(RING_BUFFER* rbuf, const void* buf);
int ring_buf_remove(RING_BUFFER* rbuf, void* buf, size_t len);
BOOL ring_buf_empty(RING_BUFFER* rbuf);
void ring_buf_free(RING_BUFFER* rbuf);

void mailbox_free(task_t tid);
int mailbox_init(MAILBOX* mailbox, size_t size);
int mailbox_init_all(void);

void memcpy(void* src, void* dest, size_t size);

#endif // ! K_MSG_H_

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */

