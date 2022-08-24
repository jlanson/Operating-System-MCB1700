/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTX LAB  
 *
 *                     Copyright 2020-2022 Yiqing Huang
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
 * @file        k_msg.c
 * @brief       kernel message passing routines          
 * @version     V1.2021.06
 * @authors     Yiqing Huang
 * @date        2021 JUN
 *****************************************************************************/


#include "k_inc.h"
#include "k_rtx.h"
#include "k_msg.h"

extern int UART_RECV;

MAILBOX mailboxes[MAX_TASKS+1];  // tasks' mailboxes (+1 for UART)
const void* blocked_messages[MAX_TASKS+1];  // messaged blocked on send; blocked_messaged[tid] is the blocked msg sent by tid

int ring_buf_init(RING_BUFFER* rbuf, size_t capacity) {
    rbuf->capacity = capacity;
    rbuf->count = 0;
    rbuf->head = 0;
    rbuf->tail = 0;
    rbuf->len = 0;
    rbuf->data = k_mpool_alloc(MPID_IRAM2, capacity);
    return rbuf->data == NULL ? RTX_ERR : RTX_OK;
}

size_t ring_buf_remaining(RING_BUFFER* rbuf) {
    return rbuf->capacity - rbuf->len;
}

// This is implemented below.
// int mailbox_init(MAILBOX* mailbox, size_t capacity) {
//     if (ring_buf_init(mailbox->ring_buf, capacity) == RTX_ERR) return RTX_ERR;
//     mailbox->initialized = TRUE;
//     mailbox->capacity = capacity;
//     queue_init(&(mailbox->waiting_q));
//     return RTX_OK;
// }

int ring_buf_add(RING_BUFFER* rbuf, const void* buf) {
    RTX_MSG_HDR* msg = (RTX_MSG_HDR*)buf;
    if (ring_buf_remaining(rbuf) < msg->length) return RTX_ERR; // not enough space left.
    rbuf->count++; // a new message is added.
    rbuf->len += msg->length; // update occupied space in the ring buffer.
    
    if (rbuf->capacity - rbuf->tail >= msg->length) {
        // no need to split memory
        // memcpy(msg, (void*)((char*)rbuf->data + msg->length), msg->length); // why this destination?
        memcpy(msg, (void*)((char*)rbuf->data + rbuf->tail), msg->length); // should probably be this one.
        rbuf->tail += msg->length;
    } else { // need to split msg into half (attach to tail & head)
        size_t first_half_size = rbuf->capacity - rbuf->tail;
        size_t second_half_size = msg->length - first_half_size;
        memcpy(msg, (void*)((char*)rbuf->data + rbuf->tail), first_half_size);
        memcpy((void*)((char*)msg + first_half_size), rbuf->data, second_half_size);
        rbuf->tail = second_half_size;
    }
    return RTX_OK;
}

int ring_buf_remove(RING_BUFFER* rbuf, void* buf, size_t size) {
    // should this be a pointer?
    RTX_MSG_HDR* msg; // first copy 
    
    // copy msg header first (byte by byte) to msg. This is only used to get msg->length.
    for (int i = 0; i < MSG_HDR_SIZE; i++) {
        memcpy((void*)((char*)rbuf->data + (rbuf->head + i) % rbuf->capacity),
            (void*)((char*)msg + i),
            1
        );
    }

    // when will this happen?
    // check if the msg is large enough
    if (msg->length > size) return RTX_ERR;

    // copy the entire msg to buf. buf is what is returned, msg is just used to get msg->length to decide how many data to remove. 
    for (int i = 0; i < msg->length; i++) {
        memcpy((void*)((char*)rbuf->data + (rbuf->head + i) % rbuf->capacity),
            (void*)((char*)buf + i),
            1
        );
    }

    // modify the ring buffer accordingly
    rbuf->count--;
    rbuf->head = (rbuf->head + msg->length) % rbuf->capacity;
    rbuf->len -= msg->length; // occupied space reduces, cuz we've removed the message starting at "head".
    return RTX_OK;
}

BOOL ring_buf_empty(RING_BUFFER* rbuf) {
    return rbuf->count == 0;
}

void ring_buf_free(RING_BUFFER* rbuf) {
    k_mpool_dealloc(MPID_IRAM2, rbuf->data);
}
// Refer to p62, if a task exits, do we only deallocate the ring_buf within the mailbox?
void mailbox_free(task_t tid) {
    task_t mailbox_id = tid == TID_UART ? UART_MAILBOX_ID : tid;
    if (mailboxes[mailbox_id].initialized == TRUE) {
        ring_buf_free(&mailboxes[mailbox_id].ring_buf);
        mailboxes[mailbox_id].initialized = FALSE;
        mailboxes[mailbox_id].capacity = 0;
        mailboxes[mailbox_id].size = 0;
    }
}

int mailbox_init(MAILBOX* mailbox, size_t size) {
    if (mailbox->initialized == TRUE) return RTX_ERR;
    if (ring_buf_init(&mailbox->ring_buf, size) == RTX_ERR) return RTX_ERR;
    waiting_pq_init(&mailbox->waiting_q);
    mailbox->initialized = TRUE;
    mailbox->capacity = size; // size is ring buffer size, and also mailbox size for messages (excluding meta data).
    mailbox->size = NULL; // no used space at initialization.
    return RTX_OK;
}

int mailbox_init_all(void) {
    for (int i = 0; i < MAX_TASKS + 1; i++) {
        mailboxes[i].initialized = FALSE;
    }
    return mailbox_init(&mailboxes[UART_MAILBOX_ID], UART_MBX_SIZE);
}

void memcpy(void* src, void* dest, size_t size) {
    char* src_ptr = (char*) src;
    char* dest_ptr = (char*) dest;

    for (size_t i = 0; i < size; i++) {
        *(dest_ptr + i) = *(src_ptr + i);
    }
}

int k_mbx_create(size_t size) {
#ifdef DEBUG_0
    printf("k_mbx_create: size = %u\r\n", size);
#endif /* DEBUG_0 */
    // calling task already has a mailbox
    // remove first if sub-condition, cuz cannot compare MAILBOX type with NULL
    if (mailboxes[gp_current_task->tid].initialized == TRUE) {
        errno = EEXIST;
        return RTX_ERR;
    }

    // *size* argument is less than MIN_MSG_SIZE
    if (size < MIN_MSG_SIZE) {
        errno = EINVAL;
        return RTX_ERR;
    }

    // not enough memory to support the operation
    if (mailbox_init(&mailboxes[gp_current_task->tid], size) == RTX_ERR) {
        errno = ENOMEM;
        return RTX_ERR;
    }

    return gp_current_task->tid; // mailbox ID = calling task's tid
}


int k_send_msg_shared(task_t receiver_tid, const void *buf, BOOL nb) {
#ifdef DEBUG_0
    printf("k_send_msg_shared: receiver_tid = %d, buf=0x%x\r\n", receiver_tid, buf);
#endif /* DEBUG_0 */

    // receiver identified by *receiver_tid* doesn't exist (TID 0 - 9 OR TID_UART)
    if (receiver_tid != TID_UART && (receiver_tid < TID_NULL || receiver_tid >= MAX_TASKS)) {
        errno = EINVAL;
        return RTX_ERR;
    }

    task_t recv_mailbox_id = receiver_tid == TID_UART ? UART_MAILBOX_ID : receiver_tid;
    
    // receiver exists but has no mailboxes
    if (mailboxes[recv_mailbox_id].initialized == FALSE) {
        errno = ENOENT;
        return RTX_ERR;
    }
    // this check seems to have been covered by the above check, i.e. mailbox's initialized == false
    // if (receiver_tid != TID_UART && 
    //     (g_tcbs[receiver_tid].state == DORMANT || g_tcbs[receiver_tid].state == UNINITIALIZED)) {
    //     // DORMANT / UNINITIALIZED tasks have no mailboxes
    //     errno = ENOENT;
    //     return RTX_ERR;
    // }

    RTX_MSG_HDR* msg_header = (RTX_MSG_HDR*)buf; // casting here to avoid casting multiple times
    
    if (msg_header != NULL && msg_header->length < MIN_MSG_SIZE) {
        errno = EINVAL;
        return RTX_ERR;
    }

    if (msg_header != NULL && msg_header->length > mailboxes[recv_mailbox_id].capacity) {
        errno = EMSGSIZE;
        return RTX_ERR;
    }

    if (buf == NULL) {
        errno = EFAULT;
        return RTX_ERR;
    }

    // This is when RUNNING -> BLK_SEND for blocking, or ERROR for non-blocking.
    if (ring_buf_remaining(&(mailboxes[recv_mailbox_id].ring_buf)) < msg_header->length || !waiting_pq_empty(&(mailboxes[recv_mailbox_id].waiting_q))) {
        // Two possible situations.
        // receiver's mailbox does not have enough free space for message
        // Or, receiver's waiting list is not empty.

        // Case 1: Non-blocking, therefore returns -1 for both situation.
        if (nb == TRUE) {
            errno = ENOSPC;
            return RTX_ERR;
        }

        // Case 2: Blocking. Gets blocked for both situation.
        gp_current_task->state = BLK_SEND;
        gp_current_task->waiting_pq_tid = recv_mailbox_id;
        waiting_pq_push(&(mailboxes[recv_mailbox_id].waiting_q), gp_current_task);
        blocked_messages[gp_current_task->tid] = buf; // ???
        return k_tsk_run_new(); // should directly return.
    } 
    
    else {
    // This is when RUNNING stays RUNNING for blocking and non-blocking.
    // ??? Why don't send if receiver is in BLK_RECV?
        if (ring_buf_add(&(mailboxes[recv_mailbox_id].ring_buf), buf) == RTX_ERR) return RTX_ERR;
    }
    
   
 
 
    // if (receiver_tid == TID_UART || g_tcbs[receiver_tid].state != BLK_RECV) {
    //     return ring_buf_add(&(mailboxes[recv_mailbox_id].ring_buf), buf);
    // }

    // If receiver is in state BLK_RECV, the delivered msg leads to BLK_RECV -> READY
    // receiver_tid should NOT be 10, otherwise g_tcbs[10] is an access violation.
    if (receiver_tid != TID_UART && g_tcbs[receiver_tid].state == BLK_RECV) {
        memcpy((void *)buf, mailboxes[recv_mailbox_id].msg_buf, msg_header->length); // ? What's this line doing?
        g_tcbs[receiver_tid].state = READY;
        if (rt_tasks[receiver_tid].period != 0) {
            pq_push_rt(ready_pq, &g_tcbs[receiver_tid]);
        } else {
            pq_push_back(ready_pq, &g_tcbs[receiver_tid]);
        }
    }
    
    // The scheduler then makes new scheduling decision.
    // We probably don't need the if condition here.
    // Notice that if uart is currently calling k_send_nb, it doesn't matter,
    // uart is not a task, but the lines below will check the real "current task" instead of uart and make decision.
    if (gp_current_task->prio > peek(ready_pq)->prio)
    {
        // gp_current_task->state = READY;
        pq_push_front(ready_pq, gp_current_task);
        k_tsk_run_new();
    }

    if (gp_current_task->prio == PRIO_RT && peek_rt(ready_pq) 
        && rt_tasks[gp_current_task->tid].deadline > rt_tasks[peek_rt(ready_pq)->tid].deadline) {
        // preempted because current task has a later deadline
        pq_push_rt(ready_pq, gp_current_task);
        k_tsk_run_new();
    }
    
    // if (g_tcbs[receiver_tid].prio < gp_current_task->prio) { // preempt
    //     gp_current_task->state = READY;
    //     pq_push_front(ready_pq, gp_current_task);
    //     return k_tsk_run_new(); // should directly return
    // }

    return RTX_OK;
}

int k_send_msg(task_t receiver_tid, const void *buf) {
#ifdef DEBUG_0
    printf("k_send_msg: receiver_tid = %d, buf=0x%x\r\n", receiver_tid, buf);
#endif /* DEBUG_0 */

    int retval = k_send_msg_shared(receiver_tid, buf, FALSE);

    //Checking if the receiver/mailbox still exists after the message has been sent 
    if(k_mbx_get(receiver_tid) == -1 && (receiver_tid < TID_NULL || receiver_tid >= MAX_TASKS)){
        errno = EINVAL;
        return -1;
    }else if(k_mbx_get(receiver_tid) == -1){
        errno = ENOENT;
        return -1;
    }else{
        return retval;
    }
}

int k_send_msg_nb(task_t receiver_tid, const void *buf) {
#ifdef DEBUG_0
    printf("k_send_msg_nb: receiver_tid = %d, buf=0x%x\r\n", receiver_tid, buf);
#endif /* DEBUG_0 */

    return k_send_msg_shared(receiver_tid, buf, TRUE);
}


int k_recv_msg_shared(void *buf, size_t len, BOOL nb) {
#ifdef DEBUG_0
    printf("k_recv_msg_shared: buf=0x%x, len=%d\r\n", buf, len);
#endif /* DEBUG_0 */

    if (buf == NULL) {
        errno = EFAULT;
        return RTX_ERR;
    }

    // notice that gp_current_task->tid will NEVER be equal to TID_UART.
    task_t recv_mailbox_id = gp_current_task->tid == TID_UART ? UART_MAILBOX_ID : gp_current_task->tid;

    if (UART_RECV == 1) {
        recv_mailbox_id = UART_MAILBOX_ID;
    }
    
    // the calling task does not have a mailbox
    if (mailboxes[recv_mailbox_id].initialized == FALSE) {
        errno = ENOENT;
        return RTX_ERR;
    }

    // receiver (caller)'s mailbox is empty.
    if (ring_buf_empty(&(mailboxes[recv_mailbox_id].ring_buf)) == TRUE) {
        // Non-blocking will return -1 immediately.
        if (nb == TRUE) {
            errno = ENOMSG;
            return RTX_ERR;
        }
        // Blocking will change RUNNING -> BLK_RECV.
        gp_current_task->state = BLK_RECV;
        mailboxes[recv_mailbox_id].msg_buf = buf; // What's this line doing?
        // what's this line doing?
        // mailboxes[recv_mailbox_id].len = len; 
        k_tsk_run_new();
        // return RTX_OK;
    }

    // receiver (caller)'s mailbox is not empty. Therefore, both non-blocking and blocking could receive.
    // there are messages to receive; need to check if the buf is large enough.
    if (ring_buf_remove(&(mailboxes[recv_mailbox_id].ring_buf), buf, len) == RTX_ERR) {
        // the buffer is too small to hold the message.
        errno = ENOSPC;
        return RTX_ERR;
    } 

    // if reaches here, it means non-blocking or blocking recv succeeds.
    // Accordingly, mailbox has more free space, so next we check for possible preemptions.
    // BOOL preempt = FALSE;

    // check if there are tasks blocked on sending msg to current task
    while (mailboxes[recv_mailbox_id].waiting_q.size > 0) {
        // Return the task id of the highest prio BLK_SEND in the waiting list.
        task_t waiting_tid = waiting_pq_peek(&(mailboxes[recv_mailbox_id].waiting_q));
        // Fetch the message so that the kernel could send it later.
        RTX_MSG_HDR* blocked_msg = (RTX_MSG_HDR*) blocked_messages[waiting_tid];
        // error checking.
        if (blocked_msg == NULL 
            || ring_buf_remaining(&(mailboxes[recv_mailbox_id].ring_buf)) < blocked_msg->length) {
                break;
                // return RTX_ERR;
        }   
        // if reaches here, it means the new free space is larger than msg to-be-sent.
        // therefore, do 4 things:
        // 1. Kernel copies the message to the ring_buf
        // 2. Dequeue the BLK_SEND task from the waiting list.
        // 3. The BLK_SEND task's state goes from BLK_SEND -> READY
        // 4. Add it to the ready queue.
        ring_buf_add(&(mailboxes[recv_mailbox_id].ring_buf), blocked_msg);
        waiting_pq_pop(&(mailboxes[recv_mailbox_id].waiting_q));
        g_tcbs[waiting_tid].state = READY;
        if (rt_tasks[waiting_tid].period != 0) {
            pq_push_rt(ready_pq, &g_tcbs[waiting_tid]);
        } else {
            pq_push_back(ready_pq, &g_tcbs[waiting_tid]);
        }
        // pq_push_back(ready_pq, &(g_tcbs[waiting_tid]));

        // Don't need this, because kernel sends all the messages and then all blocked_send tasks are in ready queue.
        // if (g_tcbs[waiting_tid].prio < gp_current_task->prio) { // should later preempt the current task
        //     preempt = TRUE;
        // }
    }

    // check if newly unblocked tasks will preempt the current running task (if not in UART)
    if (/*preempt == TRUE &&*/gp_current_task->tid != TID_UART && gp_current_task->prio > peek(ready_pq)->prio) {
        // gp_current_task->state = READY;
        pq_push_front(ready_pq, gp_current_task);
        k_tsk_run_new();
    }

    if (gp_current_task->prio == PRIO_RT && peek_rt(ready_pq) 
        && rt_tasks[gp_current_task->tid].deadline > rt_tasks[peek_rt(ready_pq)->tid].deadline) {
        // preempted because current task has a later deadline
        pq_push_rt(ready_pq, gp_current_task);
        k_tsk_run_new();
    }

    return RTX_OK;
}

int k_recv_msg(void *buf, size_t len) {
#ifdef DEBUG_0
    printf("k_recv_msg: buf=0x%x, len=%d\r\n", buf, len);
#endif /* DEBUG_0 */
    return k_recv_msg_shared(buf, len, FALSE);
}

int k_recv_msg_nb(void *buf, size_t len) {
#ifdef DEBUG_0
    printf("k_recv_msg_nb: buf=0x%x, len=%d\r\n", buf, len);
#endif /* DEBUG_0 */
    return k_recv_msg_shared(buf, len, TRUE);
}

int k_mbx_ls(task_t *buf, size_t count) { // DO NOT COUNT THE UART MAILBOX
#ifdef DEBUG_0
    printf("k_mbx_ls: buf=0x%x, count=%u\r\n", buf, count);
#endif /* DEBUG_0 */
    if (buf == NULL) {
        errno = EFAULT;
        return RTX_ERR;
    }

    int mailbox_count = 0;
    for (task_t tid = 0; tid < MAX_TASKS; tid++) {
        if (mailboxes[tid].initialized == TRUE) {
            buf[mailbox_count++] = tid;
            if (mailbox_count == count) break;
        }
    }

    return mailbox_count;
}

int k_mbx_get(task_t tid)
{
#ifdef DEBUG_0
    printf("k_mbx_get: tid=%u\r\n", tid);
#endif /* DEBUG_0 */

    if (tid != TID_UART && (tid < TID_NULL || tid >= MAX_TASKS)) { // invalid tid
        errno = ENOENT;
        return RTX_ERR;
    }

    task_t recv_mailbox_id = tid == TID_UART ? UART_MAILBOX_ID : tid;
    if (mailboxes[recv_mailbox_id].initialized == FALSE) { // task does not have a valid mailbox
        errno = ENOENT;
        return RTX_ERR;
    }

    return ring_buf_remaining(&(mailboxes[recv_mailbox_id].ring_buf));
}
/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */

