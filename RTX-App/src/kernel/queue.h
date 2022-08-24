#ifndef QUEUE_H
#define QUEUE_H

#include "common.h"

extern TASK_RT rt_tasks[MAX_TASKS];

#define NUM_PRIORITIES 6      // PRIO_NULL, LOWEST, LOW, MED, HIGH, PRIO_RT

typedef struct tcb {
    struct tcb *prev;         /**< prev tcb, not used in the starter code     */
    struct tcb *next;         /**< next tcb, not used in the starter code     */
    U32        *msp;          /**< kernel sp of the task, TCB_MSP_OFFSET = 8  */
    U32        *m_stack_base; /**< kernel stack base, (sp + stack_size)       */
    U32        *usp;          /**< user sp of the task                        */
    U32        *u_stack_base; /**< user stack base (higher address)           */
    U32         u_stack_size; /**< user stack size, (stack base = sp + stack_size)  */
    U8          priv;         /**< = 0 unprivileged, =1 privileged,           */    
    U8          tid;          /**< task id                                    */
    U8          prio;         /**< scheduling priority                        */
    U8          state;        /**< task state                                 */
    void        (*ptask)();   /**< task entry address                         */
		U8 					waiting_pq_tid;
} TCB;

typedef struct pq {
    TCB* heads[NUM_PRIORITIES];
    TCB* ends[NUM_PRIORITIES];
} PQ;

typedef struct waiting_pq { // a single priority queue, priority from HIGH LOW
    int size;
    TCB* head;
    TCB* end;
} WAITING_PQ;

void pq_init(PQ* pq);
int pq_push_front(PQ* pq, TCB* task);
int pq_push_back(PQ* pq, TCB* task);
TCB* peek(PQ* pq);
TCB* pop(PQ* pq);
int pq_modify_priority(PQ* pq, TCB* task, U8 new_prio);
int pq_push_rt(PQ* pq, TCB* task);
int pq_push_rt(PQ* pq, TCB* task);
TCB* peek_rt(PQ* pq);

void waiting_pq_move_to_end(WAITING_PQ* waiting_pq, task_t tid);
void waiting_pq_init(WAITING_PQ* waiting_pq);
task_t waiting_pq_peek(WAITING_PQ* waiting_pq);
TCB* waiting_pq_pop(WAITING_PQ* waiting_pq);
int waiting_pq_push(WAITING_PQ* waiting_pq, TCB* task);
BOOL waiting_pq_empty(WAITING_PQ* waiting_pq);

// sending task waiting list of mailbox
typedef struct queue {
    size_t size; // how many tids are in the array.
    size_t capacity; // equal to 10,  length of the array.
    task_t items[MAX_TASKS]; // array of length 10.
} QUEUE;

// reference: implementation of queue using array (https://www.geeksforgeeks.org/array-implementation-of-queue-simple/)
void queue_init(QUEUE* queue);
BOOL queue_isFull(QUEUE* queue);
BOOL queue_isEmpty(QUEUE* queue);
int queue_push_back(QUEUE* queue, TCB* task);
task_t queue_peak(QUEUE* queue);
void queue_pop(QUEUE* queue);

#endif
