#include "queue.h"

// init all priorities heads & ends to NULL
void pq_init(PQ* pq) {
    for (int i = 0; i < NUM_PRIORITIES; i++) {
        pq->heads[i] = NULL;
        pq->ends[i] = NULL;
    }
}

// find index based on a given priority. E.g., PRIO_RT --> 0
int ind(U8 prio) {
    int index;
    switch (prio)
    {
        case PRIO_RT: index = 0; break;
        case HIGH: index = 1; break;
        case MEDIUM: index = 2; break;
        case LOW: index = 3; break;
        case LOWEST: index = 4; break;
        case PRIO_NULL: index = 5; break;
    }
    return index;
}

// push a process to the front of its priority's queue (preempted)
int pq_push_front(PQ* pq, TCB* task) {
    task->next = pq->heads[ind(task->prio)];
    task->prev = NULL;
    if (pq->heads[ind(task->prio)] == NULL) {
        pq->ends[ind(task->prio)] = task;
    } else {
        pq->heads[ind(task->prio)]->prev = task;
    }
    pq->heads[ind(task->prio)] = task;
    return RTX_OK;
}

// append a process to the back of its priority's queue
int pq_push_back(PQ* pq, TCB* task) {
    task->next = NULL;
    task->prev = pq->ends[ind(task->prio)];
    if (pq->heads[ind(task->prio)] == NULL) {
        pq->heads[ind(task->prio)] = task;
    } else {
        pq->ends[ind(task->prio)]->next = task;
    }
    pq->ends[ind(task->prio)] = task;
    return RTX_OK;
}

// return the task with the highest priority in the pq
TCB* peek(PQ* pq) {
    for (int i = 0; i < NUM_PRIORITIES; i++) {
        if (pq->heads[i] == NULL) continue;
        return pq->heads[i];
    }
    // moved into the for loop.
    // return pq->heads[NUM_PRIORITIES - 1]; // NULL task
}

// remove & return the task with the highest priority in the pq 
TCB* pop(PQ* pq) {
    TCB* ret_task = pq->heads[NUM_PRIORITIES - 1];
    for (int i = 0; i < NUM_PRIORITIES - 1; i++) {
        if (pq->heads[i] == NULL) continue;
        ret_task = pq->heads[i];
        pq->heads[i] = pq->heads[i]->next;
        if (pq->heads[i] == NULL) {
            pq->ends[i] = NULL;
        } else {
            pq->heads[i]->prev = NULL;
        }
        break;
    }
    /* Could we remove this? Null task should ALWAYS stay in its prio's queue.
    if (ret_task == NULL) { // need to remove NULL task
        if (pq->heads[NUM_PRIORITIES - 1] == NULL) return NULL;
        ret_task = pq->heads[NUM_PRIORITIES - 1];
        pq->heads[NUM_PRIORITIES - 1] = NULL;
        pq->ends[NUM_PRIORITIES - 1] = NULL;
    }
    */
    return ret_task;
}

// modify the task & pq based on the new priority
int pq_modify_priority(PQ* pq, TCB* task, U8 new_prio) {
    int i = ind(task->prio); // find i first, cuz we are sure to remove this task of prio i.
    TCB* curr = pq->heads[i];
    while (curr != NULL) {
        if (curr->tid == task->tid) { // found the task to be modified.
            if (curr->prev == NULL) { // remove the front.
                pq->heads[i] = curr->next;
                if (pq->heads[i] == NULL) {
                    pq->ends[i] = NULL;
                } else {
                    pq->heads[i]->prev = NULL;
                }
            } else { // remove a middle or end.
                curr->prev->next = curr->next;
                if (curr->next == NULL) { // remove the end.
                    pq->ends[i] = curr->prev;
                } else { // remove a middle.
                    curr->next->prev = curr->prev;
                }
            }

            task->prio = new_prio;
            if (pq_push_back(pq, task) != RTX_OK) {
                #ifdef DEBUG_0
                    printf("k_set_priority: failed to push modified task to pq...\n\r");
                    printf("task_id = %d, new prio = %d.\n\r", task->tid, new_prio);
                #endif /* DEBUG_0 */
                return RTX_ERR;
            }
            return RTX_OK;
        }
        curr = curr->next;
    }
    return RTX_ERR;
}

// push a real time task to the PRIO_RT priority's queue based on its deadline
int pq_push_rt(PQ* pq, TCB* task) {
    if (ind(task->prio) != 0) return RTX_ERR; // new tsk must be real time
    if (pq->heads[0] == NULL) return pq_push_front(pq, task);
    
    // non-empty real time queue; need to look through deadlines
    TCB* temp = pq->heads[0];
    //Same deadline, arbitrary order.
    // 1. if new RT deadline is shorter than head, put to front.
    if (rt_tasks[pq->heads[0]->tid].deadline >= rt_tasks[task->tid].deadline) {
        return pq_push_front(pq, task);
    }
    // 2. if new RT deadline is longer than tail, put to back.
    if (rt_tasks[pq->ends[0]->tid].deadline <= rt_tasks[task->tid].deadline) {
        return pq_push_back(pq, task);
    }

    // find location to insert in the middle
    while (rt_tasks[temp->tid].deadline <= rt_tasks[task->tid].deadline) {
        temp = temp->next;
    }

    // now we need to insert task before temp
    task->next = temp;
    temp->prev->next = task;
    task->prev = temp->prev;
    temp->prev = task;

    // task->next = temp->next;
    // temp->next = task;
    // task->prev = temp;
    // task->next->prev = task;

    return RTX_OK;
}

TCB* pq_pop_rt(PQ* pq, TCB* task) {
    task_t tid = task->tid;
    // if (ind(task->prio) != 0) return RTX_ERR; // new tsk must be real time
    // if (pq->heads[0] == NULL) return RTX_ERR; // rt pq is empty
    if (pq->heads[0]->tid == task->tid) return pop(pq);

    TCB* temp = pq->heads[0];
    while (temp->next && temp->next->tid != task->tid) {
        temp = temp->next;
    }

    // if (!temp->next) return RTX_ERR; // task is not in real time pq

    // remove task after temp
    temp->next = task->next;
    
    if (task->next) {
        task->next->prev = temp;
    } else {
        pq->ends[0] = temp;
    }

    return task;
}

TCB* peek_rt(PQ* pq) {
    return pq->heads[0];
}

void waiting_pq_init(WAITING_PQ* waiting_pq) {
    waiting_pq->size = 0;
    waiting_pq->head = NULL;
    waiting_pq->end = NULL;
}

void waiting_pq_move_to_end(WAITING_PQ* waiting_pq, task_t tid){
		if(waiting_pq->size == 1){
			return;
		}else{
			TCB* p1 = waiting_pq->head->next;
			TCB* p2 = waiting_pq->head;
			TCB* p3 = waiting_pq->head->next;
			
			while(p3->next){
				p3 = p3->next;
				if(p3->next->tid == tid){
					p1 = p3->next;
					p2 = p3;
				}					
			}
			
			if(p3 == p1){
				return;
			}
			
			p2->next = p1->next;
			p3->next = p1;
			p1->next = NULL;
		}
		return;
}

task_t waiting_pq_peek(WAITING_PQ* waiting_pq) {

    TCB* iterator = waiting_pq->head;
    int index = 0;
	int highest_prio = 1000;
    TCB* ret_task = waiting_pq->head;
    for (int i = 0; i < waiting_pq->size; i++) {
        if (iterator && iterator->prio < highest_prio){
					index = i;
					highest_prio = iterator->prio;
                    ret_task = iterator;
				}
        iterator = iterator->next;
    }

    return ret_task->tid;
}

TCB* waiting_pq_pop(WAITING_PQ* waiting_pq) {
    if (waiting_pq_empty(waiting_pq) == TRUE) return NULL;

    TCB* temp = waiting_pq->head;
    if (waiting_pq->size == 1) { // deleting only element
        waiting_pq->head = NULL;
        waiting_pq->end = NULL;
        waiting_pq->size = 0;
        return temp;
    }

    // 1. iterate through queue to find uindex of highest priority
    int index = 0;
	int highest_prio = 1000;
    for (int i = 0; i < waiting_pq->size; i++) {
        if (temp && temp->prio < highest_prio){
					index = i;
					highest_prio = temp->prio;
				}
        temp = temp->next;
    }

    // 2. delete max priority node from queue
    if(index == 0){
        waiting_pq->head = waiting_pq->head->next;
        waiting_pq->size--;
    }else{
        TCB* prev = waiting_pq->head;
        for (int i = 0; i < index - 1; i++) prev = prev->next; // now prev->next == max priority task
        temp = prev->next;
        prev->next = temp->next;
        waiting_pq->size--;
        if (index == waiting_pq->size - 1) waiting_pq->end = prev->next;
    }

    return waiting_pq->head;
}

int waiting_pq_push(WAITING_PQ* waiting_pq, TCB* task) {
    if (waiting_pq_empty(waiting_pq) == TRUE) {  // empty queue, need to set both head and tail to new task
        waiting_pq->head = task;
        waiting_pq->end = task;
        waiting_pq->size = 1;
        return RTX_OK;
    }
    
    waiting_pq->end->next = task;
    waiting_pq->end = task;
    waiting_pq->size++;
    return RTX_OK;
}

BOOL waiting_pq_empty(WAITING_PQ* waiting_pq) {
    return waiting_pq->size == 0 ? TRUE : FALSE;
}

void queue_init(QUEUE* queue) {
    queue->size = 0;
    queue->capacity = MAX_TASKS;
    for (int i = 0; i < MAX_TASKS; i++) {
        queue->items[i] = MAX_TASKS; // init to 10 to denote "empty", as no tid should be equal to 10.
    }
}

BOOL queue_isFull(QUEUE* queue) {
    return queue->size == queue->capacity;
}

BOOL queue_isEmpty(QUEUE* queue) {
    return queue->size == 0;
}

int queue_push_back(QUEUE* queue, TCB* task) {
    if (queue_isFull(queue) == TRUE) return RTX_ERR;

    queue->items[queue->size++] = task->tid;
    return RTX_OK;
}

task_t queue_peak(QUEUE* queue) {
    if (queue->size == 0) return MAX_TASKS; // neccessary? Below we've returned first item anyways.
    return queue->items[0];
}

void queue_pop(QUEUE* queue) {
    // if queue is empty
    if (queue->size == 0)  return;

    // shift all the elements from index 1 till rear to the left by one
    for (int i = 0; i < queue->size - 1; i++) {
        queue[i] = queue[i + 1];
    }

    // decrement size
    queue->size--;

    // probably need to reset the item at index "size" to 10 to show that it's "empty" again.
    // this also covers edge case when size == 1, in which case the first and also last item in "items" is marked "empty".
    queue->items[queue->size] = MAX_TASKS;
}
