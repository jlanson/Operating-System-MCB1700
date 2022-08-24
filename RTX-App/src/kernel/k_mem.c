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

/**************************************************************************/ /**
                                                                              * @file        k_mem.c
                                                                              * @brief       Kernel Memory Management API C Code
                                                                              *
                                                                              * @version     V1.2021.01.lab2
                                                                              * @authors     Yiqing Huang
                                                                              * @date        2021 JAN
                                                                              *
                                                                              * @note        skeleton code
                                                                              *
                                                                              *****************************************************************************/

#include "k_inc.h"
#include "k_mem.h"

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
                              |                           |     |
                              |---------------------------|     |
                              |                           |     |
                              |      other data           |     |
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
             g_k_stacks[15]-->|---------------------------|     |
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
 *                            GLOBAL VARIABLES
 *===========================================================================
 */
// kernel stack size, referred by startup_a9.s
const U32 g_k_stack_size = KERN_STACK_SIZE;
// task proc space stack size in bytes, referred by system_a9.c
const U32 g_p_stack_size = PROC_STACK_SIZE;

// task kernel stacks
U32 (*g_k_stacks)[KERN_STACK_SIZE >> 2];
//U32 g_k_stacks[MAX_TASKS][KERN_STACK_SIZE >> 2] __attribute__((aligned(8)));


// task process stack (i.e. user stack) for tasks in thread mode
// remove this bug array in your lab2 code
// the user stack should come from MPID_IRAM2 memory pool
// U32 g_p_stacks[MAX_TASKS][PROC_STACK_SIZE >> 2] __attribute__((aligned(8)));
U32 g_p_stacks[1][PROC_STACK_SIZE >> 2] __attribute__((aligned(8)));

// Global Data Structures
// Should we comment these? Seems like they aren't used.
// extern MEM_BLK* mem_pool_heads_1[8];
// extern MEM_BLK* mem_pool_heads_2[11];

RAM_MEM_SPACE memory_bit_array;
MEM_BLK *mem_pool_1_heads[8] = {NULL};
MEM_BLK *mem_pool_2_heads[11] = {NULL};

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */

/* note list[n] is for blocks with order of n */
mpool_t k_mpool_create(int algo, U32 start, U32 end)
{
    mpool_t mpid = MPID_IRAM1;

#ifdef DEBUG_0
    printf("k_mpool_init: algo = %d\r\n", algo);
    printf("k_mpool_init: RAM range: [0x%x, 0x%x].\r\n", start, end);
#endif /* DEBUG_0 */

    if (algo != BUDDY || end <= start)
    {
        errno = EINVAL;
        return RTX_ERR;
    }

    if (start == RAM1_START)
    {
        if (end > RAM1_END)
        { // check if end boundary is valid
            errno = ENOMEM;
            return RTX_ERR;
        }

        // create and initialize the top level memory block (representing an entire free memory pool)
        MEM_BLK *temp_mem_blk = (MEM_BLK *)RAM1_START;
        temp_mem_blk->level = 0;
        temp_mem_blk->prev = NULL;
        temp_mem_blk->next = NULL;
        // !!!! Need to change to real size, not a fixed size !!!
        temp_mem_blk->size = RAM1_SIZE;
        mem_pool_1_heads[0] = temp_mem_blk;
    }
    else if (start == RAM2_START)
    {
        if (end > RAM2_END)
        {
            errno = ENOMEM;
            return RTX_ERR;
        }
        mpid = MPID_IRAM2;
        // create and initialize the top level memory block (representing an entire free memory pool)
        // create and initialize the top level memory block (representing an entire free memory pool)
        MEM_BLK *temp_mem_blk = (MEM_BLK *)RAM2_START;
        temp_mem_blk->level = 0;
        temp_mem_blk->prev = NULL;
        temp_mem_blk->next = NULL;
        temp_mem_blk->size = RAM2_SIZE;
        mem_pool_2_heads[0] = temp_mem_blk;
    }
    else
    {
        errno = EINVAL;
        return RTX_ERR;
    }

    return mpid;
}

// return the smallest k where 2^k >= n
int findNextK(int n)
{
    unsigned int i = 1;
    for (int k = 0; k >= 0; k++)
    {
        if (i >= n)
        {
            return k;
        }
        i = i * 2;
    }
    return -1;
}

// return the level of a 2^k memory block in mpid's bit array
int expoToLevel(int mpid, int k)
{
    if (k < 5)
    {
        if (mpid == MPID_IRAM1)
        {
            return 7;
        }
        else
        {
            return 10;
        }
    }
    // return RTX_ERR; // MIN_BLK_SIZE = 32 = 2^5

    if (mpid == MPID_IRAM1)
    { // IRAM1_MEM_POOL_SIZE = 4096 = 2^12
        if (k > 12)
            return RTX_ERR;
        return 12 - k;
    }
    else if (mpid == MPID_IRAM2)
    { // IRAM2_MEM_POOL_SIZE = 32768 = 2^15
        if (k > 15)
            return RTX_ERR;
        return 15 - k;
    }
    return RTX_ERR;
}

// return 2^power
int pow2(int power)
{
    return (1 << power);
}

// return starting address of mem block given mpid and the block's index in bit array (0-indexed)
void *computeMemBlkAddress(mpool_t mpid, int level, int index)
{
    int mem_pool_starting_address = mpid == MPID_IRAM1 ? RAM1_START : RAM2_START;
    int block_size = mpid == MPID_IRAM1 ? 4096 / pow2(level) : 32768 / pow2(level); // ex. mpid = 1, level = 1 (blockSize = 4096/2^1 = 2048)
    return ((void *)(mem_pool_starting_address + block_size * index));
}

int computeBitArrayIndex(U16 level, U16 level_index)
{
    return (int)((pow2((int)level)) - 1 + level_index);
}

// mark as allocated in bit array
// this function only works if the allocated memory is not the starting address for levels other than level 0
void markMemAllocated(mpool_t mpid, MEM_BLK *mem_blk)
{
    if (mpid == MPID_IRAM1)
    {
        int level_index = (int)((int)mem_blk - RAM1_START) / mem_blk->size;
        SetBit(memory_bit_array.IRAM1_MEM_POOL, computeBitArrayIndex(mem_blk->level, (U16)level_index));
    }
    else
    {
        int level_index = (int)((int)mem_blk - RAM2_START) / mem_blk->size;
        SetBit(memory_bit_array.IRAM2_MEM_POOL, computeBitArrayIndex(mem_blk->level, (U16)level_index));
    }
}

// traverse from level l down to level k, splitting blocks into two and marking one as allocated at each level
void *topDownTraverseFreeLists(mpool_t mpid, MEM_BLK *parent_node, int l, int k)
{
    while (l < k)
    { // split the big free block into two smaller blocks; first is used later, second is put on free list
        // mark big block as allocated in bit array
        markMemAllocated(mpid, parent_node);

        // bottom levels' free lists are empty, so put the second (right) child onto free list
        MEM_BLK *right_node = (MEM_BLK *)((char *)parent_node + parent_node->size / 2);
        right_node->level = l + 1;
        right_node->prev = NULL;
        right_node->next = NULL;
        right_node->size = parent_node->size / 2;
        if (mpid == MPID_IRAM1)
        {
            mem_pool_1_heads[l + 1] = right_node;
        }
        else
        {
            mem_pool_2_heads[l + 1] = right_node;
        }

        // initialize the first (left) child and set as parent node to be used later
        MEM_BLK *left_node = parent_node;
        left_node->level = l + 1;
        left_node->prev = NULL;
        left_node->next = NULL;
        left_node->size = parent_node->size / 2;
        parent_node = left_node;

        l++;
    }
    // we have reached level k;
    // parent node is now the node we want to allocate
    markMemAllocated(mpid, parent_node);
    return ((void *)parent_node);
}

void *k_mpool_alloc(mpool_t mpid, size_t size)
{
#ifdef DEBUG_0
    printf("k_mpool_alloc: mpid = %d, size = %d, 0x%x\r\n", mpid, size, size);
#endif /* DEBUG_0 */

    // sanity checks
    if (size == 0)
        return NULL;
    if (mpid != MPID_IRAM1 && mpid != MPID_IRAM2)
    { // invalid mpid
        errno = EINVAL;
        return NULL;
    }
    if ((mpid == MPID_IRAM1 && size > RAM1_SIZE) || (mpid == MPID_IRAM2 && size > RAM2_SIZE))
    { // not enough memory
        errno = ENOMEM;
        return NULL;
    }

    // step 1: find smallest k where 2^(k-1) < size <= 2^k. Notice this is NOT the level.
    int k = findNextK((int)size);

    // step 2: check if there are free blocks with size of 2^k; use the first free block of size 2^k found
    int level = expoToLevel(mpid, k);
    if (mpid == MPID_IRAM1)
    {
        if (mem_pool_1_heads[level])
        { // there is at least one free block
            // remove block from free list
            MEM_BLK *allocated_blk = mem_pool_1_heads[level];
            mem_pool_1_heads[level] = allocated_blk->next;
            if (allocated_blk->next)
                allocated_blk->next->prev = allocated_blk->prev;

            // if (allocated_blk->prev) allocated_blk->prev->next = allocated_blk->next;
            // Don't think we need this line because we always remove the first node in the free list

            // mark as allocated in bit array
            markMemAllocated(mpid, allocated_blk);
            return ((void *)allocated_blk);
        }
    }
    else
    { // MPID_IRAM2
        if (mem_pool_2_heads[level])
        { // there is at least one free block
            // remove block from free list
            MEM_BLK *allocated_blk = mem_pool_2_heads[level];
            mem_pool_2_heads[level] = allocated_blk->next;
            if (allocated_blk->next)
                allocated_blk->next->prev = allocated_blk->prev;
            // if (allocated_blk->prev)
            //     allocated_blk->prev->next = allocated_blk->next;

            // mark as allocated in bit array
            markMemAllocated(mpid, allocated_blk);
            return ((void *)allocated_blk);
        }
    }

    // step 3: find the smallest free block by incrementing k up until m where m = log_2(mem_pool_size);
    //         keep splitting the big free block until we have a free block of size 2^k
    int l = level - 1; // We don't have free block on level `level`, so move up a level.
    if (l < 0)
    { // no free lists of desired sizes have free blocks: not enough memory
        errno = ENOMEM;
        return NULL;
    }
    MEM_BLK *parent_node = NULL;
    if (mpid == MPID_IRAM1)
    {
        while (mem_pool_1_heads[l] == NULL)
        {
            l--;
            if (l < 0)
            {
                break;
            }
        }
        if (l < 0)
        { // no free lists of desired sizes have free blocks: not enough memory
            errno = ENOMEM;
            return NULL;
        }
        // breaks from while loop due to free block found on level l. This free block will be split later.
        parent_node = (MEM_BLK *)mem_pool_1_heads[l];
        mem_pool_1_heads[l] = parent_node->next;
        if (parent_node->next)
            parent_node->next->prev = parent_node->prev;
    }
    else
    {
        while (mem_pool_2_heads[l] == NULL)
        {
            l--;
            if (l < 0)
            {
                break;
            }
        }
        if (l < 0)
        { // no free lists of desired sizes have free blocks: not enough memory
            errno = ENOMEM;
            return NULL;
        }
        parent_node = (MEM_BLK *)mem_pool_2_heads[l];
        mem_pool_2_heads[l] = parent_node->next;
        if (parent_node->next)
            parent_node->next->prev = parent_node->prev;
    }
    // Now we could split from level l to level `level`, where block size = 2 ^ k.
    return topDownTraverseFreeLists(mpid, parent_node, l, level);
}

// mark as free in bit array
void markMemFree(mpool_t mpid, int level, int level_index)
{
    if (mpid == MPID_IRAM1)
    {
        ClearBit(memory_bit_array.IRAM1_MEM_POOL, computeBitArrayIndex(level, (U16)level_index));
    }
    else
    {
        ClearBit(memory_bit_array.IRAM2_MEM_POOL, computeBitArrayIndex(level, (U16)level_index));
    }
}

// returns true if memory is allocated (as marked in bit array)
BOOL checkMemAllocated(mpool_t mpid, int level, int level_index)
{
    if (mpid == MPID_IRAM1)
    {
        return (TestBit(memory_bit_array.IRAM1_MEM_POOL, computeBitArrayIndex(level, (U16)level_index)) != 0);
    }
    else
    {
        return (TestBit(memory_bit_array.IRAM2_MEM_POOL, computeBitArrayIndex(level, (U16)level_index)) != 0);
    }
}

int computeBuddyLevelIndex(int level_index)
{
    return level_index % 2 ? level_index - 1 : level_index + 1;
}

int k_mpool_dealloc(mpool_t mpid, void *ptr)
{
#ifdef DEBUG_0
    printf("k_mpool_dealloc: mpid = %d, ptr = 0x%x\r\n", mpid, ptr);
#endif /* DEBUG_0 */
    if (ptr == NULL)
        return RTX_OK;
    if (mpid != MPID_IRAM1 && mpid != MPID_IRAM2)
    { // invalid mpid
        errno = EINVAL;
        return RTX_ERR;
    }
    if ((mpid == MPID_IRAM1 && ((U32)ptr < RAM1_START || (U32)ptr > RAM1_END)) || (mpid == MPID_IRAM2 && ((U32)ptr < RAM2_START || (U32)ptr > RAM2_END)))
    { // ptr out of bound
        errno = EFAULT;
        return RTX_ERR;
    }

    // TODO: how to detect all possible errors suggested in the lab manual?
    char *address = (char *)ptr; // cast to char for pointer arithmetic

    if (mpid == MPID_IRAM1)
    {
        int k = 7;
        int x = (int)(address - RAM1_START) / 32;
        while (k >= 0)
        {
            if (checkMemAllocated(mpid, k, x) == TRUE)
                break;
            k--;
            x /= 2;
        }
        if (k < 0)
        { // previously freed memory
            errno = EFAULT;
            return RTX_ERR;
        }
        markMemFree(mpid, k, x);

        if (k > 0 && checkMemAllocated(mpid, k, computeBuddyLevelIndex(x)) == FALSE)
        { // can coalesce
            // fix root level issue
            while (k > 0 && checkMemAllocated(mpid, k, computeBuddyLevelIndex(x)) == FALSE)
            { // buddy is free/unallocated
                // remove buddy off free list
                MEM_BLK *temp_blk = mem_pool_1_heads[k];
                // Probably should remove this. Level k's free list supposedly should not be null, given a buddy is found.
                int buddyAddress_copy = RAM1_START + computeBuddyLevelIndex(x) * pow2(12 - k);
                if ((/* mem_pool_1_heads[k] == NULL || */ computeBuddyLevelIndex(x) == 0) || (buddyAddress_copy == (int)(mem_pool_1_heads[k])))
                { // buddy is head of free list
                    mem_pool_1_heads[k] = temp_blk->next;
                    if (temp_blk->next)
                        temp_blk->next->prev = temp_blk->prev;
                    // if (temp_blk->prev)
                    //     temp_blk->prev->next = temp_blk->next;
                }
                else
                {
                    int buddyAddress = RAM1_START + computeBuddyLevelIndex(x) * pow2(12 - k);
                    while ((int)temp_blk != buddyAddress)
                    {
                        temp_blk = temp_blk->next;
                    }
                    /* Possible modification below.
                    MEM_BLK* prev_blk = temp_blk->prev;
                    temp_blk->next = prev_blk->next;
                    temp_blk->prev = prev_blk;
                    if (prev_blk->next) prev_blk->next->prev = temp_blk;
                    prev_blk->next = temp_blk;
                    */
                    temp_blk->prev->next = temp_blk->next;
                    if (temp_blk->next)
                        temp_blk->next->prev = temp_blk->prev;
                }
                markMemFree(mpid, k, x);
                // switch to parent
                address = computeBuddyLevelIndex(x) < x ? address - pow2(12 - k) : address;
                k--;
                x /= 2;
            }
            markMemFree(mpid, k, x);
            // cannot coalesce anymore
        }
        // cannot coalesce; just put deallocated block on free list
        MEM_BLK *free_blk = (MEM_BLK *)address;
        free_blk->level = k;
        free_blk->size = pow2(12 - k);
        if (mem_pool_1_heads[k] == NULL)
        {
            free_blk->prev = NULL;
            free_blk->next = NULL;
            mem_pool_1_heads[k] = free_blk;
            return RTX_OK;
        }
        else
        {
            MEM_BLK *temp_blk = mem_pool_1_heads[k];
            if ((char *)temp_blk > address)
            {
                free_blk->prev = NULL;
                free_blk->next = temp_blk;
                temp_blk->prev = free_blk;
                mem_pool_1_heads[k] = free_blk;
            }
            else
            {
                while (temp_blk && (char *)(temp_blk->next) < address)
                {
                    if (temp_blk->next)
                    {
                        temp_blk = temp_blk->next;
                    }
                    else
                    {
                        break;
                    }
                }
                free_blk->next = temp_blk->next;
                free_blk->prev = temp_blk;
                temp_blk->next = free_blk;
                // notice that temp_blk->next is already set to free_blk, which is NEVER == NULL
                if (free_blk->next)
                    free_blk->next->prev = free_blk;
            }
        }
        // return RTX_OK;
    }
    else
    {
        int k = 10;
        int x = (int)(address - RAM2_START) / 32;
        while (k >= 0)
        {
            if (checkMemAllocated(mpid, k, x) == TRUE)
                break;
            k--;
            x /= 2;
        }
        if (k < 0)
        { // previously freed memory
            errno = EFAULT;
            return RTX_ERR;
        }
        markMemFree(mpid, k, x);

        if (k > 0 && checkMemAllocated(mpid, k, computeBuddyLevelIndex(x)) == FALSE)
        { // can coalesce
            // fix root level issue.
            while (k > 0 && checkMemAllocated(mpid, k, computeBuddyLevelIndex(x)) == FALSE)
            { // buddy is free/unallocated
                // remove buddy off free list
                MEM_BLK *temp_blk = mem_pool_2_heads[k];
                int buddyAddress_copy = RAM2_START + computeBuddyLevelIndex(x) * pow2(15 - k);
                if ((/* mem_pool_1_heads[k] == NULL || */ computeBuddyLevelIndex(x) == 0) || (buddyAddress_copy == (int)(mem_pool_2_heads[k])))
                { // buddy is head of free list
                    mem_pool_2_heads[k] = temp_blk->next;
                    if (temp_blk->next)
                        temp_blk->next->prev = temp_blk->prev;
                    // if (temp_blk->prev)
                    //     temp_blk->prev->next = temp_blk->next;
                }
                else
                {
                    int buddyAddress = RAM2_START + computeBuddyLevelIndex(x) * pow2(15 - k);
                    while ((int)temp_blk != buddyAddress)
                    {
                        temp_blk = temp_blk->next;
                    }
                    // MEM_BLK *prev_blk = temp_blk->prev;
                    // temp_blk->next = prev_blk->next;
                    // temp_blk->prev = prev_blk;
                    // if (prev_blk->next)
                    //     prev_blk->next->prev = temp_blk;
                    // prev_blk->next = temp_blk;
                    temp_blk->prev->next = temp_blk->next;
                    if (temp_blk->next)
                        temp_blk->next->prev = temp_blk->prev;
                }
                markMemFree(mpid, k, x);
                // switch to parent
                address = computeBuddyLevelIndex(x) < x ? address - pow2(15 - k) : address;
                k--;
                x /= 2;
            }
            markMemFree(mpid, k, x);
            // cannot coalesce anymore
        }
        // cannot coalesce; just put deallocated block on free list
        MEM_BLK *free_blk = (MEM_BLK *)address;
        free_blk->level = k;
        free_blk->size = pow2(15 - k);
        if (mem_pool_2_heads[k] == NULL)
        {
            free_blk->prev = NULL;
            free_blk->next = NULL;
            mem_pool_2_heads[k] = free_blk;
            return RTX_OK;
        }
        else
        {
            MEM_BLK *temp_blk = mem_pool_2_heads[k];
            if ((char *)temp_blk > address)
            {
                free_blk->prev = NULL;
                free_blk->next = temp_blk;
                temp_blk->prev = free_blk;
                mem_pool_2_heads[k] = free_blk;
            }
            else
            {
                while (temp_blk && (char *)(temp_blk->next) < address)
                {
                    if (temp_blk->next)
                    {
                        temp_blk = temp_blk->next;
                    }
                    else
                    {
                        break;
                    }
                }
                free_blk->next = temp_blk->next;
                free_blk->prev = temp_blk;
                temp_blk->next = free_blk;
                // notice that temp_blk->next is already set to free_blk, which is NEVER == NULL
                if (free_blk->next)
                    free_blk->next->prev = free_blk;
            }
        }
        // return RTX_OK;
    }

    // Remove this?
    // return RTX_ERR;
    return RTX_OK;
}

/*func deprecated
// append n to string starting at string
void intToString(char *string, int *index, int n)
{
    while (n > 0)
    {
        *(string + index) = n % 10 + '0';
        *index++;
        n /= 10;
    }
}*/

// // print information about blocks on a free list
int printFreeList(MEM_BLK *head)
{
    MEM_BLK *temp = head;
    int local_free_blocks_count = 0;
    while (temp)
    {
        local_free_blocks_count++;
        /*char print_msg[64]; // assuming maximum print message length is 64
        int *msg_index = 0;
        intToString(print_msg, msg_index, (char *)temp);
        print_msg[msg_index++] = ':';
        print_msg[msg_index++] = ' ';
        intToString(print_msg, msg_index, temp->size);
        print_msg[msg_index++] = '\0';
        printf(print_msg);*/
        printf("0x%x: 0x%x\r\n", (U32)temp, (U32)temp->size);
        temp = temp->next;
    }
    return local_free_blocks_count;
}

// from small -> big blocks, print, for each block: "address: size"
int k_mpool_dump(mpool_t mpid)
{
#ifdef DEBUG_0
    printf("k_mpool_dump: mpid = %d\r\n", mpid);
#endif /* DEBUG_0 */
    if (mpid != MPID_IRAM1 && mpid != MPID_IRAM2)
    {
        return 0;
    }

    int free_blocks_count = 0;
    // print from bottom to top level (ordered in increasing block size)
    if (mpid == MPID_IRAM1)
    {
        for (int l = 7; l >= 0; l--)
        {
            if (mem_pool_1_heads[l] == NULL)
                continue;
            MEM_BLK *temp = mem_pool_1_heads[l];
            free_blocks_count += printFreeList(temp);
        }
    }
    else
    {
        for (int l = 10; l >= 0; l--)
        {
            if (mem_pool_2_heads[l] == NULL)
                continue;
            MEM_BLK *temp = mem_pool_2_heads[l];
            free_blocks_count += printFreeList(temp);
        }
    }

    // print summary message
    /*char print_msg[64]; // assuming maximum print message length is 64
    int *msg_index = 0;
    intToString(print_msg, msg_index, free_blocks_count);
    for (int i = 0; i < 28; i++)
    {
        print_msg[msg_index++] = SUMMARY_MSG[i];
    }*/
    printf("%d free memory block(s) found\r\n", free_blocks_count);

    return free_blocks_count;
    // return 0;
}

int k_mem_init(int algo)
{
#ifdef DEBUG_0
    printf("k_mem_init: algo = %d\r\n", algo);
#endif /* DEBUG_0 */

    if (k_mpool_create(algo, RAM1_START, RAM1_END) < 0)
    {
        return RTX_ERR;
    }

    if (k_mpool_create(algo, RAM2_START, RAM2_END) < 0)
    {
        return RTX_ERR;
    }

    return RTX_OK;
}

/**
 * @brief allocate kernel stack statically
 */
U32 *k_alloc_k_stack(task_t tid)
{

    if (tid >= MAX_TASKS)
    {
        errno = EAGAIN;
        return NULL;
    }
    U32 *sp = g_k_stacks[tid + 1];

    // 8B stack alignment adjustment
    if ((U32)sp & 0x04)
    {         // if sp not 8B aligned, then it must be 4B aligned
        sp--; // adjust it to 8B aligned
    }
    return sp;
}

/**
 * @brief allocate user/process stack statically
 * @attention  you should not use this function in your lab
 */

U32 *k_alloc_p_stack(task_t tid)
{
    if (tid >= NUM_TASKS)
    {
        errno = EAGAIN;
        return NULL;
    }

    U32 *sp = g_p_stacks[tid + 1];

    // 8B stack alignment adjustment
    if ((U32)sp & 0x04)
    {         // if sp not 8B aligned, then it must be 4B aligned
        sp--; // adjust it to 8B aligned
    }
    return sp;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
