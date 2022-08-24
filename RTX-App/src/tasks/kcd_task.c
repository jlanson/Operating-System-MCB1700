/** 
 * @brief The KCD Task Template File 
 * @note  The file name and the function name can be changed
 * @see   k_tasks.h
 */

#include "rtx.h"
#include "k_task.h"
#include "k_msg.h"

#define NOT_REG 0

char complete_cmd[KCD_CMD_BUF_SIZE] = {0};
char cmd_body[KCD_CMD_BUF_SIZE-2] = {0};
int string_index = 0;
#define MAX_CMDS  128   // maximum number of registered commands
task_t registered_cmds[MAX_CMDS] = {0};
// char registered_cmds[MAX_CMDS][KCD_CMD_BUF_SIZE];
// int cmd_count = 0;

BOOL str_cmp(char* a, char* b, size_t len) {
    for (int i = 0; i < len; i++) {
        if (*(a+i) != *(b+i)) return FALSE;
    }
    return TRUE;
}

int str_len(char* str) {
    int len = 0;
    while (*(str+len)) len++;
    return len;
}

void task_kcd(void)
{
    mbx_create(KCD_MBX_SIZE);

    char msg_buf[KCD_MBX_SIZE];
    RTX_MSG_HDR* msg;
    // BOOL send_invalid_command_msg = FALSE;
    RTX_MSG_HDR* msg_to_send;
    char send_msg_data[KCD_CMD_BUF_SIZE];

    while (TRUE) {
        recv_msg(msg_buf, KCD_MBX_SIZE);
        msg = (RTX_MSG_HDR*)msg_buf;

        switch (msg->type)
        {
        case KEY_IN:
        {
            // queue the input key
            char key_in = msg_buf[MSG_HDR_SIZE];

            if (string_index < KCD_CMD_BUF_SIZE) {
                complete_cmd[string_index++] = key_in;
            }
                
            // forward the msg to console display task
            msg->type = DISPLAY;
            msg->sender_tid = TID_KCD;
            send_msg(TID_CON, (const void *)msg);
            
            // check for 'enter' key
            if (key_in == '\r') {
                if (string_index == 4 && str_cmp(complete_cmd, "%LT", 3) == TRUE) { 
                    // print task id and state of all non-DORMANT tasks on console
                    for (int i = 0; i < MAX_TASKS; i++)
                    {
                        // Question: Do we need to print out uninitialized tasks?
                        if (g_tcbs[i].state != DORMANT && g_tcbs[i].state != UNINITIALIZED) // Check for non-dormant
                        {
                            sprintf(&send_msg_data[MSG_HDR_SIZE], "Task %d is now in state %d\r\n\r\n", i, g_tcbs[i].state);
                            msg_to_send = (RTX_MSG_HDR*) send_msg_data;
                            msg_to_send->length = MSG_HDR_SIZE + str_len(&send_msg_data[MSG_HDR_SIZE]);
                            msg_to_send->sender_tid = TID_KCD;
                            msg_to_send->type = DISPLAY;
                            send_msg(TID_CON, msg_to_send);
                        }    
                    }
                } else if (string_index == 4 && str_cmp(complete_cmd, "%LM", 3) == TRUE) {
                    // print task id and state of all non-DORMANT tasks with a mailbox, along with how much free space in bytes in the mailbox
                    task_t taskIds[MAX_TASKS];
                    int activeMailboxCount = mbx_ls(taskIds, MAX_TASKS);
                    msg_to_send = (RTX_MSG_HDR*)send_msg_data;
                    msg_to_send->sender_tid = TID_KCD;
                    msg_to_send->type = DISPLAY;
                    
                    // send DISPLAY msg for each active task
                    for (int task = 0; task < activeMailboxCount; task++) {
                        // copy task id and state to send message to console display
                        sprintf(&send_msg_data[MSG_HDR_SIZE], 
                            "Task ID: %d, state: %d, mailbox free space (bytes): %d\r\n\r\n", 
                            taskIds[task], 
                            g_tcbs[taskIds[task]].state,
                            ring_buf_remaining(&(mailboxes[taskIds[task]].ring_buf))
                        );
                        msg_to_send->length = MSG_HDR_SIZE + str_len(&send_msg_data[MSG_HDR_SIZE]);
                        send_msg(TID_CON, msg_to_send);
                    }
                } else if (complete_cmd[0] == '%' && (registered_cmds[complete_cmd[1]] == NOT_REG || g_tcbs[registered_cmds[complete_cmd[1]]].state == DORMANT)) {
                    msg_to_send = (RTX_MSG_HDR*) send_msg_data;
                    msg_to_send->length = MSG_HDR_SIZE + 20;
                    msg_to_send->sender_tid = TID_KCD;
                    msg_to_send->type = DISPLAY;
                    sprintf(&send_msg_data[MSG_HDR_SIZE], "Command not found\r\n");
                    send_msg(TID_CON, msg_to_send);                    
                } else if (complete_cmd[0] == '%' && registered_cmds[complete_cmd[1]] != NOT_REG && string_index < KCD_CMD_BUF_SIZE) {
                    // first, we need to find what to send from the complete command.
                    // E.g., "%test", only need to send "test"
                    for (int i = 0; i < string_index - 2; i++)
                    {
                        cmd_body[i] = complete_cmd[i+1];
                    }
                    // Then, send the msg.
                    msg_to_send = (RTX_MSG_HDR*) send_msg_data;
                    msg_to_send->length = MSG_HDR_SIZE + string_index - 2; // we need to remove % and enter.
                    msg_to_send->sender_tid = TID_KCD;
                    msg_to_send->type = KCD_CMD;
                    sprintf(&send_msg_data[MSG_HDR_SIZE], cmd_body);
                    send_msg(registered_cmds[complete_cmd[1]], msg_to_send);
                } else if (complete_cmd[0] != '%' || string_index >= KCD_CMD_BUF_SIZE) {
                    msg_to_send = (RTX_MSG_HDR*) send_msg_data;
                    msg_to_send->length = MSG_HDR_SIZE + 18;
                    msg_to_send->sender_tid = TID_KCD;
                    msg_to_send->type = DISPLAY;
                    sprintf(&send_msg_data[MSG_HDR_SIZE], "Invalid command\r\n");
                    send_msg(TID_CON, msg_to_send);
                }
                
                // after receiving enter key, clear input queue and send buffer regardless
                string_index = 0;
                for (int i = 0; i < KCD_CMD_BUF_SIZE; i++) {
                    complete_cmd[i] = NOT_REG;
                    send_msg_data[i] = 0;
                    cmd_body[i] = 0;
                }
            }

            break;
        }
        case KCD_REG: // register a command
        {
            char cmd = msg_buf[MSG_HDR_SIZE]; // E.g. 'W'
            if (cmd != 'L') { // register a command will work for ALL but L, which is saved for KCD only.
                registered_cmds[cmd] = msg->sender_tid; // store sender at its registered command's index.
            }
            break;
        }
        default:
            break;
        }
    }
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */

