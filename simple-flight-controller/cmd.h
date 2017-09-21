//----------------------------------------------------------------------------
// Quad copter command control definition.
//----------------------------------------------------------------------------
#define CMD_MAX_LEN 			   30	
#define ENTER_KEY_CODE             13
extern char cmdBuffer[CMD_MAX_LEN];

extern unsigned char cmdLen;
extern unsigned char last_cmd_len;
extern char last_cmd_buff[CMD_MAX_LEN];


void process_cmd(char* cmd_str, unsigned char cmd_length);
void show_info();
void clear_stats();
