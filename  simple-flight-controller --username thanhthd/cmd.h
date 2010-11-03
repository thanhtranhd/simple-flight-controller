//----------------------------------------------------------------------------
// Quad copter command control definition.
//----------------------------------------------------------------------------
#define CMD_MAX_LEN 			   30	

extern char cmdBuffer[CMD_MAX_LEN];
extern unsigned char cmdLen;

void process_cmd(char* cmd_str, unsigned char cmd_length);
void show_info();
void clear_stats();
