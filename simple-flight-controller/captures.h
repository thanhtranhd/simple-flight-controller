// capture statemachine
#ifndef CAPTURE_INC_H
#define CAPTURE_INC_H

typedef enum
{
   CAP_INIT=0,
   CAP_LOW,
   CAP_HIGH
} CAPTURE_STATE;

// INIT ---> CAP_LOW ---(intr) ---> CAP_HIGH ---(intr)--->CAP_LOW


typedef struct capture
{
   UINT8    state;
   UINT16   tick1;
} CAPTURE_CCB;

void init_ccb(CAPTURE_CCB* ccb);

// this function is supposed to be call when the GPIO interrupt is triggered.
void capture_timing(UINT8 pin, CAPTURE_CCB* ccb, UINT16* time);
void capture_p2_timing(UINT8 pin, CAPTURE_CCB* ccb, UINT16* time);

#endif
