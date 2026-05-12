#include "SMS_STS.h"
#include <cstdio>
#include <unistd.h>

static void set_id(SMS_STS &sms, u8 current_id, u8 new_id)
{
    u8 val = new_id;
    int ret = sms.genWrite(current_id, SMS_STS_ID, &val, 1);
    if (ret) {
        printf("Servo %d -> ID %d: OK\n", current_id, new_id);
    } else {
        printf("Servo %d -> ID %d: FAILED (no response)\n", current_id, new_id);
    }
    usleep(100000);
}

int main()
{
    SMS_STS sms;
    if (!sms.begin(1000000, "/dev/auldbot_servos")) {
        printf("Failed to open /dev/auldbot_servos\n");
        return 1;
    }

    printf("Step 1: servo 7 -> ID 1 (temp)\n");
    set_id(sms, 7, 1);

    printf("Step 2: servo 9 -> ID 7\n");
    set_id(sms, 9, 7);

    printf("Step 3: servo 1 -> ID 9\n");
    set_id(sms, 1, 9);

    sms.end();
    printf("Done. Left=9, Right=7\n");
    return 0;
}
