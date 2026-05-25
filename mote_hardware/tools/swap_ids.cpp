#include "SMS_STS.h"
#include <cstdio>
#include <unistd.h>

static void set_id(SMS_STS &sms, u8 current_id, u8 new_id)
{
    // Unlock EEPROM, write new ID, then lock using the new ID.
    // The servo responds to new_id immediately after the write, so
    // LockEeprom must use new_id — otherwise the lock packet is ignored.
    sms.unLockEeprom(current_id);
    usleep(10000);

    u8 val = new_id;
    int ret = sms.genWrite(current_id, SMS_STS_ID, &val, 1);
    usleep(10000);

    if (ret) {
        sms.LockEeprom(new_id);
        usleep(10000);
        printf("Servo %d -> ID %d: OK (EEPROM saved)\n", current_id, new_id);
    } else {
        printf("Servo %d -> ID %d: FAILED (no response)\n", current_id, new_id);
    }
}

int main()
{
    SMS_STS sms;
    if (!sms.begin(1000000, "/dev/mote_servos")) {
        printf("Failed to open /dev/mote_servos\n");
        return 1;
    }

    // Three-step swap of IDs 7 and 9, using ID 1 as temporary.
    printf("Step 1: servo 7 -> ID 1 (temp)\n");
    set_id(sms, 7, 1);

    printf("Step 2: servo 9 -> ID 7\n");
    set_id(sms, 9, 7);

    printf("Step 3: servo 1 -> ID 9\n");
    set_id(sms, 1, 9);

    sms.end();
    printf("Done. IDs swapped (7<->9). Verify with servo_debug.\n");
    return 0;
}
