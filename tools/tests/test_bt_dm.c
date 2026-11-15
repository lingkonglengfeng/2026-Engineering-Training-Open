/* Host regression for the real byte-stream parser, with a simulated HAL clock. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bt_button.h"

static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }

static void check(const char *command, const char *expected, int newline)
{
    bt_button_event_t event;
    const char *p;
    BT_Button_Init();
    tick = 0;
    for (p = command; *p; ++p)
    {
        BT_Button_ProcessByte((uint8_t)*p);
        tick += 20; /* Bluetooth packet gap: longer than the old 15 ms timeout. */
        BT_Button_Poll();
        assert(!BT_Button_Pop(&event));
    }
    if (newline)
        BT_Button_ProcessByte('\n');
    else
    {
        tick += 301;
        BT_Button_Poll();
    }
    assert(BT_Button_Pop(&event));
    assert(event.id == BT_BUTTON_CUSTOM);
    assert(strcmp(event.text, expected) == 0);
    assert(!BT_Button_Pop(&event));
}

int main(void)
{
    int newline;
    for (newline = 0; newline <= 1; ++newline)
    {
        check("&dm resend 0", "dm resend 0", newline);
        check("&dm txlog 1", "dm txlog 1", newline);
        check("&dm pos 0.2 0.5", "dm pos 0.2 0.5", newline);
        check("&dm mit 0 0 100 1 0", "dm mit 0 0 100 1 0", newline);
    }
    puts("PASS: 8 Bluetooth DM byte-stream cases (newline / idle timeout)");
    return 0;
}
