#include <stdio.h>
#include "pico/stdlib.h"

// Slotted optical sensors (one output per wheel, e.g. LM393 speed modules)
#define PIN_SLOT_A  10   // motor A (left wheel)
#define PIN_SLOT_B  12   // motor B (right wheel)

// Cheap slot sensors chatter on their edges; a rising edge closer than this to
// the previous one is ignored. Must stay below the slot period at top speed
// (e.g. 20 slots at 300 rpm = 10 ms per slot).
#define DEBOUNCE_US  1000

// Serial commands from the Pi, one per line:
//   6   counts (replies "ENC <count_a> <count_b> <time_us>")
// The reply matches the motor Pico's encoder reply, so the same odom node can
// read either. Slot sensors can't tell direction, so counts only go up.
enum {
    CMD_COUNTS = 6,
};

typedef struct {
    uint pin;
    uint32_t last_edge_us;
    volatile uint32_t count;   // written only by the GPIO interrupt
} slot_counter_t;

static slot_counter_t slot_a = { PIN_SLOT_A, 0, 0 };
static slot_counter_t slot_b = { PIN_SLOT_B, 0, 0 };

static void slot_update(slot_counter_t *s) {
    uint32_t now = time_us_32();
    if (now - s->last_edge_us < DEBOUNCE_US) return;
    s->last_edge_us = now;
    s->count++;
}

// One count per slot: fires on the rising edge as the beam is unblocked.
static void slot_irq(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == slot_a.pin) slot_update(&slot_a);
    else if (gpio == slot_b.pin) slot_update(&slot_b);
}

static void slot_init(slot_counter_t *s) {
    gpio_init(s->pin);
    gpio_set_dir(s->pin, GPIO_IN);
    gpio_pull_up(s->pin);
    gpio_set_irq_enabled_with_callback(s->pin, GPIO_IRQ_EDGE_RISE, true, &slot_irq);
}

static void print_counts(void) {
    // Aligned 32-bit reads are atomic on the M0+, so no need to mask the IRQ.
    uint32_t a = slot_a.count;
    uint32_t b = slot_b.count;
    printf("ENC %lu %lu %llu\n", (unsigned long)a, (unsigned long)b,
           (unsigned long long)time_us_64());
}

// Returns false (and prints why) if the line isn't a valid command.
static bool handle_command(const char *line) {
    int cmd;
    if (sscanf(line, "%d", &cmd) < 1) {
        printf("ERR expected '<cmd>'\n");
        return false;
    }

    switch (cmd) {
    case CMD_COUNTS: print_counts(); break;
    default:
        printf("ERR unknown command %d (use 6)\n", cmd);
        return false;
    }
    return true;
}

int main(void) {
    stdio_init_all();

    slot_init(&slot_a);
    slot_init(&slot_b);

    char line[32];
    size_t len = 0;

    while (true) {
        int c = getchar_timeout_us(10000);
        if (c == PICO_ERROR_TIMEOUT) continue;

        if (c == '\n' || c == '\r') {
            if (len == 0) continue;   // ignore blank lines / the \n after \r
            line[len] = '\0';
            len = 0;
            if (handle_command(line)) printf("OK %s\n", line);
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
        }
        // Characters past the buffer size are dropped; the line then fails to parse.
    }
}
