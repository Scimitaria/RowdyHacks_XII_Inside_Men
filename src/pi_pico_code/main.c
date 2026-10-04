#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// TB6612FNG wiring
#define PIN_PWMA  0   // PWM slice 0, channel A
#define PIN_PWMB  1   // PWM slice 0, channel B
#define PIN_AIN2  2
#define PIN_AIN1  3
#define PIN_BIN1  4
#define PIN_BIN2  5
#define PIN_STBY  6

// Quadrature wheel encoders
#define PIN_ENC_A_A  10   // motor A, channel A
#define PIN_ENC_A_B  11   // motor A, channel B
#define PIN_ENC_B_A  12   // motor B, channel A
#define PIN_ENC_B_B  13   // motor B, channel B

#define PWM_FREQ_HZ        20000   // above audible range, well under TB6612's 100 kHz max
#define TURN_DUTY_PERCENT  50      // used by left/right when no duty is given

// Motor B is mounted mirrored, so its IN1/IN2 levels are swapped to make
// DIR_CW spin it CW from the robot's point of view.
#define MOTOR_B_REVERSED 1

// The mirrored mounting also flips motor B's encoder, so its count is negated
// to make both counts go up when the robot drives forward. Flip these if a
// wheel counts down while driving forward.
#define ENC_A_REVERSED 0
#define ENC_B_REVERSED 1

// Serial commands from the Pi, one per line ("<cmd> [duty]", duty is 0-100):
//   1 <duty>   forward  (both CW)
//   2 <duty>   backward (both CCW)
//   3 [duty]   left     (spin in place: A CCW, B CW)
//   4 [duty]   right    (spin in place: A CW, B CCW)
//   5          stop     (IN1 = IN2 = L on both motors)
//   6          encoders (replies "ENC <count_a> <count_b> <time_us>")
//   7 <l> <r>  per wheel (signed duty -100-100 for A/left and B/right,
//              positive is forward, negative is backward)
enum {
    CMD_FORWARD  = 1,
    CMD_BACKWARD = 2,
    CMD_LEFT     = 3,
    CMD_RIGHT    = 4,
    CMD_STOP     = 5,
    CMD_ENCODERS = 6,
    CMD_WHEELS   = 7,
};

typedef struct {
    uint pin_pwm;
    uint pin_in1;
    uint pin_in2;
    bool reversed;
} motor_t;

// IN1 = H, IN2 = L is CW; IN1 = L, IN2 = H is CCW (before any reversal).
typedef enum { DIR_CW, DIR_CCW } motor_dir_t;

// Motor A is the left wheel, motor B the right wheel.
static const motor_t motor_a = { PIN_PWMA, PIN_AIN1, PIN_AIN2, false };
static const motor_t motor_b = { PIN_PWMB, PIN_BIN1, PIN_BIN2, MOTOR_B_REVERSED };

static uint16_t pwm_wrap;

static void out_pin_init(uint pin, bool value) {
    gpio_init(pin);
    gpio_put(pin, value);   // latch the level before enabling the output driver
    gpio_set_dir(pin, GPIO_OUT);
}

// Both motor PWM pins (GPIO0/1) share slice 0, so one slice config covers both.
static void pwm_setup(void) {
    gpio_set_function(PIN_PWMA, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWMB, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_PWMA);

    // f_pwm = clk_sys / (div * (wrap + 1)); with div = 1 at 125 MHz, wrap = 6249
    pwm_wrap = (uint16_t)(clock_get_hz(clk_sys) / PWM_FREQ_HZ - 1);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, pwm_wrap);
    pwm_init(slice, &cfg, false);

    pwm_set_gpio_level(PIN_PWMA, 0);
    pwm_set_gpio_level(PIN_PWMB, 0);
    pwm_set_enabled(slice, true);
}

static void motor_init(const motor_t *m) {
    out_pin_init(m->pin_in1, 0);
    out_pin_init(m->pin_in2, 0);
}

static void motor_set(const motor_t *m, motor_dir_t dir, uint percent) {
    bool cw = (dir == DIR_CW) != m->reversed;
    gpio_put(m->pin_in1, cw);
    gpio_put(m->pin_in2, !cw);
    if (percent > 100) percent = 100;
    pwm_set_gpio_level(m->pin_pwm, (uint32_t)(pwm_wrap + 1) * percent / 100);
}

// IN1 = IN2 = L: the TB6612 turns the outputs off and the motor coasts.
static void motor_stop(const motor_t *m) {
    gpio_put(m->pin_in1, 0);
    gpio_put(m->pin_in2, 0);
    pwm_set_gpio_level(m->pin_pwm, 0);
}

static void stop_all(void) {
    motor_stop(&motor_a);
    motor_stop(&motor_b);
}

static void drive(motor_dir_t a, motor_dir_t b, uint percent) {
    motor_set(&motor_a, a, percent);
    motor_set(&motor_b, b, percent);
}

// Signed duty: positive drives the wheel forward (CW), negative backward, 0 coasts.
static void wheel_set(const motor_t *m, int duty) {
    if (duty == 0) motor_stop(m);
    else motor_set(m, duty > 0 ? DIR_CW : DIR_CCW, (uint)(duty > 0 ? duty : -duty));
}

typedef struct {
    uint pin_a;
    uint pin_b;
    uint8_t state;           // last (A << 1) | B
    volatile int32_t count;  // written only by the GPIO interrupt
} encoder_t;

static encoder_t enc_a = { PIN_ENC_A_A, PIN_ENC_A_B, 0, 0 };
static encoder_t enc_b = { PIN_ENC_B_A, PIN_ENC_B_B, 0, 0 };

// Indexed by (prev_state << 2) | new_state. Forward is A leading B
// (00 -> 10 -> 11 -> 01); invalid jumps (both pins changed) count as 0.
static const int8_t quad_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static inline uint8_t encoder_read(const encoder_t *e, uint32_t pins) {
    return (uint8_t)((((pins >> e->pin_a) & 1u) << 1) | ((pins >> e->pin_b) & 1u));
}

static void encoder_update(encoder_t *e, uint32_t pins) {
    uint8_t s = encoder_read(e, pins);
    e->count += quad_table[(e->state << 2) | s];
    e->state = s;
}

// Fires on every edge of all four encoder pins (4x decoding).
static void encoder_irq(uint gpio, uint32_t events) {
    (void)events;
    uint32_t pins = gpio_get_all();
    if (gpio == enc_a.pin_a || gpio == enc_a.pin_b) encoder_update(&enc_a, pins);
    else if (gpio == enc_b.pin_a || gpio == enc_b.pin_b) encoder_update(&enc_b, pins);
}

static void encoder_init(encoder_t *e) {
    const uint pins[2] = { e->pin_a, e->pin_b };
    for (int i = 0; i < 2; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);   // most motor encoders are open-collector
    }
    e->state = encoder_read(e, gpio_get_all());

    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(e->pin_a, edges, true, &encoder_irq);
    gpio_set_irq_enabled(e->pin_b, edges, true);
}

static void print_encoders(void) {
    // Aligned 32-bit reads are atomic on the M0+, so no need to mask the IRQ.
    int32_t a = enc_a.count;
    int32_t b = enc_b.count;
    if (ENC_A_REVERSED) a = -a;
    if (ENC_B_REVERSED) b = -b;
    printf("ENC %ld %ld %llu\n", (long)a, (long)b, (unsigned long long)time_us_64());
}

// Returns false (and prints why) if the line isn't a valid command.
static bool handle_command(const char *line) {
    int cmd, duty, duty_b;
    int n = sscanf(line, "%d %d %d", &cmd, &duty, &duty_b);
    if (n < 1) {
        printf("ERR expected '<cmd> [duty]'\n");
        return false;
    }

    if (cmd == CMD_WHEELS) {
        if (n < 3 || duty < -100 || duty > 100 || duty_b < -100 || duty_b > 100) {
            printf("ERR command %d needs '<left> <right>' duty cycles (-100-100)\n", cmd);
            return false;
        }
        wheel_set(&motor_a, duty);
        wheel_set(&motor_b, duty_b);
        return true;
    }

    bool needs_duty = (cmd == CMD_FORWARD || cmd == CMD_BACKWARD);
    if (needs_duty && n < 2) {
        printf("ERR command %d needs a duty cycle (0-100)\n", cmd);
        return false;
    }
    if (n < 2) duty = TURN_DUTY_PERCENT;
    if (duty < 0 || duty > 100) {
        printf("ERR duty %d out of range (0-100)\n", duty);
        return false;
    }

    switch (cmd) {
    case CMD_FORWARD:  drive(DIR_CW,  DIR_CW,  (uint)duty); break;
    case CMD_BACKWARD: drive(DIR_CCW, DIR_CCW, (uint)duty); break;
    case CMD_LEFT:     drive(DIR_CCW, DIR_CW,  (uint)duty); break;
    case CMD_RIGHT:    drive(DIR_CW,  DIR_CCW, (uint)duty); break;
    case CMD_STOP:     stop_all(); break;
    case CMD_ENCODERS: print_encoders(); break;
    default:
        printf("ERR unknown command %d (use 1-6)\n", cmd);
        return false;
    }
    return true;
}

int main(void) {
    stdio_init_all();

    // Keep the driver in standby while everything is configured.
    out_pin_init(PIN_STBY, 0);
    motor_init(&motor_a);
    motor_init(&motor_b);
    pwm_setup();
    stop_all();
    encoder_init(&enc_a);
    encoder_init(&enc_b);

    // Everything is configured: bring the driver out of standby.
    gpio_put(PIN_STBY, 1);

    char line[32];
    size_t len = 0;
    bool was_connected = false;

    while (true) {
        // Safety: stop the motors if the Pi closes the port or the cable is unplugged.
        bool connected = stdio_usb_connected();
        if (was_connected && !connected) {
            stop_all();
            len = 0;
        }
        was_connected = connected;

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
