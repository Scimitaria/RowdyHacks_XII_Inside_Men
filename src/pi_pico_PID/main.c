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

#define PWM_FREQ_HZ  20000   // above audible range, well under TB6612's 100 kHz max

// Motor B is mounted mirrored, so its IN1/IN2 levels are swapped to make
// DIR_CW spin it CW from the robot's point of view.
#define MOTOR_B_REVERSED 1

// The mirrored mounting also flips motor B's encoder, so its count is negated
// to make both counts go up when the robot drives forward. Flip these if a
// wheel counts down while driving forward.
#define ENC_A_REVERSED 0
#define ENC_B_REVERSED 1

// PI speed loop. Speeds are in encoder ticks per second (the same 4x-decoded
// ticks the "ENC" reply counts); the output is signed duty in percent.
#define CONTROL_PERIOD_US  20000    // 50 Hz
#define MAX_TARGET_TPS     3046    // reject setpoints beyond this (sanity bound)
#define KP                 0.0098f    // % duty per (tick/s) of error
#define KI                 0.0492f    // % duty per tick of accumulated error
// Low-pass on the measured speed: 1 = raw, smaller = smoother but laggier.
#define VEL_FILTER_ALPHA   0.5f

// Serial commands from the Pi, one per line ("<type> [left] [right]"):
//   1 <l> <r>    speed  (signed ticks/s for A/left and B/right, positive is
//                forward; 0 coasts that wheel)
//   5            stop   (both setpoints 0, IN1 = IN2 = L on both motors)
//   6            encoders (replies "ENC <count_a> <count_b> <time_us>")
enum {
    CMD_SPEED    = 1,
    CMD_STOP     = 5,
    CMD_ENCODERS = 6,
};

typedef struct {
    uint pin_pwm;
    uint pin_in1;
    uint pin_in2;
    bool reversed;
} motor_t;

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

// IN1 = IN2 = L: the TB6612 turns the outputs off and the motor coasts.
static void motor_stop(const motor_t *m) {
    gpio_put(m->pin_in1, 0);
    gpio_put(m->pin_in2, 0);
    pwm_set_gpio_level(m->pin_pwm, 0);
}

// Signed duty in percent: positive drives the wheel forward (CW), negative
// backward, 0 coasts. Uses the full PWM resolution, not whole percents.
static void motor_set_duty(const motor_t *m, float duty) {
    if (duty == 0.0f) {
        motor_stop(m);
        return;
    }
    bool forward = duty > 0.0f;
    float mag = forward ? duty : -duty;
    if (mag > 100.0f) mag = 100.0f;

    bool cw = forward != m->reversed;
    gpio_put(m->pin_in1, cw);
    gpio_put(m->pin_in2, !cw);
    pwm_set_gpio_level(m->pin_pwm, (uint16_t)((pwm_wrap + 1) * mag / 100.0f));
}

typedef struct {
    uint pin_a;
    uint pin_b;
    bool reversed;
    uint8_t state;           // last (A << 1) | B
    volatile int32_t count;  // written only by the GPIO interrupt
} encoder_t;

static encoder_t enc_a = { PIN_ENC_A_A, PIN_ENC_A_B, ENC_A_REVERSED, 0, 0 };
static encoder_t enc_b = { PIN_ENC_B_A, PIN_ENC_B_B, ENC_B_REVERSED, 0, 0 };

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

// Count with the robot's sign convention (forward is positive).
// Aligned 32-bit reads are atomic on the M0+, so no need to mask the IRQ.
static int32_t encoder_count(const encoder_t *e) {
    int32_t c = e->count;
    return e->reversed ? -c : c;
}

static void print_encoders(void) {
    printf("ENC %ld %ld %llu\n", (long)encoder_count(&enc_a), (long)encoder_count(&enc_b),
           (unsigned long long)time_us_64());
}

typedef struct {
    const motor_t *motor;
    const encoder_t *enc;
    int32_t target;       // setpoint, ticks/s
    int32_t last_count;   // encoder count at the previous control step
    float speed;          // filtered measured speed, ticks/s
    float integral;       // I term, already scaled by KI (% duty)
} wheel_t;

static wheel_t wheel_l = { &motor_a, &enc_a, 0, 0, 0.0f, 0.0f };
static wheel_t wheel_r = { &motor_b, &enc_b, 0, 0, 0.0f, 0.0f };

static void wheel_step(wheel_t *w, float dt) {
    int32_t count = encoder_count(w->enc);
    float raw = (float)(count - w->last_count) / dt;
    w->last_count = count;
    w->speed += VEL_FILTER_ALPHA * (raw - w->speed);

    // A zero setpoint coasts instead of actively holding the wheel still.
    if (w->target == 0) {
        w->integral = 0.0f;
        motor_stop(w->motor);
        return;
    }

    float err = (float)w->target - w->speed;
    float p = KP * err;
    float integral = w->integral + KI * err * dt;
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    // Anti-windup: while the output is saturated, only let the integral move
    // back towards the unsaturated range.
    float out = p + integral;
    bool winding_up = (out > 100.0f && err > 0.0f) || (out < -100.0f && err < 0.0f);
    if (!winding_up) w->integral = integral;

    out = p + w->integral;
    if (out > 100.0f) out = 100.0f;
    if (out < -100.0f) out = -100.0f;
    motor_set_duty(w->motor, out);
}

static void wheel_set_target(wheel_t *w, int32_t target) {
    // Reversing direction: start from a clean integral rather than unwinding
    // the old one through zero.
    if ((target > 0 && w->target < 0) || (target < 0 && w->target > 0)) w->integral = 0.0f;
    w->target = target;
}

static void stop_all(void) {
    wheel_l.target = wheel_r.target = 0;
    wheel_l.integral = wheel_r.integral = 0.0f;
    motor_stop(&motor_a);
    motor_stop(&motor_b);
}

// Returns false (and prints why) if the line isn't a valid command.
static bool handle_command(const char *line) {
    int cmd;
    if (sscanf(line, "%d", &cmd) < 1) {
        printf("ERR expected '<type> [left] [right]'\n");
        return false;
    }

    switch (cmd) {
    case CMD_SPEED: {
        long left, right;
        if (sscanf(line, "%*d %ld %ld", &left, &right) < 2 ||
            left < -MAX_TARGET_TPS || left > MAX_TARGET_TPS ||
            right < -MAX_TARGET_TPS || right > MAX_TARGET_TPS) {
            printf("ERR command %d needs '<left> <right>' ticks/s (-%d-%d)\n",
                   cmd, MAX_TARGET_TPS, MAX_TARGET_TPS);
            return false;
        }
        wheel_set_target(&wheel_l, (int32_t)left);
        wheel_set_target(&wheel_r, (int32_t)right);
        break;
    }
    case CMD_STOP:     stop_all(); break;
    case CMD_ENCODERS: print_encoders(); break;
    default:
        printf("ERR unknown command %d (use 1, 5, 6)\n", cmd);
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

    // The control loop runs here rather than in a timer IRQ so it never races
    // the command handler over the setpoints.
    uint64_t last_step = time_us_64();
    uint64_t next_step = last_step + CONTROL_PERIOD_US;

    while (true) {
        // Safety: stop the motors if the Pi closes the port or the cable is unplugged.
        bool connected = stdio_usb_connected();
        if (was_connected && !connected) {
            stop_all();
            len = 0;
        }
        was_connected = connected;

        uint64_t now = time_us_64();
        if (now >= next_step) {
            float dt = (float)(now - last_step) * 1e-6f;   // actual, in case a step ran late
            last_step = now;
            next_step += CONTROL_PERIOD_US;
            if (next_step <= now) next_step = now + CONTROL_PERIOD_US;   // don't burst to catch up
            wheel_step(&wheel_l, dt);
            wheel_step(&wheel_r, dt);
        }

        // Short timeout so the control loop keeps its rate while waiting for input.
        int c = getchar_timeout_us(1000);
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
