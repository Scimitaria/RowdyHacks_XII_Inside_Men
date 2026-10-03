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

#define PWM_FREQ_HZ        20000   // above audible range, well under TB6612's 100 kHz max
#define TURN_DUTY_PERCENT  50      // used by left/right when no duty is given

// Motor B is mounted mirrored, so its IN1/IN2 levels are swapped to make
// DIR_CW spin it CW from the robot's point of view.
#define MOTOR_B_REVERSED 1

// Serial commands from the Pi, one per line ("<cmd> [duty]", duty is 0-100):
//   1 <duty>   forward  (both CW)
//   2 <duty>   backward (both CCW)
//   3 [duty]   left     (spin in place: A CCW, B CW)
//   4 [duty]   right    (spin in place: A CW, B CCW)
//   5          stop     (IN1 = IN2 = L on both motors)
enum {
    CMD_FORWARD  = 1,
    CMD_BACKWARD = 2,
    CMD_LEFT     = 3,
    CMD_RIGHT    = 4,
    CMD_STOP     = 5,
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

// Returns false (and prints why) if the line isn't a valid command.
static bool handle_command(const char *line) {
    int cmd, duty;
    int n = sscanf(line, "%d %d", &cmd, &duty);
    if (n < 1) {
        printf("ERR expected '<cmd> [duty]'\n");
        return false;
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
    default:
        printf("ERR unknown command %d (use 1-5)\n", cmd);
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
