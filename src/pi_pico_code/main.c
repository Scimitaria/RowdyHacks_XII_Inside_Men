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

#define PWM_FREQ_HZ   20000   // above audible range, well under TB6612's 100 kHz max
#define DUTY_PERCENT  50

typedef struct {
    uint pin_pwm;
    uint pin_in1;
    uint pin_in2;
} motor_t;

// IN1 = H, IN2 = L is CW; IN1 = L, IN2 = H is CCW.
typedef enum { DIR_CW, DIR_CCW } motor_dir_t;

static const motor_t motor_a = { PIN_PWMA, PIN_AIN1, PIN_AIN2 };
static const motor_t motor_b = { PIN_PWMB, PIN_BIN1, PIN_BIN2 };

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
    bool cw = (dir == DIR_CW);
    gpio_put(m->pin_in1, cw);
    gpio_put(m->pin_in2, !cw);
    if (percent > 100) percent = 100;
    pwm_set_gpio_level(m->pin_pwm, (uint32_t)(pwm_wrap + 1) * percent / 100);
}

int main(void) {
    stdio_init_all();

    // Keep the driver in standby while everything is configured.
    out_pin_init(PIN_STBY, 0);
    motor_init(&motor_a);
    motor_init(&motor_b);
    pwm_setup();

    motor_set(&motor_a, DIR_CW, DUTY_PERCENT);
    motor_set(&motor_b, DIR_CW, DUTY_PERCENT);

    // Everything is configured: bring the driver out of standby to start running.
    gpio_put(PIN_STBY, 1);

    // PWM hardware keeps generating the waveform; nothing left for the CPU to do.
    while (true) {
        tight_loop_contents();
    }
}
