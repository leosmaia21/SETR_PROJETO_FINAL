/** @file main.c
 * @brief Main.c is the main file of the Assignment 5 - Implementing a closed loop control application in Zephyr
 *
 * 
 * @author Ângelo Mostardinha, Ruben Costa, Leonardo Maia
 * @date 24th June 2022
 * @bug No known bugs.
 */

/* Includes */
#include <device.h>
#include <devicetree.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <hal/nrf_saadc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/__assert.h>
#include <sys/printk.h>
#include <timing/timing.h>
#include <zephyr.h>

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

#define SAMPLES 4

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1
#define thread_D_prio 1
#define thread_E_prio 2

#define BOARDBUT1 11 /* Pin at which BUT1 is connected. Addressing is direct (i.e., pin number) */
#define BOARDBUT2 12 /* Pin at which BUT2 is connected. Addressing is direct (i.e., pin number) */
#define BOARDBUT3 24 /* Pin at which BUT3 is connected. Addressing is direct (i.e., pin number) */
#define BOARDBUT4 25 /* Pin at which BUT4 is connected. Addressing is direct (i.e., pin number) */

#define SAMP_PERIOD_MS 25
/* Therad periodicity (in ms)*/
#define thread_A_period 1000

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_D_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_E_stack, STACK_SIZE);

/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;
struct k_thread thread_D_data;
struct k_thread thread_E_data;

/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;
k_tid_t thread_D_tid;
k_tid_t thread_E_tid;

/* Global vars (shared memory between tasks A/B and B/C, resp) */
int ab = 100;
int bc = 200;

/* Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;
struct k_sem sem_cd;
struct k_sem sem_de;

/* Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);
void thread_D_code(void *argA, void *argB, void *argC);
void thread_E_code(void *argA, void *argB, void *argC);

/*ADC config*/
#define ADC_NID DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1

/* This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx. In fact a channel can */
/*    be assigned to two ANx, when differential reading is set (one ANx for the positive signal and the other one for the negative signal) */
/* Note also that the configuration of differnt channels is completely independent (gain, resolution, ref voltage, ...) */
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1

#define BUFFER_SIZE 1

/* Other defines */
// define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {.gain = ADC_GAIN, .reference = ADC_REFERENCE, .acquisition_time = ADC_ACQUISITION_TIME, .channel_id = ADC_CHANNEL_ID, .input_positive = ADC_CHANNEL_INPUT};

/* Global vars */
struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];

/* Takes one sample */
static int adc_sample(void) {
    int ret;
    const struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_dev == NULL) {
        printk("adc_sample(): error, must bind to adc first \n\r");
        return -1;
    }

    ret = adc_read(adc_dev, &sequence);
    if (ret) {
        printk("adc_read() failed with code %d\n", ret);
    }

    return ret;
}

#define GPIO0_NID DT_NODELABEL(gpio0)
#define PWM0_NID DT_NODELABEL(pwm0)
#define BOARDLED_PIN 0x04
//#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)

int err = 0;

static int array[SAMPLES];
int finalMean = 0;

const struct device *gpio0_dev; /* Pointer to GPIO device structure */
const struct device *pwm0_dev;  /* Pointer to PWM device structure */
static struct gpio_callback but_cb_data;
int ret = 0; /* Generic return value variable */

unsigned int pwmPeriod_us = 100; /* PWM priod in us */

volatile int flag = 0;
typedef enum { manual, automatic } m;

m mode = manual;

/* Main function */
/**
 *
 * Main is where all the configurations are defined,the threads are created and the semaphores are initialized
 *
 */
void config(void);
void main(void) {
    /* Welcome message */
    printf("\n\r Illustration of the use of shmem + semaphores\n\r");
    config();
    memset(array, 0, SAMPLES);
    /* Create and init semaphores */
    k_sem_init(&sem_ab, 0, 1);
    k_sem_init(&sem_bc, 0, 1);
    k_sem_init(&sem_cd, 0, 1);
    k_sem_init(&sem_de, 0, 1);

    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
    if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    }
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }

    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 °C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 50;
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Error: Failed to bind to GPIO0\n\r");
        return;
    } else {
        printk("Bind to GPIO0 successfull \n\r");
    }

    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
        printk("Error: Failed to bind to PWM0\n r");
        return;
    } else {
        printk("Bind to PWM0 successful\n\r");
    }
    /* Create tasks */
    k_msleep(1000);
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack, K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code, NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack, K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code, NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_C_tid = k_thread_create(&thread_C_data, thread_C_stack, K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code, NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);

    thread_D_tid = k_thread_create(&thread_D_data, thread_D_stack, K_THREAD_STACK_SIZEOF(thread_D_stack), thread_D_code, NULL, NULL, NULL, thread_D_prio, 0, K_NO_WAIT);

    thread_E_tid = k_thread_create(&thread_E_data, thread_E_stack, K_THREAD_STACK_SIZEOF(thread_E_stack), thread_E_code, NULL, NULL, NULL, thread_E_prio, 0, K_NO_WAIT);
    // while (1) {
    // }
    return;
}

/* Thread A code implementation */
/**
 *
 * thread_A_code is the function in which we get the samples and save them in a array. The ADC values were mapped from 0 to 100.
 */
void thread_A_code(void *argA, void *argB, void *argC) {
    /* Timing variables to control task periodicity */
    int64_t fin_time = 0, release_time = 0;

    /* Other variables */

    printk("Thread A init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + SAMP_PERIOD_MS;
    int x = 0;
    /* Thread loop */
    while (1) {
        // while (x < 10) {
        /*    Do the workload */
        err = adc_sample();
        if (err) {
            printk("adc_sample() failed with error code %d\n\r", err);
        } else {
            if (adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
            } else {
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                // printk("adc reading: raw:%4u / %4u mV: \n\r", adc_sample_buffer[0], (uint16_t)(1000 * adc_sample_buffer[0] * ((float)3 / 1023)));
                for (int i = SAMPLES - 1; i > 0; i--) {
                    array[i] = array[i - 1];
                }
                array[0] = (adc_sample_buffer[0] * 100 / 1023);
                // printk("Valor inicial:%d\r\n", (int)array[0]);
            }
        }
        // k_msleep(3);
        x++;
        // }
        x = 0;

        k_sem_give(&sem_ab);

        /* Wait for next release instant */
        fin_time = k_uptime_get();
        if (fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += SAMP_PERIOD_MS;
        }
    }
}

/* Thread B code implementation */
/**
 *
 * thread_B_code is the function in which we get the mean value of the values from thread A, using the last four samples.
 *
 */
void thread_B_code(void *argA, void *argB, void *argC) {
    int i = 0;
    int sum = 0;
    int mean = 0;
    int newArray[SAMPLES];
    int ii = 0;
    /* Other variables */
    printk("Thread B init (sporadic, waits on a semaphore by task A)\n");
    while (1) {
        k_sem_take(&sem_ab, K_FOREVER);
        for (i = 0; i < SAMPLES; i++) {
            sum += array[i];
            // sum = 1;
        }
        mean = sum / SAMPLES;
        sum = 0;
        int up = mean * 1.1;
        int down = mean * 0.9;
        for (i = 0; i < SAMPLES; i++) {
            if (up > array[i] && array[i] > down) {
                newArray[ii] = array[i];
                ii++;
            }
        }
        if (ii != 0) {
            for (i = 0; i < ii; i++) {
                sum += newArray[i];
            }
            finalMean = sum / ii;
        }
          finalMean = mean;
        sum = 0;
        ii = 0;
        //  }else{
        //    finalMean=0;
        //}
        // ii = 0;
        // printk(" final mean: %d, \n\r", array[0]);
        k_sem_give(&sem_bc);
    }
}

/* Thread C code implementation */
/**
 *
 * thread_C_code is the is the PI controller, adjusting the value that is received from the transistor. Because of 
 * the pull up resistance the value that is read from the ADC, becomes the symmetric value of the 
 * light intensity. It also applies the PWM signal to one of the DevKit leds, the duty cycle is the control signal.
 */

int ret1 = 0;
const float TI = 0.05;
const float K = 0.05;
const float h = SAMP_PERIOD_MS / 1000.0;
float s0 = K * (1 + (h / TI));
float s1 = -K;
float u = 0;
float error = 0;
float prevError = 0;
float prevU = 0;
float ref = 40;
float feedBack = 0;
int count = 0;
int duty = 0;
int display = 0;
void thread_C_code(void *argA, void *argB, void *argC) {
    /* Other variables */
    printk("Thread C init (sporadic, waits on a semaphore by task A)\n");
    while (1) {
        k_sem_take(&sem_bc, K_FOREVER);
        feedBack = 100 - finalMean;
        error = ref - feedBack;
        //  if (error < 0) error = 0;
        //  if (error > 10000) error = 10000;
        u = (s0 * error) + (s1 * prevError) + prevU;
        // u = error * K;
        if (u <= 0.0) {
            u = 0.0;
        }
        if (u >= 100.0) {
            u = 100.0;
        }
        prevU = u;
        prevError = error;

        // integral += error * TI;   // no entanto o PI dos mataroanos funciona
        // if(integral>100)integral=100;
        // if(integral<-0)integral=-0;
        //  u=integral+(error*K);
        
        //  if (duty > 100) duty = 100;
        ret1 = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN, pwmPeriod_us, (unsigned int)((pwmPeriod_us * u) / 100), PWM_POLARITY_NORMAL);
        if (ret1) {
            printk("Erroooor %d: failed to set pulse width\n", ret1);
        }

        k_sem_give(&sem_cd);
    }
}

/* Thread D code implementation */
/**
 *
 * thread_D_code is the function that allows to show a message to the user, showing if it is in automatic or manual mode, the light intensity wanted, instantaneous light intensity, the error and the control signal.
 */
void thread_D_code(void *argA, void *argB, void *argC) {
    while (1) {
        k_sem_take(&sem_cd, K_FOREVER);
        //  printk("ola\r\n");
        display++;
        if (display % 10 == 0) {
            display++;
            if (display % 10 == 0) printk("ref:%d,error:%d,u:%d,feedback:%d \r\n", (int)ref, (int)error, (int)u, (int)feedBack);
            printk("\e[1;1H\e[2J");
            printk("*-----* Projeto SETR *-----*\r\n");
            if (mode == automatic) {
                printk("Modo automático\r\n");

            } else {
                printk("Modo manual\r\n");
            }
            printk("ref:%d,error:%d,u:%d,feedback:%d \r\n", (int)ref, (int)error, (int)u, (int)feedBack);
        }
        //k_sem_give(&sem_de);
    }
}

/* Thread E code implementation */
/**
 *
 * thread_E_code (Not using semaphores) is always reading the buttons, allowing the user to choose from automatic or manual mode. 
 *
 */
void thread_E_code(void *argA, void *argB, void *argC) {
    while (1) {
       // k_sem_take(&sem_de, K_FOREVER);
        if (flag) {
            k_msleep(10);
            if (!gpio_pin_get(gpio0_dev, BOARDBUT1)) mode = manual;

            if ((!gpio_pin_get(gpio0_dev, BOARDBUT2)) && mode == manual) {
                ref++;
                if (ref == 101) ref = 0;
            }
            if ((!gpio_pin_get(gpio0_dev, BOARDBUT3))) mode = automatic;
            if ((!gpio_pin_get(gpio0_dev, BOARDBUT4)) && mode == manual) {
                ref--;
                if (ref == -1) ref = 0;
            }
            flag = 0;
        }
    }
}

void butpress_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { flag = 1; }

void config() {
    int ret = 0; /* Generic return value variable */

    printk("LED1 blink demo \n");

    /* Bind to GPIO 0 */
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Failed to bind to GPIO0\n\r");
        return;
    } else {
        printk("Bind to GPIO0 successfull \n\r");
    }

    /* Configure PIN */

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT1, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 1 \n\r", ret);
        return;
    }

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT2, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 2 \n\r", ret);
        return;
    }

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT3, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 3 \n\r", ret);
        return;
    }

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT4, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 4 \n\r", ret);
        return;
    }

    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT1, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT1 pin \n\r", ret);
        return;
    }

    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT2, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT2 pin \n\r", ret);
        return;
    }

    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT3, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT3 pin \n\r", ret);
        return;
    }

    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT4, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT4 pin \n\r", ret);
        return;
    }

    gpio_init_callback(&but_cb_data, butpress_cbfunction, BIT(BOARDBUT1) | BIT(BOARDBUT2) | BIT(BOARDBUT3) | BIT(BOARDBUT4));
    gpio_add_callback(gpio0_dev, &but_cb_data);
}