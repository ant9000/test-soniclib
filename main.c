#include <stdio.h>
#include "soniclib.h"
#include "chirp_bsp.h"
#include "periph/i2c.h"

#include "MedianFilter.h"
#include "ultrasound_display_config_info.h"

#define CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprstr_init			/* standard STR firmware */
//#define	CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprstr_wd_init	/* watchdog-enabled STR firmware */
#define CHIRP_SENSOR_MODE		CH_MODE_FREERUN
#define CHIRP_SENSOR_TARGET_INT		CH_TGT_INT_FILTER_ANY
#define CHIRP_SENSOR_TARGET_INT_HIST	5		// num of previous results kept in history
#define CHIRP_SENSOR_TARGET_INT_THRESH  3		// num of target detections req'd to interrupt
#define CHIRP_SENSOR_TARGET_INT_RESET   0		// if non-zero, target filter resets after interrupt
#define	CHIRP_SENSOR_MAX_RANGE_MM		4000	/* maximum range, in mm */
#define	CHIRP_SENSOR_THRESHOLD_0		0	/* close range threshold (0 = use default) */
#define	CHIRP_SENSOR_THRESHOLD_1		0	/* standard threshold (0 = use default) */
#define	CHIRP_SENSOR_RX_HOLDOFF			0	/* # of samples to ignore at start of meas */
#define	CHIRP_SENSOR_RX_LOW_GAIN		0	/* # of samples (0 = use default) */
#define	CHIRP_SENSOR_TX_LENGTH			0	/* Tx pulse length, in cycles (0 = use default) */
#define	CHIRP_SENSOR_STR_RANGE		0		/* STR range, in samples (0 = entire meas range) */
#define	MEASUREMENT_INTERVAL_MS		100		// 100ms interval = 10Hz sampling
#define IQ_DATA_MAX_NUM_SAMPLES  CH201_GPRSTR_MAX_SAMPLES	// use max for ch201_gprstr
#define SHOW_PRESENCE_INDICATORS			/* COMMENT OUT THIS LINE TO DISABLE "Present" INDICATORS */
#define PRESENCE_HOLD_SECONDS      	1		/* time until end of presence is reported, in seconds */
#define NUM_RANGE_HISTORY_VALUES    7 		/* number of range history values used in filtering */ 
// #define OUTPUT_AMP_DATA_CSV	/* uncomment to output calculated amplitudes in CSV */
// #define OUTPUT_IQ_DATA_CSV	/* uncomment to output I/Q data in CSV format */
#define READ_IQ_BLOCKING 		/* use blocking mode when reading I/Q */
// #define READ_IQ_NONBLOCKING 	/* use non-blocking mode when reading I/Q */

/* Bit flags used in main loop to check for completion of I/O or timer operations.  */
#define DATA_READY_FLAG     (1 << 0)        // data ready from sensor
#define IQ_READY_FLAG       (1 << 1)        // non-blocking I/Q read has completed
#define TIMER_FLAG          (1 << 2)        // period timer has interrupted
/* Number of measurement cycles to hold presence count, equals (hold seconds * sampling rate) */
#define PRESENCE_HOLD_CYCLES    (PRESENCE_HOLD_SECONDS * 1000 / MEASUREMENT_INTERVAL_MS)
/* Enable I/Q readout if needed */
#if (defined(OUTPUT_AMP_DATA_CSV) || defined(OUTPUT_IQ_DATA_CSV))
#define READ_IQ_DATA                        // enable I/Q read
#undef  SHOW_PRESENCE_INDICATORS            // disable "Present" indicators if amp or I/Q output
#endif

typedef struct {
    uint32_t        range;       // from ch_get_range()
    uint16_t        amplitude;   // from ch_get_amplitude()
    uint16_t        num_samples; // from ch_get_num_samples()
#ifdef READ_IQ_DATA
    ch_iq_sample_t  iq_data[IQ_DATA_MAX_NUM_SAMPLES]; // from ch_get_iq_data()
#endif
} chirp_data_t;

typedef struct {
    uint8_t     presence_detection;
    uint32_t    presence_range;
} presence_output_t;

typedef struct {
    sMedianFilter_t medianFilter;
    sMedianNode_t medianBuffer[NUM_RANGE_HISTORY_VALUES];
} presence_utils_t;

chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];
ch_dev_t     chirp_devices[CHIRP_MAX_NUM_SENSORS];
ch_group_t   chirp_group;

volatile uint32_t taskflags = 0;
static uint32_t active_devices;
static uint32_t data_ready_devices;
static uint8_t num_connected_sensors = 0;
presence_output_t   presence_output;
presence_utils_t    presence_utils;
static int presence_hold_count = 0;
static uint8_t  num_triggered_devices = 0;
#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
static uint8_t  num_io_queued = 0;
#endif

void periodic_timer_callback(void) {
    /* Set timer flag - it will be checked and cleared in main() loop */
    taskflags |= TIMER_FLAG;
    if (num_triggered_devices > 0) {
        ch_group_trigger(&chirp_group);
    }
}

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t __attribute__((unused)) int_type)
{
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
    data_ready_devices |= (1 << dev_num);       // add to data-ready bit mask
    if (data_ready_devices == active_devices) {
        /* All active sensors have interrupted after performing a measurement */
        data_ready_devices = 0;
        /* Set data-ready flag - it will be checked and cleared in main() loop */
        taskflags |= DATA_READY_FLAG;
        /* Disable interrupt unless in free-running mode
         *   It will automatically be re-enabled by the next ch_group_trigger()
         */
        if (ch_get_mode(dev_ptr) == CH_MODE_FREERUN) {
            chdrv_int_set_dir_in(dev_ptr);              // set INT line as input
            chdrv_int_group_interrupt_enable(grp_ptr);
        } else {
            chdrv_int_group_interrupt_disable(grp_ptr);
        }
    }
}

static void io_complete_callback(ch_group_t __attribute__((unused)) *grp_ptr) {
    taskflags |= IQ_READY_FLAG;
}

static void presence_utils_init(presence_utils_t *util) {
    util->medianFilter.numNodes = NUM_RANGE_HISTORY_VALUES;
    util->medianFilter.medianBuffer = util->medianBuffer;
    MEDIANFILTER_Init(&util->medianFilter);
}

static uint32_t update_range(presence_utils_t *util, uint32_t range_in) {
    uint32_t range_out = (uint32_t) MEDIANFILTER_Insert(&(util->medianFilter), range_in);
    if (range_out == 0) {
        range_out = range_in;       // use input value if zero
    }
    return range_out;
}

static uint8_t handle_data_ready(ch_group_t *grp_ptr) {
    uint8_t     dev_num;
    uint8_t     ret_val = 0;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            chirp_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);
            if (chirp_data[dev_num].range != CH_NO_TARGET) {
                /* Because sensor is in target interrupt mode, it should only interrupt
                 * if a target was successfully detected, and this should always be true */
                 /* Get the new amplitude value - it's only updated if range
                  * was successfully measured.  */
                chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);
                uint32_t range = update_range(&presence_utils, chirp_data[dev_num].range);
                presence_output.presence_range = range;
                printf("Port %d: Range: %0.1f mm   Amplitude: %u  ",
                        dev_num,
                        (float) range/32.0f,
                        chirp_data[dev_num].amplitude);
                presence_output.presence_detection = 1;
#ifdef SHOW_PRESENCE_INDICATORS
                printf("\n");
                printf("Present ");
                for (uint8_t i = 0; i < (presence_hold_count + 1); i++) {
                    printf("=");
                }
#endif
            printf("\n");
            }
#ifdef READ_IQ_DATA
            /* Optionally read raw I/Q values for all samples */
            display_iq_data(dev_ptr);
#endif
        }
    }
    return ret_val;
}

int main(void)
{
    ch_group_t *grp_ptr = &chirp_group;
    uint8_t    chirp_error = 0;
    uint8_t    num_ports;
    uint8_t    dev_num;

    presence_utils_init(&presence_utils);
	chbsp_board_init(grp_ptr);

	printf("\nTDK InvenSense CH-201 STR Example\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);
	printf("\n");

    printf("Initializing sensor(s)...\n");
    num_ports = ch_get_num_ports(grp_ptr);
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);
        chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
    }

    if (chirp_error == 0) {
        printf("starting group...\n");
        chirp_error = ch_group_start(grp_ptr);
    }

    if (chirp_error == 0) {
        printf("OK\n");
    } else {
        printf("FAILED: %d\n", chirp_error);
    }
    printf("\n");

    printf("Sensor\tType \t  Freq   \t  B/W  \t RTC Cal\tFirmware\n");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            printf("%d\tCH%d\t%u Hz\t%u Hz\t%u@%ums\t%s\n", dev_num,
                                            ch_get_part_number(dev_ptr),
                                            (unsigned int) ch_get_frequency(dev_ptr),
                                            (unsigned int) ch_get_bandwidth(dev_ptr),
                                            ch_get_rtc_cal_result(dev_ptr),
                                            ch_get_rtc_cal_pulselength(dev_ptr),
                                            ch_get_fw_version_string(dev_ptr));
        }
    }
    printf("\n");

    ch_io_int_callback_set(grp_ptr, sensor_int_callback);
    ch_io_complete_callback_set(grp_ptr, io_complete_callback);

    printf ("Configuring sensor(s)...\n");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_config_t dev_config;
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num_connected_sensors++;            // count one more connected
            active_devices |= (1 << dev_num);   // add to active device bit mask
            dev_config.mode = CHIRP_SENSOR_MODE;
            if (dev_config.mode != CH_MODE_FREERUN) {   // unless free-running
                num_triggered_devices++;                // will be triggered
            }
            dev_config.tgt_int_filter = CHIRP_SENSOR_TARGET_INT;
            ch_set_target_int_counter(dev_ptr, CHIRP_SENSOR_TARGET_INT_HIST,
                                      CHIRP_SENSOR_TARGET_INT_THRESH, CHIRP_SENSOR_TARGET_INT_RESET);
            dev_config.max_range = CHIRP_SENSOR_MAX_RANGE_MM;
            if (CHIRP_SENSOR_STR_RANGE != 0) {
                dev_config.static_range = CHIRP_SENSOR_STR_RANGE;
            } else {
                dev_config.static_range = ch_mm_to_samples(dev_ptr, CHIRP_SENSOR_MAX_RANGE_MM);
            }
            /* If sensor will be free-running, set internal sample interval */
            if (dev_config.mode == CH_MODE_FREERUN) {
                dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
            } else {
                dev_config.sample_interval = 0;
            }
            dev_config.thresh_ptr = NULL;
            /* Apply sensor configuration */
            chirp_error = ch_set_config(dev_ptr, &dev_config);
            /* Apply detection threshold settings, if specified */
            if (!chirp_error && (CHIRP_SENSOR_THRESHOLD_0 != 0)) {
                chirp_error = ch_set_threshold(dev_ptr, 0, CHIRP_SENSOR_THRESHOLD_0);
            }
            if (!chirp_error && (CHIRP_SENSOR_THRESHOLD_1 != 0)) {
                chirp_error = ch_set_threshold(dev_ptr, 1, CHIRP_SENSOR_THRESHOLD_1);
            }
            /* Apply other sensor settings, if not using defaults */
            if (!chirp_error && (CHIRP_SENSOR_RX_HOLDOFF != 0)) {
                chirp_error = ch_set_rx_holdoff(dev_ptr, CHIRP_SENSOR_RX_HOLDOFF);
            }
            if (!chirp_error && (CHIRP_SENSOR_RX_LOW_GAIN != 0)) {
                chirp_error = ch_set_rx_low_gain(dev_ptr, CHIRP_SENSOR_RX_LOW_GAIN);
            }
            if (!chirp_error && (CHIRP_SENSOR_TX_LENGTH != 0)) {
                chirp_error = ch_set_tx_length(dev_ptr, CHIRP_SENSOR_TX_LENGTH);
            }
            /* Enable sensor interrupt if using free-running mode
             *   Note that interrupt is automatically enabled if using
             *   triggered modes.
             */
            if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN)) {
                chdrv_int_set_dir_in(dev_ptr);
                chdrv_int_interrupt_enable(dev_ptr);
            }
            /* Read back and display config settings */
            if (!chirp_error) {
                ultrasound_display_config_info(dev_ptr);
            } else {
                printf("Device %d: Error during ch_set_config()\n", dev_num);
            }
            /* Display detection thresholds */
            printf("\t\tthreshold_0 = %d\tthreshold_1 = %d\n", ch_get_threshold(dev_ptr, 0),
                                                 ch_get_threshold(dev_ptr, 1));
            /* Display other sensor settings */
            printf("\t\trx_holdoff = %d\t\trx_low_gain = %d\ttx_length = %d\n",
                                                ch_get_rx_holdoff(dev_ptr),
                                                ch_get_rx_low_gain(dev_ptr),
                                                ch_get_tx_length(dev_ptr));
            /* Display target interrupt filter setting */
            ch_tgt_int_filter_t filter = ch_get_target_interrupt(dev_ptr);
            printf("\t\tTarget interrupt filter: ");
            if (filter == CH_TGT_INT_FILTER_OFF) {
                printf("OFF");
            } else if (filter == CH_TGT_INT_FILTER_ANY) {
                printf("ANY");
            } else if (filter == CH_TGT_INT_FILTER_COUNTER) {
                printf("COUNTER");
            }
            if (filter == CH_TGT_INT_FILTER_COUNTER) {
                uint8_t meas_hist;
                uint8_t count_thresh;
                uint8_t reset;

                ch_get_target_int_counter(dev_ptr, &meas_hist, &count_thresh, &reset);
                printf("\n\t\t   meas_hist=%d  count_thresh=%d", meas_hist, count_thresh);
                printf("  reset=%s", reset ? "yes":"no");
            }
            printf("\n");
            /* Get number of active samples per measurement */
            chirp_data[dev_num].num_samples = ch_get_num_samples(dev_ptr);
            /* Turn on an LED to indicate device connected */
            if (!chirp_error) {
                chbsp_led_on(dev_num);
            }
        }
    }
    printf("\n");
    if (num_connected_sensors > 1) {
        printf(" ** Multiple sensors connected - this is a single sensor application! **\n\n");
    }
    /* Initialize the periodic timer.
     *   For sensors in triggered mode, the timer will be used to trigger the
     *   measurements.  This timer is also used to control the persistent output
     *   of a presence indication for a fixed "hold" time.
     *
     *   This function initializes a timer that will interrupt every time it
     *   expires, after the specified measurement interval.  The function also
     *   registers a callback function that will be called from the timer
     *   handler when the interrupt occurs.
     */
    printf("Initializing sample timer for %dms interval... ", MEASUREMENT_INTERVAL_MS);
    chbsp_periodic_timer_init(MEASUREMENT_INTERVAL_MS, periodic_timer_callback);
    chbsp_periodic_timer_irq_enable();
    chbsp_periodic_timer_start();
    printf("OK\n");
    printf("Starting measurements\n");
    while (1) {     /* LOOP FOREVER */
        if (taskflags==0) {
            chbsp_proc_sleep();         // put processor in low-power sleep mode
            /* We only continue here after an interrupt wakes the processor */
        }
        if (taskflags & DATA_READY_FLAG) {
            /* Sensor has interrupted - handle sensor data */
            taskflags &= ~DATA_READY_FLAG;      // clear flag
            presence_hold_count = PRESENCE_HOLD_CYCLES; // re-init counter
            handle_data_ready(grp_ptr);         // read and display measurement
        } else if (taskflags & TIMER_FLAG) {
            /* Periodic timer interrupt occurred */
            taskflags &= ~TIMER_FLAG;           // clear flag
            // Hold presence indication for specified number of cycles
            if (presence_hold_count > 0) {
#ifdef SHOW_PRESENCE_INDICATORS
                printf("Present ");
                for (uint8_t i = 0; i < presence_hold_count; i++) {
                    printf("=");
                }
                printf("\n");
#endif
                presence_hold_count--;
            } else {
                presence_output.presence_detection = 0;
            }
        }
#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
        /* Start any pending non-blocking I/Q reads */
        if (num_io_queued != 0) {
            ch_io_start_nb(grp_ptr);
            num_io_queued = 0;
        }
        /* Check for non-blocking I/Q read complete */
        if (taskflags & IQ_READY_FLAG) {
            /* All non-blocking I/Q readouts have completed */
            taskflags &= ~IQ_READY_FLAG;        // clear flag
            handle_iq_data_done(grp_ptr);       // display I/Q data
        }
#endif
    }   // end  while(1) main loop
    return 0;
}
