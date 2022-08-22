#include <stdio.h>
#include "soniclib.h"
#include "chirp_bsp.h"

#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprstr_init			/* standard STR firmware */
// #define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprstr_wd_init	/* watchdog-enabled STR firmware */
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

typedef struct {
    uint32_t        range;       // from ch_get_range()
    uint16_t        amplitude;   // from ch_get_amplitude()
    uint16_t        num_samples; // from ch_get_num_samples()
} chirp_data_t;

chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];
ch_dev_t     chirp_devices[CHIRP_MAX_NUM_SENSORS];
ch_group_t   chirp_group;

int main(void)
{
    ch_group_t *grp_ptr = &chirp_group;
    uint8_t    chirp_error = 0;
    uint8_t    num_ports;
    uint8_t    dev_num;

	chbsp_board_init(grp_ptr);

	printf("\nTDK InvenSense CH-201 STR Example\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);
	printf("\n");

    num_ports = ch_get_num_ports(grp_ptr);

    printf("Initializing sensor(s)... ");

    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);
        chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
    }


    return 0;
}
