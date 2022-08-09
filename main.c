#include <stdio.h>
#include "soniclib.h"
#include "chirp_bsp.h"

ch_group_t chirp_group;

int main(void)
{
    ch_group_t *grp_ptr = &chirp_group;

	printf("\nTDK InvenSense CH-201 STR Example\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);
	printf("\n");

	chbsp_board_init(grp_ptr);

    return 0;
}
