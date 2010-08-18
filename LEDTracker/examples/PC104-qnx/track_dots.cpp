#include <conio.h>
#include "t_dah.h"
#include "TDahMe3Fc.h"
#include "ni_pci6713.h"

// ROI parameters
#define NUM_ROI 2
#define ROI_W 16
#define ROI_H 16

#define USE_KAL false
#define USE_TPT false

#define GR_CH 1
#define GR_DEPTH IPL_DEPTH_8U

#define NIMGS 800
ROILoc r[NIMGS];

int track_dots(TDahMe3Fc *capture, char *conf, char *intrins, char *extrins)
{
	int i, j, imgs_lost, prev_img;

	if(open_analog() != CV_OK) return !CV_OK;

	// explicitly give ROI info and save config for later use
	i = capture->initROIs(NUM_ROI, ROI_W, ROI_H, conf, 
		USE_KAL, USE_TPT, intrins, extrins);
	if(i != CV_OK) return i;

	imgs_lost = 0;
	prev_img = 0;
	i = 0;
	j = 0;
	while(1) {
		if(_kbhit()) break;

		j = i % NIMGS; // get ROI index
		++i; // get next image

		// get and process next image, assuming NUM_ROI >= 2
		prev_img = capture->getROILoc(i, &r[j]);
		if(prev_img != i) imgs_lost++;

		if(!r[j].obj_found) {
			printf("object lost (%d): (%d) %d (%0.4g, %0.4g)\n", 
				r[j].obj_found, r[j].img_nr, r[j].roi_nr, 
				r[j].loc.x, r[j].loc.y);

			printf("images lost: %d\n", imgs_lost);

			// output data
			if(output_analog(r[j].loc.x, r[j].loc.y, ROI_LOST) != CV_OK) {
				printf("problem writing to analog card\n");
			}

			// output data ready signal
			capture->toggle_dio0();

			printf("press any key to exit\n");
			_getch();
			break;
		}

		// output data
		if(output_analog(r[j].loc.x, r[j].loc.y, r[j].roi_nr) != CV_OK) {
			printf("problem writing to analog card\n");
			break;
		}

		// output data ready signal
		capture->toggle_dio0();
	}

	return close_analog();
}
