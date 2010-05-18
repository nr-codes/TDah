#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

#define NUM_IMGS 9
#define WIDTH 1024
#define HEIGHT 1024
#define EXPOSURE 10
#define MIN_LENGTH 20
#define MAX_LENGTH 40


extern int open_cam(Fg_Struct **gr, int mode, int num_images, int w, int h);
extern int close_cam(Fg_Struct *fg);
extern int method1(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f);
extern int method2(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f);
extern int method3(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f);

int main(int argc, char *argv[])
{
	Fg_Struct *fg;
	int w,h;

	/*
	w = atoi(argv[1]);
	h = atoi(argv[2]);
	*/

	for(w = MIN_LENGTH; w <= MAX_LENGTH; w += 20) {
		for(h = MIN_LENGTH; h <= MAX_LENGTH; h += 20) {

			if(open_cam(&fg, GRABBER_CONTROLLED, NUM_IMGS, w, h) == FG_OK) {
				method1(fg, NUM_IMGS, w, h, EXPOSURE, EXPOSURE);
				close_cam(fg);
			}


			if(open_cam(&fg, ASYNC_SOFTWARE_TRIGGER, NUM_IMGS, w, h) == FG_OK) {
				method2(fg, NUM_IMGS, w, h, EXPOSURE, EXPOSURE);
				close_cam(fg);
			}

		}
	}

	/*
	if(open_cam(&fg, GRABBER_CONTROLLED, NUM_IMGS, WIDTH, HEIGHT) == FG_OK) {
		method3(fg, NUM_IMGS, WIDTH, HEIGHT, EXPOSURE, FRAME_TIME);
		close_cam(fg);
	}
	*/

	return FG_OK;
}