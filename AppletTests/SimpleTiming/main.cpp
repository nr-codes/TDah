#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

#define NUM_IMGS 3


#define WIDTH 524
#define HEIGHT 1
#define EXPOSURE 600

#define MIN_EXP 1
#define MAX_EXP 1

#define MIN_WIDTH 16
#define MAX_WIDTH 16
#define MIN_HEIGHT 1
#define MAX_HEIGHT 1

#define MIN_LENGTH 128
#define MAX_LENGTH 128


extern int open_cam(Fg_Struct **gr, int mode, int num_images, int w, int h);
extern int close_cam(Fg_Struct *fg);
extern int method1(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f);
extern int method2(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f);
extern int method3(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f);

int main(int argc, char *argv[])
{
	Fg_Struct *fg;
	int w,h;
	double e;

	for(e = MIN_EXP; e <= MAX_EXP; e *= 10) {
		for(w = MIN_WIDTH; w <= MAX_WIDTH; w += 4) {
			for(h = MIN_HEIGHT; h <= MAX_HEIGHT; h++) {
				if(open_cam(&fg, ASYNC_SOFTWARE_TRIGGER, NUM_IMGS, w, h) == FG_OK) {
					method2(fg, NUM_IMGS, w, h, e, e);
					close_cam(fg);
				}
			}
		}
	}

	/*
				if(open_cam(&fg, GRABBER_CONTROLLED, NUM_IMGS, w, h) == FG_OK) {
					method1(fg, NUM_IMGS, w, h, EXPOSURE, EXPOSURE);
					close_cam(fg);
				}
	*/

	/*
	if(open_cam(&fg, GRABBER_CONTROLLED, NUM_IMGS, WIDTH, HEIGHT) == FG_OK) {
		method3(fg, NUM_IMGS, WIDTH, HEIGHT, EXPOSURE, FRAME_TIME);
		close_cam(fg);
	}
	*/

	return FG_OK;
}