#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

#define NUM_IMGS 16
#define WIDTH 256
#define HEIGHT 256
#define EXPOSURE 20000
#define FRAME_TIME 50000

extern int open_cam(Fg_Struct **gr, int mode, int num_images, int w, int h);
extern int close_cam(Fg_Struct *fg);
extern int method1(Fg_Struct *fg, int num_imgs, int w, int h, int e, int f);
extern int method2(Fg_Struct *fg, int num_imgs, int w, int h, int e, int f);

int main()
{
	Fg_Struct *fg;

	if(open_cam(&fg, GRABBER_CONTROLLED, NUM_IMGS, WIDTH, HEIGHT) == FG_OK) {
		method1(fg, NUM_IMGS, WIDTH, HEIGHT, EXPOSURE, FRAME_TIME);
		close_cam(fg);
	}

	if(open_cam(&fg, ASYNC_SOFTWARE_TRIGGER, NUM_IMGS, WIDTH, HEIGHT) == FG_OK) {
		method2(fg, NUM_IMGS, WIDTH, HEIGHT, EXPOSURE, FRAME_TIME);
		close_cam(fg);
	}

	return FG_OK;
}