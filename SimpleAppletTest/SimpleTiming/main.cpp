#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

extern int open_cam(Fg_Struct **gr, int mode);
extern int close_cam(Fg_Struct *fg);
extern int method1(Fg_Struct *fg, int num_imgs, int w, int h, int e, int f);

int main()
{
	Fg_Struct *fg;

	if(open_cam(&fg, GRABBER_CONTROLLED) == FG_OK) {
		method1(fg, 16, 256, 256, 20000, 50000);
		close_cam(fg);
	}

	return FG_OK;
}