#include <errno.h>

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

#define TIMEOUT 2

extern int update_roi(Fg_Struct *fg, int w, int h, double e, double f);
extern int get_images(Fg_Struct *fg, int num_imgs);
extern int show_images(Fg_Struct *fg, int *nr, int num_images, int w, int h);

int overhead(int num_imgs)
{
	// in an average sense
	//int i = 0, img_nr = 0;
	//unsigned int nr[16];
	//unsigned __int64 loop_start, loop_stop;

	//QueryPerformanceCounter(&loop_start);
	//while(i < num_imgs) {
	//	QueryPerformanceCounter(pc_start + i);
	//	nr[i] = img_nr;
	//	i++;
	//	QueryPerformanceCounter(pc_stop + i);
	//}
	//QueryPerformanceCounter(&loop_stop));

	return FG_OK;
}

int method1(Fg_Struct *fg, int num_imgs, int w, int h, int e, int f)
{
	int i = 0, img_nr = 0;
	int rc;
	int *nr;
	unsigned char *img = NULL;
	unsigned __int64 freq, loop_start, loop_stop;
	unsigned __int64 *pc_ts;
	unsigned __int64 *fg_ts;

	nr = (int *) calloc(num_imgs, sizeof(int));
	if(nr == NULL) {
		return -ENOMEM;
	}

	pc_ts = (unsigned __int64 *) calloc(num_imgs, sizeof(unsigned __int64));
	if(pc_ts == NULL) {
		return -ENOMEM;
	}

	fg_ts = (unsigned __int64 *) calloc(num_imgs, sizeof(unsigned __int64));
	if(fg_ts == NULL) {
		return -ENOMEM;
	}

	update_roi(fg, w, h, e, f);
	get_images(fg, num_imgs);
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_start);
	while(i < num_imgs) {
		QueryPerformanceCounter((LARGE_INTEGER *) (pc_ts + i));
		img_nr = Fg_getLastPicNumberBlocking(fg, i + 1, PORT_A, TIMEOUT);
		if(img_nr > FG_OK) {
			img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
			update_roi(fg, w, h, e, f);
		}
		nr[i] = img_nr;
		i++;
	}
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_stop);

	rc = QueryPerformanceFrequency((LARGE_INTEGER *) &freq);
	printf("%lld %lld %lld\n", 
		((rc == TRUE) ? freq : -ENODEV), loop_start, loop_stop);
	for(i = 0; i < num_imgs; i++) {
		fg_ts[i] = nr[i];
		rc = Fg_getParameter(fg, FG_TIMESTAMP, fg_ts + i, PORT_A);
		printf("%d %lld %lld\n", nr[i], pc_ts[i], ((rc < 0) ? rc : fg_ts[i]));
	}

	show_images(fg, nr, num_imgs, w, h);

	free(nr);
	free(pc_ts);
	free(fg_ts);
	return FG_OK;
}

int method2(Fg_Struct *fg, int num_imgs, int w, int h, int e, int f)
{
	int i = 0, img_nr = 0, rc;
	int *nr;
	unsigned char *img = NULL;
	unsigned __int64 freq, loop_start, loop_stop;
	unsigned __int64 *pc_start;
	unsigned __int64 *pc_stop;
	unsigned __int64 *fg_ts;

	nr = (int *) calloc(num_imgs, sizeof(int));
	if(nr == NULL) {
		return -ENOMEM;
	}

	pc_start = (unsigned __int64 *) calloc(num_imgs, sizeof(unsigned __int64));
	if(pc_start == NULL) {
		return -ENOMEM;
	}

	pc_stop = (unsigned __int64 *) calloc(num_imgs, sizeof(unsigned __int64));
	if(pc_stop == NULL) {
		return -ENOMEM;
	}

	fg_ts = (unsigned __int64 *) calloc(num_imgs, sizeof(unsigned __int64));
	if(fg_ts == NULL) {
		return -ENOMEM;
	}

	update_roi(fg, w, h, e, f);
	get_images(fg, num_imgs);
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_start);
	while(i < num_imgs) {
		QueryPerformanceCounter((LARGE_INTEGER *) (pc_start + i));
		Fg_sendSoftwareTrigger(fg, PORT_A);
		img_nr = Fg_getLastPicNumberBlocking(fg, i + 1, PORT_A, TIMEOUT);
		if(img_nr > FG_OK) {
			img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
			update_roi(fg, w, h, e, f);
		}
		nr[i] = img_nr;
		QueryPerformanceCounter((LARGE_INTEGER *) (pc_stop + i));
		i++;
	}
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_stop);

	rc = QueryPerformanceFrequency((LARGE_INTEGER *) &freq);
	printf("%lld %lld %lld -1\n", 
		((rc == TRUE) ? freq : -ENODEV), loop_start, loop_stop);
	for(i = 0; i < num_imgs; i++) {
		fg_ts[i] = nr[i];
		rc = Fg_getParameter(fg, FG_TIMESTAMP, fg_ts + i, PORT_A);
		printf("%d %lld %lld %lld\n", 
			nr[i], pc_start[i], pc_stop[i], ((rc < 0) ? rc : fg_ts[i]));
	}

	show_images(fg, nr, num_imgs, w, h);

	free(nr);
	free(pc_start);
	free(pc_stop);
	free(fg_ts);
	return FG_OK;
}