#include <errno.h>
#include <math.h>

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

#define TIMEOUT 2
#define GRAB_TWO 2
#define FRAME_TIME_MAX 1000000
#define FT_ERROR 5.0
#define FT_DELTA .1
#define DO_INIT 1

extern int update_roi(Fg_Struct *fg, int label, int w, int h, double e, double f, int init);
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

int method1(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f)
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

	update_roi(fg, 0xabf, w, h, e, f, DO_INIT);
	get_images(fg, num_imgs);
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_start);
	while(i < num_imgs) {
		QueryPerformanceCounter((LARGE_INTEGER *) (pc_ts + i));
		img_nr = Fg_getLastPicNumberBlocking(fg, i + 1, PORT_A, TIMEOUT);
		if(img_nr > FG_OK) {
			img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
			update_roi(fg, i, w, h, e, f, !DO_INIT);
		}
		nr[i] = img_nr;
		i++;
	}
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_stop);

	printf("%d %d %f -1\n", w, h, e);
	rc = QueryPerformanceFrequency((LARGE_INTEGER *) &freq);
	printf("%lld %lld %lld -1\n", 
		((rc == TRUE) ? freq : -ENODEV), loop_start, loop_stop);
	for(i = 0; i < num_imgs; i++) {
		fg_ts[i] = nr[i];
		rc = Fg_getParameter(fg, FG_TIMESTAMP, fg_ts + i, PORT_A);
		printf("%d %lld %lld -1\n", nr[i], pc_ts[i], ((rc < 0) ? rc : fg_ts[i]));
	}

	show_images(fg, nr, num_imgs, w, h);

	free(nr);
	free(pc_ts);
	free(fg_ts);
	return FG_OK;
}

int method2(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f)
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

	update_roi(fg, 0, w, h, e, f, DO_INIT);
	get_images(fg, num_imgs);
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_start);
	while(i < num_imgs) {
		QueryPerformanceCounter((LARGE_INTEGER *) (pc_start + i));
		if(Fg_sendSoftwareTrigger(fg, PORT_A) < 0) { // this always seems to return success, never busy
			printf("swt: %s\n", Fg_getLastErrorDescription(fg));
		}
		img_nr = Fg_getLastPicNumberBlocking(fg, i + 1, PORT_A, TIMEOUT);
		if(img_nr > FG_OK) {
			img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
			update_roi(fg, 0, w, h, e, f, DO_INIT);
		}
		nr[i] = img_nr;
		QueryPerformanceCounter((LARGE_INTEGER *) (pc_stop + i));

		i++;
	}
	QueryPerformanceCounter((LARGE_INTEGER *) &loop_stop);

	printf("%d %d %f -1\n", w, h, e);
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


int method3(Fg_Struct *fg, int num_imgs, int w, int h, double e, double f)
{
	int i = 0, img_nr = 0;
	int *nr;
	double act_f;
	unsigned char *img = NULL;
	unsigned __int64 *fg_ts, pic1, pic2;

	nr = (int *) calloc(num_imgs, sizeof(int));
	if(nr == NULL) {
		return -ENOMEM;
	}

	fg_ts = (unsigned __int64 *) calloc(num_imgs, sizeof(unsigned __int64));
	if(fg_ts == NULL) {
		return -ENOMEM;
	}

	///////// find the largest frame time that breaks the programmed frame time

	act_f = 0;
	update_roi(fg, 0xfab, w, h, e, f, DO_INIT);
	while(1) {
		get_images(fg, GRAB_TWO);
		img_nr = Fg_getLastPicNumberBlocking(fg, i + 2, PORT_A, TIMEOUT);
		if(img_nr > FG_OK) {
			pic1 = img_nr - 1;
			if(Fg_getParameter(fg, FG_TIMESTAMP, (LARGE_INTEGER *) &pic1, PORT_A) < FG_OK) {
				continue;
			}
			
			pic2 = img_nr;
			if(Fg_getParameter(fg, FG_TIMESTAMP, (LARGE_INTEGER *) &pic2, PORT_A) < FG_OK) {
				continue;
			}

			if(pic2 < pic1) {
				// overflow
				act_f = (_UI64_MAX - pic1) + pic2 + 1;
			}
			else {
				act_f = pic2 - pic1;
			}

			// stopping condition
			if(fabs(act_f - f) > FT_ERROR) {
				break;
			}

			if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
				printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
			}

			f -= FT_DELTA;
			update_roi(fg, 0xfab, w, h, e, f, !DO_INIT);
			printf("%f\n", f);
		}
		else {
			// an error occurred
			printf("method3 loop1: %s\n", Fg_getLastErrorDescription(fg));
			return img_nr;
		}
	}

	///////// use f found above as largest frame time that breaks the programmed frame time
	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
	}

	i = 0;
	act_f = f - num_imgs/2*FT_DELTA;
	update_roi(fg, 0xfab, w, h, e, act_f, DO_INIT);
	while(i < num_imgs) {
		get_images(fg, GRAB_TWO);
		img_nr = Fg_getLastPicNumberBlocking(fg, i + 2, PORT_A, TIMEOUT);
		if(img_nr > FG_OK) {
			nr[i] = img_nr;

			pic1 = img_nr - 1;
			if(Fg_getParameter(fg, FG_TIMESTAMP, (LARGE_INTEGER *) &pic1, PORT_A) < FG_OK) {
				continue;
			}
			
			pic2 = img_nr;
			if(Fg_getParameter(fg, FG_TIMESTAMP, (LARGE_INTEGER *) &pic2, PORT_A) < FG_OK) {
				continue;
			}

			if(pic2 < pic1) {
				// overflow
				fg_ts[i] = (_UI64_MAX - pic1) + pic2 + 1;
			}
			else {
				fg_ts[i] = pic2 - pic1;
			}

			if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
				printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
			}

			i++;
			f = FT_DELTA*i + act_f;
			update_roi(fg, i, w, h, e, f, !DO_INIT);
		}
		else {
			// an error occurred
			printf("method3 loop2: %s\n", Fg_getLastErrorDescription(fg));
			return img_nr;
		}
	}

	///////////////////////////////////////////////////////////

	printf("%d %d %f\n", w, h, e);
	for(i = 0; i < num_imgs; i++) {
		printf("%d %lld %lld\n", nr[i], fg_ts[i]);
	}

	show_images(fg, nr, num_imgs, w, h);

	free(nr);
	free(fg_ts);
	return FG_OK;
}