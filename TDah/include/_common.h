#ifndef __COMMON_H_
#define __COMMON_H_

#include <cv.h>

#define BAD_TAG -1

enum {
	TDAH_PROP_IS_ROI = -1700, 
	TDAH_PROP_NEXT_DOT
};

class Dot;
class Dots;
class Camera;
class Tracker;
class TrackingAlg;

/** @brief a read-only vector type for active dots */
typedef const std::vector<Dot*> ActiveDots;

class Util {
public:
	Util(const char* func_name)
	{
		_s = func_name;
		trace();
	};

	void start()
	{
		_t = cv::getTickCount() / ( cv::getTickFrequency() * 1e-3 );
	};

	void stop()
	{
		_t = cv::getTickCount() / ( cv::getTickFrequency() * 1e-3 ) - _t;
		printf("%s: %0.5g us\n", _s, _t);
	};

	void trace()
	{
		printf("%s\n", _s);
	};

	void msg(const char* msg)
	{
		printf("%s: %s");
	};

private:
	double _t;
	const char* _s;
};

#define _TRACE
#if defined( _TRACE )
	#define UTIL( func ) Util u983247( (func) )
	#define TRACE() u983247.trace()
	#define START() u983247.start()
	#define STOP() u983247.stop()
	#define MSG( m ) u983247.msg( (m) )
#else
	#define UTIL( func )
	#define TRACE()
	#define START()
	#define STOP()
	#define MSG( m )
#endif

#endif /* __COMMON_H_ */