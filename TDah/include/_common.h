#ifndef __COMMON_H_
#define __COMMON_H_

#include <vector>
#include <cv.h>

#define BAD_TAG -1

enum {
	TDAH_PROP_IS_ROI = -1700, 
	TDAH_PROP_NEXT_DOT,
	TDAH_PROP_LAST_GRABBED_IMAGE,
	TDAP_PROP_LAST_TRANSFERRED_IMAGE,
	TDAH_PROP_MIN_FRAME_TIME,
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
	Util(const char* func_name, size_t n = 100)
	{
		_func = func_name;
		_start.reserve(n);
		_stop.reserve(n);
		trace();
	};

	void start()
	{
		_start.push_back( static_cast<double> ( cv::getTickCount() ) );
	};

	void stop()
	{
		_stop.push_back( static_cast<double> ( cv::getTickCount() ) );
	};

	void reset()
	{
		_start.clear();
		_stop.clear();
	}

	void printResults()
	{
		double min, max;
		cv::Scalar mean, stddev;
		size_t n = std::min(_start.size(), _stop.size());

		if(!n) {
			return;
		}

		_start.resize(n);
		_stop.resize(n);

		double freq = 1e-6 * cv::getTickFrequency();
		cv::Mat time =  (cv::Mat(_stop) - cv::Mat(_start)) / freq;
		cv::MatConstIterator_<double> it;
		for(it = time.begin<double>(); it < time.end<double>(); ++it) {
			printf("%0.5g us\n", (*it));
		}

		cv::meanStdDev(time, mean, stddev);
		cv::minMaxLoc(time, &min, &max);
		printf("avg: %0.5g us +/- %0.5g us\n", mean[0], stddev[0]);		
		printf("min: %0.5g us max: %0.5g us\n", min, max);
	}

	void trace()
	{
		printf("%s\n", _func);
	};

	void msg(const char* msg)
	{
		printf("%s: %s\n", _func, msg);
	};

private:
	std::vector<double> _start, _stop;
	const char* _func;
};

#endif /* __COMMON_H_ */