#ifndef __COMMON_H_
#define __COMMON_H_

#define TDAH_PROP_IS_ROI -17001
#define TDAH_PROP_NEXT_DOT -17000
#define NO_MORE_DOTS -1

class Dot;
class Dots;
class Camera;
class Tracker;
class TrackingAlg;

/** @brief a read-only vector type for active dots */
typedef const std::vector<Dot*> ActiveDots;

#endif /* __COMMON_H_ */