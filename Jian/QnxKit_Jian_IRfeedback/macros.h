#ifndef macros_h
#define macros_h

#include <stdlib.h>

// Connect macro
//#define CONNECT(out, in) (in) = &(out)

// Warn macro
#define WARN_IF(EXP) \
do { if ((EXP)==-1) printf ("Warning: " #EXP " %d %s\n", __LINE__, __FILE__); } while (0)

// Error macro
#define DIE_IF(EXP) \
do { if ((EXP)==-1) {printf ("Error: " #EXP " %d %s\n", __LINE__, __FILE__); exit(0);} } while (0)

#define FATAL_ERROR(MSG) \
do { printf("Error: %s %d %s\n", MSG, __LINE__, __FILE__); exit(0); } while (0)


#endif
