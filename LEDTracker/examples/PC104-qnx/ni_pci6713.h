#ifndef NI_PCI6713_H_
#define NI_PCI6713_H_

#define ROI_LOST -10

extern int open_analog(void);
extern int close_analog(void);
extern int output_analog(double x, double y, int roi);

#endif /* NI_PCI6713_H_ */