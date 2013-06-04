#define NUM_MARKERS 4  //# of the tracking markers
#define CALIBRA_NUM 98  //This is the number of total dots on the calibration board

#define TESTMODE 0 // Testmode is for testing and calibration, "0" is for testing and "1" is for calibration
#define OUTMODE 1  // Determine output data to text file or not, "1" means will write all data to file

#define FRAMERATE 250
#define Time_to_record 10 //in second
#define FILENAME "data4.txt" //The name for output file

#define rid 100   //Since at the beginning the data seems to be a little unstable, so get 'rid' of the first # data.

#define framecnt int(FRAMERATE*Time_to_record + rid)
#define err 20