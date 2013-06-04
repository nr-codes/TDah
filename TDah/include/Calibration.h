#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <string>
#include <vector>
#include <cv.h>
#include <highgui.h>

/** @brief default window size */
#define WIN_SIZE cv::Size(5, 5)
/** @brief default zero zone */
#define ZERO_ZNE cv::Size(-1, -1)
/** @brief default error tolerance */
#define ERR_TOL cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1)

/** @brief contains information useful for a camera calibration
*
* If used in the correct calibration routines, it can be safely assumed
* that the first index of any non-empty vector (1D or 2D) will correspond 
* to the i-th image taken by the member function getViews.  For example, 
* after calling getViews with save_views = true, chessboards[i], world[i], 
* and pixel[i] correspond to the same image.  Furthermore, for 2D vectors
* points located at world[i][j] correspond to points at pixel[i][j].  
* 
* Many of the parameters in this class are optional and default values are
* provided that work well in the general case.  The only value that must
* be set prior to calling any calibration routine is find_chessboard.grid
* to the column by row layout of the chessboard corners.
* 
* @note when setting grid, which is of type Size, remember that 
* width x height = cols x rows
*/

class Calibration {
public:
	typedef std::vector<std::vector<cv::Point3f>> VVP3f;
	typedef std::vector<std::vector<cv::Point2f>> VVP2f;

	struct {
		cv::Mat A; /**< camera matrix */
		cv::Mat k; /**< distortion coefficient */
		std::string file; /**< file name to save or load from */
	} intrinsic_params; /**< parameters for getIntrinsics and getExtrinsics */

	struct {
		cv::Mat R; /**< rotation matrix */
		cv::Mat t; /**< translation vector */
		std::string file; /**< file name to save or load from */
	} extrinsic_params; /**< parameters for getExtrinsics */

	struct {
		int n; /**< the number of views to take */
		bool prompt; /**< if true, prompts the user before adding a picture */
		VVP3f world; /**< a vector of vectors containing world locations */
		VVP2f pixel; /**< a vector of vectors containing pixel locations */
		bool save_views; /**< if true, images of the pattern are saved */
		std::vector<cv::Mat> imgs; /**< a vector of images */
	} views; /**< parameters for getChessboardViews */

	struct {
		int flags; /** passed to the flags parameter */
		cv::Size grid; /**< the size of the chessboard corners */
	} find_chessboard; /**< parameters for findChessboardCorners */

	struct {
		int dilate; /**< number of times to dilate image */
		int erode; /**< number of time to erode image */
		int thr1; /**< first threshold value for Canny threshold1 */
		int thr2; /**< second threshold value for Cannny threshold2 */
	} polka_dots;

	struct {
		cv::Size win; /**< passed to winSize parameter */
		cv::Size zz; /**< passed to zeroZone parameter */
		cv::TermCriteria crit; /**< passed to criteria parameter */
	} sub_pixel; /**< parameters for cornerSubPix */

	struct {
		int flags; /**< passed to the flags parameter */
		std::vector<cv::Mat> rvecs; /**< passed to the rvecs parameter */
		std::vector<cv::Mat> tvecs; /**< passed to the tvecs parameter */
	} calib_cam; /**< parameters for calibrateCamera */

	struct {
		bool useExtGuess; /**< passed to useExtrinsicGuess parameter */
	} solve_pnp; /**< parameters for solvePnP */

	/** @brief the constructor */
	Calibration();
	Calibration(cv::Size grid, int nimgs = 15, 
		bool prompt = false, bool save_views = false);

	/** @brief sets the parameters to useful default values */
	void setDefaults();
	/** @brief changes the current grid size of the chessboard */
	void setGridSize(cv::Size grid);
	/** if true, saves all images of the rig captured by views(...) */
	void saveViews(bool save);
	void clearVectors(); /**< clears all members with vector<> type */
	
	/** @brief uses default XY world coordinate points */
	void xyWorldPoints();
	/** @brief loads world coordinate points from file */
	bool loadWorldPoints(std::string filename);
	/** @brief maps world and image points using a chessboard pattern */
	int getChessboardViews(cv::VideoCapture* cam, std::string title = "");
	/** @brief maps world and image points using a grid pattern */
	int getPolkaDotViews(cv::VideoCapture* cam, std::string title = "");
	/** @brief maps world and image points using an arbitrary grid pattern */
	int getClickViews(cv::VideoCapture* cam, std::string title = "");
	/** @brief maps world and image points using an arbitrary grid pattern Automatically*/
	int getClickViewsAuto(cv::VideoCapture* cam, std::string title = "");
	/** @brief performs an intrinsic camera calibration */
	double getIntrinsics(cv::Size img_size);
	/** @brief performs an extrinsic camera calibration */
	double getExtrinsics(const cv::Mat& Tr = cv::Mat());
	/** @brief reprojects the world points back to the image plane */
	void reproject(cv::Scalar pixel_color, cv::Scalar reproj_color);

	/** save and load camera parameters **/
	bool writeExtrinsics();
	static bool writeExtrinsics(const std::string& file, 
		const cv::Mat& R, const cv::Mat& t);

	bool readExtrinsics();
	static bool readExtrinsics(const std::string& file, 
		cv::Mat& R, cv::Mat& t);

	bool writeIntrinsics();
	static bool writeIntrinsics(const std::string& file, 
		const cv::Mat& A, const cv::Mat& k);

	bool readIntrinsics();
	static bool readIntrinsics(const std::string& file, 
		cv::Mat& A, cv::Mat& k);
};

extern bool simple_intrinsic(Calibration& calib, cv::VideoCapture* cap);
extern bool simple_extrinsic(Calibration& calib, cv::VideoCapture* cap, 
							cv::Mat& Tr = cv::Mat());


#endif /* _CALIBRATION_H_ */