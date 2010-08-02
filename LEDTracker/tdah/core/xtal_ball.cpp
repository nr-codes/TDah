#include "t_dah.h"

#define GRAVITY 9.8f

CV_INLINE CvMat u(CvMat *x_k, float dt_k)
{
	// apply zero control
	float u_k[] = {0};
	return cvMat(U_DIM, 1, CV_32FC1, u_k);
}

void print_mat(char *str, CvMat *mat)
{
	printf("%s:\n", str);
	for(int i = 0; i < mat->rows; i++) {
		printf("\t");
		for(int j = 0; j < mat->cols; j++) {
			printf("%g ", cvmGet(mat, i, j));
		}
		printf("\n");
	}
}

// TODO: beware a bad process model will cause a program crash
void estimate_and_predict(CvKalman *kal, float dt_k, float *z_k)
{
	CvMat z, u;
	float u_k[] = {0};

	float A_k[] = { 1, 0, dt_k, 0, 
					0, 1, 0, dt_k, 
					0, 0, 0, 0,
					0, 0, 0, 0};
	
	float B_k[] = { 0, 
					0.5f*GRAVITY*dt_k*dt_k, 
					0, 
					GRAVITY*dt_k};

	memcpy(kal->transition_matrix->data.fl, A_k, sizeof(A_k));
	memcpy(kal->control_matrix->data.fl, B_k, sizeof(B_k));

	// combine estimate with state at step k
	if(z_k) {
		z = cvMat(kal->MP, 1, CV_32FC1, z_k);
		cvKalmanCorrect(kal, &z);
	}

	// predict state at step k + 1
	u = cvMat(U_DIM, 1, CV_32FC1, u_k);
	cvKalmanPredict(kal, &u);
}

void setup_kalman(CvKalman **kal, int n, float **x0, float **P0)
{
	int i;
	CvKalman *k;

	if(!kal) return;

	for(i = 0; i < n; i++) {
		k = kal[i];
		if(k == NULL) {
			continue;
		}

		// H matrix (mapping from state to camera measurements)
		// assumes first k->MP columns of state vector are 
		// configuration variables
		cvSetIdentity(k->measurement_matrix);

		// might be able to tune based on off-line analysis
		cvSetIdentity(k->process_noise_cov, cvRealScalar(1e-1));

		// should know after experiments
		cvSetIdentity(k->measurement_noise_cov, cvRealScalar(1e-5));

		// initial state
		if(x0 && x0[i]) {
			memcpy(k->state_post->data.fl, x0[i], k->DP*sizeof(float));
		}
		else {
			cvZero(k->state_post);
		}

		// initial covariance matrix
		if(P0 && P0[i]) {
			memcpy(k->error_cov_post->data.fl, P0, k->DP*k->DP*sizeof(float));
		}
		else {
			// don't trust initial guess
			cvSetIdentity(k->error_cov_post, cvRealScalar(100));
		}

		estimate_and_predict(k, 0, NULL);
	}
}
