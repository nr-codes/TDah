#include "t_dah.h"

#define GRAVITY 9.8f

// TODO: figure out correct interface for kalman filter, should it predict k+1?
void prediction(CvKalman *kal, float dt_k, float *z_k)
{
	float u_k[] = {0};

	float A_k[] = { 1, 0, dt_k, 0, 
					0, 1, 0, dt_k, 
					0, 0, 1, 0,
					0, 0, 0, 1};
	
	float B_k[] = { 0, 
					0.5f*GRAVITY*dt_k*dt_k, 
					0, 
					GRAVITY*dt_k};

	CvMat z = cvMat(Z_DIM, 1, CV_32FC1, z_k);
	CvMat u = cvMat(U_DIM, 1, CV_32FC1, u_k);

	memcpy(kal->transition_matrix->data.fl, A_k, sizeof(A_k));
	memcpy(kal->control_matrix->data.fl, B_k, sizeof(B_k));

	cvKalmanPredict(kal, &u);
	cvKalmanCorrect(kal, &z);

	printf("pred: \npre ");
	for(int i = 0; i < X_DIM; i++) {
		printf("%1.5g ", kal->state_pre->data.fl[i]);
	}
	printf(" \npost ");
	for(int i = 0; i < X_DIM; i++) {
		printf("%1.5g ", kal->state_post->data.fl[i]);
	}
	printf("\n");

	printf("K\t");
	for(int i = 0; i < kal->gain->rows; i++) {
		for(int j = 0; j < kal->gain->cols; j++) {
			printf("%1.5g ", kal->gain->data.fl[i*kal->gain->cols + j]);
		}
		printf("\n\t");
	}

	printf("\nA\t");
	for(int i = 0; i < kal->transition_matrix->rows; i++) {
		for(int j = 0; j < kal->transition_matrix->cols; j++) {
			printf("%1.5g ", 
				kal->transition_matrix->data.fl[i*kal->transition_matrix->cols + j]);
		}
		printf("\n\t");
	}

	printf("\nB\t");
	for(int i = 0; i < kal->control_matrix->rows; i++) {
		for(int j = 0; j < kal->control_matrix->cols; j++) {
			printf("%1.5g ", 
				kal->control_matrix->data.fl[i*kal->control_matrix->cols + j]);
		}
		printf("\n\t");
	}
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
		cvSetIdentity(k->measurement_matrix, cvRealScalar(1));

		// might be able to tune based on off-line analysis
		cvSetIdentity(k->process_noise_cov, cvRealScalar(1e-0));
		//cvSetIdentity(k->process_noise_cov, cvRealScalar(1e-1));

		// should know after experiments
		cvSetIdentity(k->measurement_noise_cov, cvRealScalar(1e-0));
		//cvSetIdentity(k->measurement_noise_cov, cvRealScalar(1e-5));

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
			cvSetIdentity(k->error_cov_post, cvRealScalar(1));
			//cvSetIdentity(k->error_cov_post, cvRealScalar(100));
		}
	}
}
