/***********************************************************************/
/*                                                                     */
/*  FILE        :mark.h                                                */
/*  DATE        :June, 30, 2009                                    	   */
/*  DESCRIPTION :Particle Filter by Mark Calnon                        */
/*  CPU TYPE    :SH7047                                                */
/*                                                                     */
/***********************************************************************/

//
// Particle Filter Defines
//

#define NUM_POSE_PARTICLES		80		// Total number of Pose Particles in the Particle Filter
#define NUM_RANDOM_RESAMPLES	10		// X of the Pose Particles will be randomly sampled from the IR Model during resampling

// Threshold for determining nearby particles when estimating the best mean pose
#define NEARBY_X_THRESHOLD			2		// cm
#define NEARBY_Y_THRESHOLD			2		// cm
#define NEARBY_THETA_THRESHOLD		0.349	// radians

// Perception model
#define NUM_ACTIVE_IR_MODEL_GAUSSIANS	1
#define NUM_PASSIVE_IR_MODEL_GAUSSIANS	3

// Motion model
#define NUM_LOCOMOTION_MODES		6

// Gaussian constants
#define PI				3.14159
#define HALF_PI			PI / 2

#define NORMALIZATION_RATIO_ONE		2.5066	// (2*PI)^(1/2)
#define NORMALIZATION_RATIO_TWO		6.2830	// (2*PI)^(2/2)
#define NORMALIZATION_RATIO_THREE	15.7496	// (2*PI)^(3/2)

// Other defines
#define X		0
#define Y		1
#define THETA	2

//
// Useful Particle Filter Structures
//

typedef struct
{
	float weight;
	
	struct
	{
		float x, y, theta;
	} state;
	
} PoseParticle;

//
// Particle Filter: Main Functions
//

void InitializePoseParticles();
void InitializePoseParticle(int particle_index);

void PoseParticlesIteration(unsigned char loc_direction, unsigned long elapsed_time);

void PredictPoseParticles(unsigned char loc_direction, unsigned long elapsed_time);
void UpdatePoseParticles();
void EstimateBestPose();
void EstimateBestPoseMean();
void ResamplePoseParticles();

//
// Helper Functions
//

// Random Number Functions
float randUniform();
float randGaussian();
void randMultivariateGaussian(float *data_out, unsigned char data_count,
	float *mean, float *cholesky, float prior);
void randMultivariateGaussianMM(float *data_out, unsigned char data_count,
	float *mean, float *cholesky, float *prior, unsigned char gaussian_count);

// Gaussian Functions
float evalGaussian(float data, float mean, float variance_sqrt, float variance);
float evalMultivariateGaussian(float *data, unsigned char data_count, float normalization_ratio,
	float *mean, float covariance_determinant_sqrt, float *covariance_inverse, float prior);
float evalMultivariateGaussianMM(float *data, unsigned char data_count, float normalization_ratio,
	float *mean, float *covariance_determinant_sqrt, float *covariance_inverse, float *prior, unsigned char gaussian_count);

// Matrix and Vector Math Functions
void MatrixMultiply(float *data_out, float *data_lhs, float *data_rhs, 
	unsigned char m, unsigned char n, unsigned char p);
//void VectorMultiply(float *data_out, float *data_lhs, float *data_rhs, unsigned char m);
void VectorMultiplyScalar(float *data_out, float *data_lhs, float data_rhs, unsigned char m);
void VectorAdd(float *data_out, float *data_lhs, float *data_rhs, unsigned char m);
//void VectorAddScalar(float *data_out, float *data_lhs, float data_rhs, unsigned char m);
void VectorSubtract(float *data_out, float *data_lhs, float *data_rhs, unsigned char m);
//void VectorSubtractScalar(float *data_out, float *data_lhs, float data_rhs, unsigned char m);
//void VectorSum(float *data_out, float *data_lhs, unsigned char m);