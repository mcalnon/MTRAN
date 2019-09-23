/***********************************************************************/
/*                                                                     */
/*  FILE        :mark.c        	                                       */
/*  DATE        :June, 30, 2009                                    	   */
/*  DESCRIPTION :Autonomous Docking Program by Mark Calnon             */
/*  CPU TYPE    :SH7047                                                */
/*                                                                     */
/***********************************************************************/
#include  <stdio.h>
#include  <string.h>
#include  <stdlib.h>
#include  <machine.h>
#include  <math.h>
#include  "iodefine.h"
#include  "Api.h"
#include  "common.h"
#include  "h8.h"
#include  "h8IR.h"
#include  "control.h"
#include  "initialConf.h"

#include  "mark.h"
#include  "markPF.h"

//
// Global Variables
//

extern struct {
	unsigned char mode;
	unsigned char face;
	unsigned char dark, bright;
} irDetect;


//
// Particle Filter: Pose Particles
//

PoseParticle _pose_particles1[NUM_POSE_PARTICLES];
PoseParticle _pose_particles2[NUM_POSE_PARTICLES];

PoseParticle *PoseParticles = _pose_particles1;
PoseParticle *NextPoseParticles = _pose_particles2;

PoseParticle BestPoseParticle;

//
// Particle Filter: Perception Models
//

// Perception Model for the Active IR Sensors
float ActiveIRModelMean[NUM_ACTIVE_IR_MODEL_GAUSSIANS][3] = 
{
	{11, 12, 13}
};
/*
float ActiveIRModelCovariance[NUM_ACTIVE_IR_MODEL_GAUSSIANS][3][3] = 
{
	{{11, 0, 0}, 
	 {0, 12, 0}, 
	 {0, 0, 13}}
};
*/
float ActiveIRModelCovarianceInverse[NUM_ACTIVE_IR_MODEL_GAUSSIANS][3][3] = 
{
	{{1/11, 0, 0}, 
	 {0, 1/12, 0}, 
	 {0, 0, 1/13}}
};

float ActiveIRModelCovarianceDeterminantSqrt[NUM_PASSIVE_IR_MODEL_GAUSSIANS] = 
{
	41.4246
};

float ActiveIRModelPrior[NUM_ACTIVE_IR_MODEL_GAUSSIANS] = 
{
	1
};

float ActiveIRModelCholesky[NUM_ACTIVE_IR_MODEL_GAUSSIANS][3][3] = 
{
	{{3.3166, 0, 0}, 
	 {0, 3.4641, 0}, 
	 {0, 0, 3.6065}}
};

// Error Model for the Active IR Sensors
float ActiveIRErrorMean = 0;
float ActiveIRErrorVariance = 0;
float ActiveIRErrorVarianceInverse = 0;
float ActiveIRErrorVarianceSqrt = 0;

// Perception Model for the Passive IR Sensors
float PassiveIRModelMean[NUM_PASSIVE_IR_MODEL_GAUSSIANS][3] = 
{
	{1, 2, 3}, 
	{4, 5, 6}, 
	{7, 8, 9}
};
/*
float PassiveIRModelCovariance[NUM_PASSIVE_IR_MODEL_GAUSSIANS][3][3] = 
{
	{{1, 0, 0}, 
	 {0, 2, 0}, 
	 {0, 0, 3}},
		
	{{4, 0, 0}, 
	 {0, 5, 0}, 
	 {0, 0, 6}},
		
	{{7, 0, 0}, 
	 {0, 8, 0}, 
	 {0, 0, 9}}
};
*/
float PassiveIRModelCovarianceInverse[NUM_PASSIVE_IR_MODEL_GAUSSIANS][3][3] = 
{
	{{1/1, 0, 0}, 
	 {0, 1/2, 0}, 
	 {0, 0, 1/3}},
		
	{{1/4, 0, 0}, 
	 {0, 1/5, 0}, 
	 {0, 0, 1/6}},
		
	{{1/7, 0, 0}, 
	 {0, 1/8, 0}, 
	 {0, 0, 1/9}}
};

float PassiveIRModelCovarianceDeterminantSqrt[NUM_PASSIVE_IR_MODEL_GAUSSIANS] = 
{
	2.4495, 
	10.9545, 
	22.4499
};

float PassiveIRModelPrior[NUM_PASSIVE_IR_MODEL_GAUSSIANS] = 
{
	.25, 
	.25, 
	.50
};

float PassiveIRModelCholesky[NUM_PASSIVE_IR_MODEL_GAUSSIANS][3][3] = 
{
	{{1, 0, 0}, 
	 {0, 1.4142, 0}, 
	 {0, 0, 1.7321}},
		
	{{2, 0, 0}, 
	 {0, 2.2361, 0}, 
	 {0, 0, 2.4495}},
		
	{{2.6458, 0, 0}, 
	 {0, 2.8284, 0}, 
	 {0, 0, 3}}
};

// Error Model for the Passive IR Sensors
float PassiveIRErrorMean = 0;
float PassiveIRErrorVariance = 0;
float PassiveIRErrorVarianceInverse = 0;
float PassiveIRErrorVarianceSqrt = 0;

//
// Particle Filter: Motion Models
//

float MotionModelXMean[NUM_LOCOMOTION_MODES] = // in cm/sec
{
	3.19706, 
	-0.94991, 
	-0.52588, 
	3.64395, 
	-1.4376, 
	1.4376
};

float MotionModelXVariance[NUM_LOCOMOTION_MODES] = 
{
	0.09353, 
	0.26308, 
	0.03953, 
	0.03837, 
	0.26745, 
	0.26745
};

float MotionModelYMean[NUM_LOCOMOTION_MODES] = // cm/sec
{
	-0.64593, 
	3.31631, 
	3.27966, 
	-0.47462, 
	-1.18516, 
	-1.18516
};

float MotionModelYVariance[NUM_LOCOMOTION_MODES] = 
{
	0.05768, 
	0.17445, 
	0.12847, 
	0.11266, 
	0.06343, 
	0.06343
};

float MotionModelThetaMean[NUM_LOCOMOTION_MODES] = // in rad/sec
{
	0.32640, 
	-0.84825, 
	-0.92534, 
	-0.81846, 
	4.55219, 
	-4.55219
};

float MotionModelThetaVariance[NUM_LOCOMOTION_MODES] = 
{
	0.01162, 
	0.02963, 
	0.02089, 
	0.18993, 
	0.14306, 
	0.14306
};

//
// Particle Filter: Main Functions
//

// Performs a single iteration of the Pose Particle Filter
void PoseParticlesIteration(unsigned char loc_direction, unsigned long elapsed_time)
{
	PredictPoseParticles(loc_direction, elapsed_time);
	UpdatePoseParticles();
	EstimateBestPose();
	ResamplePoseParticles();
}

// Initializes all Pose Particles by sampling from the Passive IR Model
void InitializePoseParticles()
{
	int i;
	
	srand(GetTickCount());
	
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		InitializePoseParticle(i);
	}
}

// Initializes a single Pose Particle by sampling from the Passive IR Model
void InitializePoseParticle(int particle_index)
{
	// Sample from the Passive IR Model N(PassiveIRModelMean, PassiveIRModelCovariance)
	randMultivariateGaussianMM(&PoseParticles[particle_index].state, 3,
		PassiveIRModelMean, PassiveIRModelCholesky, PassiveIRModelPrior, NUM_PASSIVE_IR_MODEL_GAUSSIANS);
}

// Predicts a new pose for all Pose Particles by sampling from the Motion Model
void PredictPoseParticles(unsigned char loc_direction, unsigned long elapsed_time)
{
	int i;
	float delta_x, delta_y, delta_theta;
	
	// Update the motion model according to the elapsed time
	float x_mean = MotionModelXMean[loc_direction] * elapsed_time;
	float x_variance = MotionModelXVariance[loc_direction] * elapsed_time;
	
	float y_mean = MotionModelYMean[loc_direction] * elapsed_time;
	float y_variance = MotionModelYVariance[loc_direction] * elapsed_time;
	
	float theta_mean = MotionModelThetaMean[loc_direction] * elapsed_time;
	float theta_variance = MotionModelThetaVariance[loc_direction] * elapsed_time;
	
	// Reverse direction
	switch (loc_direction)
	{
		case FORWARD_DIRECTION: 
		case LEFT_DIRECTION: 
			x_mean = -x_mean;
			y_mean = -y_mean;
			theta_mean = -theta_mean;
			break;
	}
	
	// Predict all pose particles
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		// Sample from the motion model
		delta_x = randGaussian() * x_variance + x_mean;
		delta_y = randGaussian() * y_variance + y_mean;
		delta_theta = randGaussian() * theta_variance + theta_mean;
	
		// Update pose particle orientation (before calculating new x & y position)
		PoseParticles[i].state.theta += delta_theta;
	
		// Update pose particle position
		PoseParticles[i].state.x += 
			cos(PoseParticles[i].state.theta - HALF_PI) * delta_x +
			cos(PoseParticles[i].state.theta) * delta_y;
			
		PoseParticles[i].state.y += 
			sin(PoseParticles[i].state.theta - HALF_PI) * delta_x +
			sin(PoseParticles[i].state.theta) * delta_y;
	}
}

// Updates the weight for all Pose Particles based on the Active and Passive IR Models
void UpdatePoseParticles()
{
	int i;
	float weight_sum = 0;
	float active_model_value, passive_model_value;
	float active_model_probility, passive_model_probility;
	
	// Update all pose particles
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		// Calculate the expected IR values according to the Active and Passive IR Models
		active_model_value = evalMultivariateGaussianMM(&PoseParticles[i].state, 3, NORMALIZATION_RATIO_THREE, 
			ActiveIRModelMean, ActiveIRModelCovarianceDeterminantSqrt, ActiveIRModelCovarianceInverse, ActiveIRModelPrior, NUM_ACTIVE_IR_MODEL_GAUSSIANS);
	
		passive_model_value = evalMultivariateGaussianMM(&PoseParticles[i].state, 3, NORMALIZATION_RATIO_THREE, 
			PassiveIRModelMean, PassiveIRModelCovarianceDeterminantSqrt, PassiveIRModelCovarianceInverse, PassiveIRModelPrior, NUM_PASSIVE_IR_MODEL_GAUSSIANS);
	
		// Calculate the Active and Passive IR probabilities
		active_model_probility = evalGaussian(irDetect.bright - active_model_value,
			ActiveIRErrorMean, ActiveIRErrorVarianceSqrt, ActiveIRErrorVariance);
			
		active_model_probility = evalGaussian(irDetect.dark - passive_model_value,
			PassiveIRErrorMean, PassiveIRErrorVarianceSqrt, PassiveIRErrorVariance);
	
		// Update pose particle weight
		PoseParticles[i].weight = active_model_probility * passive_model_probility;
		
		// Sum all weights
		weight_sum += PoseParticles[i].weight;
	}
	
	// Normalize all weights
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		PoseParticles[i].weight /= weight_sum;
	}
}

// Estimates the best pose based on:
//	- the most likely Pose Particle
void EstimateBestPose()
{
	int i;
	int max_index = 0;
	float max_weight = 0;
	
	// Find max weight
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		if (PoseParticles[i].weight > max_weight)
		{
			max_index = i;
			max_weight = PoseParticles[i].weight;
		}
	}
	
	BestPoseParticle = PoseParticles[max_index];
}

// Estimates the best pose based on:
//	- the weighted mean of all particles near the most likely Pose Particle
void EstimateBestPoseMean()
{
	int i;
	int max_index = 0;
	float max_weight = 0;
	
	// Find max weight
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		if (PoseParticles[i].weight > max_weight)
		{
			max_index = i;
			max_weight = PoseParticles[i].weight;
		}
	}
	
	BestPoseParticle.state.x = 0;
	BestPoseParticle.state.y = 0;
	BestPoseParticle.state.theta = 0;
	BestPoseParticle.weight = 0;

	// Caclulate the weighted mean of the best pose
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		if (abs(PoseParticles[i].state.x - PoseParticles[max_index].state.x) < NEARBY_X_THRESHOLD &&
			abs(PoseParticles[i].state.y - PoseParticles[max_index].state.y) < NEARBY_Y_THRESHOLD &&
			abs(PoseParticles[i].state.theta - PoseParticles[max_index].state.theta) < NEARBY_THETA_THRESHOLD)
		{
			BestPoseParticle.state.x += PoseParticles[i].state.x * PoseParticles[i].weight;
			BestPoseParticle.state.y += PoseParticles[i].state.y * PoseParticles[i].weight;
			BestPoseParticle.state.theta += PoseParticles[i].state.theta * PoseParticles[i].weight;
			BestPoseParticle.weight += PoseParticles[i].weight;
		}
	}

	// Normalize the best pose
	BestPoseParticle.state.x /= BestPoseParticle.weight;
	BestPoseParticle.state.y /= BestPoseParticle.weight;
	BestPoseParticle.state.theta /= BestPoseParticle.weight;
}

// Resamples all Pose Particles based on their current weight
void ResamplePoseParticles()
{
	int i, j, num_particles;
	float weight_cumulative_sum[NUM_POSE_PARTICLES];
	float rand_sum, rand_cumulative_sum[NUM_POSE_PARTICLES];
	PoseParticle *TempPoseParticles;
	
	// Calculate the cumulative sum of particle weights and random numbers
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		weight_cumulative_sum[i] = PoseParticles[i].weight +
			(i == 0) ? 0 : weight_cumulative_sum[i - 1];
			
		rand_cumulative_sum[i] = -log(randUniform()) +
			(i == 0) ? 0 : rand_cumulative_sum[i - 1];
	}
	
	rand_sum = -log(randUniform()) + rand_cumulative_sum[NUM_POSE_PARTICLES - 1];
	
	// Normalize the random numbers (uniform between 0 and 1)
	for (i = 0; i < NUM_POSE_PARTICLES; ++i)
	{
		rand_cumulative_sum[i] /= rand_sum;
	}
	
	// Resample existing particles
	num_particles = NUM_POSE_PARTICLES - NUM_RANDOM_RESAMPLES;
	
	for (i = 0, j = 0; i < num_particles; )
	{
		if (rand_cumulative_sum[i] < weight_cumulative_sum[j])
			NextPoseParticles[i++] = PoseParticles[j];
		else
			++j;
	}
	
	// Resample remaining particles randomly
	while (i < NUM_POSE_PARTICLES)
	{
		InitializePoseParticle(i++);
	}
	
	// Update pointers
	TempPoseParticles = PoseParticles;
	PoseParticles = NextPoseParticles;
	NextPoseParticles = TempPoseParticles;
}

//
// Helper Functions
//

// Random Number Functions

// Returns a uniformly distributed random number between 0.0 and 1.0
float randUniform()
{
	return rand() / (float)RAND_MAX;
}

// Returns a normally distributed random number with a mean of 0.0 and a standard deviation of 1.0
float randGaussian()
{
	static float next_gaussian = 0;
	static unsigned char has_next_gaussian = FALSE;
	
	if (has_next_gaussian)
	{
		has_next_gaussian = FALSE;
		return next_gaussian;
	}
	else
	{
		float r1, r2, s, multiplier;
		
		do
		{
			r1 = 2 * randUniform() - 1; // uniformly distributed between -1.0 and 1.0
			r2 = 2 * randUniform() - 1; // uniformly distributed between -1.0 and 1.0
			s = r1 * r1 + r2 * r2;
		}
		while (s == 0 || s >= 1);
		
		multiplier = sqrt(-2 * log(s) / s);
		
		next_gaussian = r2 * multiplier;
		has_next_gaussian = TRUE;
		
		return r1 * multiplier;
	}
}

// Generates a random number by sampling the provided Multivariate Gaussian
void randMultivariateGaussian(float *data_out, unsigned char data_count,
	float *mean, float *cholesky, float prior)
{
	int i;
	float *rand_data = malloc(data_count * sizeof(float));
	
	// Initialize new random values: N(0, 1)
	for (i = 0; i < data_count; ++i)
	{
		rand_data[i] = randGaussian();
	}

	// Perform matrix multiplication with the Cholesky factorization of the covariance
	MatrixMultiply(data_out, rand_data, cholesky, 1, data_count, data_count);

	// Add in the Gaussian mean
	VectorAdd(data_out, data_out, mean, data_count);

	// And weight according to the Gaussian prior
	VectorMultiplyScalar(data_out, data_out, prior, data_count);
	
	// Deallocate dynamic memmory
	free(rand_data);
}

// Generates a random number by sampling the provided Gaussian Mixture Model
// - mean is (g x d), cholesky is (g x d x d), prior is (g x 1)
void randMultivariateGaussianMM(float *data_out, unsigned char data_count,
	float *mean, float *cholesky, float *prior, unsigned char gaussian_count)
{
	int i, g;
	int m_index = 0, c_index = 0;
	int data_count_sq = data_count * data_count;
	
	float *rand_data = malloc(data_count * sizeof(float));
	
	// Initialize data
	for (i = 0; i < data_count; ++i)
	{
		data_out[i] = 0;
	}
	
	// Sample from the Gaussian Mixture Model
	for (g = 0; g < gaussian_count; ++g)
	{
		// Sample from the current Gaussian
		randMultivariateGaussian(rand_data, data_count, &mean[m_index], &cholesky[c_index], prior[g]);
		
		// And add it to the Gaussian Mixture
		VectorAdd(data_out, data_out, rand_data, data_count);
		
		// Update indexes
		m_index += data_count;
		c_index += data_count_sq;
	}
	
	// Deallocate dynamic memmory
	free(rand_data);
}

// Gaussian Functions

float evalGaussian(float data, float mean, float variance_sqrt, float variance)
{
	float data_centered = data - mean;
	
	return 1 / (NORMALIZATION_RATIO_ONE * variance_sqrt) * 
		exp(-0.5 * data_centered * data_centered / variance);
}

float evalMultivariateGaussian(float *data, unsigned char data_count, float normalization_ratio,
	float *mean, float covariance_determinant_sqrt, float *covariance_inverse, float prior)
{
	float *data_centered = malloc(data_count * sizeof(float));
	float *data_centered_sq = malloc(data_count * sizeof(float));
	float data_final;
	
	VectorSubtract(data_centered, data, mean, data_count);
	MatrixMultiply(data_centered_sq, data_centered, covariance_inverse, 1, data_count, data_count);
	MatrixMultiply(&data_final, data_centered_sq, data_centered, 1, data_count, 1);
	
	// Deallocate dynamic arrays
	free(data_centered);
	free(data_centered_sq);
	
	return prior / (normalization_ratio * covariance_determinant_sqrt) * 
		exp(-0.5 * data_final);
}
	
float evalMultivariateGaussianMM(float *data, unsigned char data_count, float normalization_ratio,
	float *mean, float *covariance_determinant_sqrt, float *covariance_inverse, float *prior, unsigned char gaussian_count)
{
	int g;
	int m_index = 0, c_index = 0;
	int data_count_sq = data_count * data_count;
	
	float result = 0;
		
	// Evaluate the Gaussian Mixture Model
	for (g = 0; g < gaussian_count; ++g)
	{
		// Evaluate the current Gaussian
		result += evalMultivariateGaussian(data, data_count, normalization_ratio, 
			&mean[m_index], covariance_determinant_sqrt[g], &covariance_inverse[c_index], prior[g]);
		
		// Update indexes
		m_index += data_count;
		c_index += data_count_sq;
	}
	
	return result;
}

// Matrix and Vector Math Functions

// Performs 2D Matrix Multiplication
// - data_lhs (m x n) * data_rhs (n x p) = data_out (m x p)
void MatrixMultiply(float *data_out, float *data_lhs, float *data_rhs, 
	unsigned char m, unsigned char n, unsigned char p)
{
	int i, j, k;
	int lhs_index, rhs_index, out_index = 0;
	
	for (i = 0; i < m; ++i) // for each row (lhs)
	{
		for (j = 0; j < p; ++j) // for each col (rhs)
		{
			lhs_index = i * n; // initialize row
			rhs_index = j;     // initialize column
				
			data_out[out_index] = 0;
			
			for (k = 0; k < n; ++k) // for each col (lhs) and row (rhs)
			{
				data_out[out_index] += data_lhs[lhs_index] * data_rhs[rhs_index];
				
				lhs_index += 1; // next column
				rhs_index += p; // next row
			}
			
			out_index += 1; // next row/column
		}
	}
}

void VectorMultiply(float *data_out, float *data_lhs, float *data_rhs, unsigned char m)
{
	int i;
	
	for (i = 0; i < m; ++i)
	{
		data_out[i] = data_lhs[i] * data_rhs[i];
	}
}

void VectorMultiplyScalar(float *data_out, float *data_lhs, float data_rhs, unsigned char m)
{
	int i;
	
	for (i = 0; i < m; ++i)
	{
		data_out[i] = data_lhs[i] * data_rhs;
	}
}

void VectorAdd(float *data_out, float *data_lhs, float *data_rhs, unsigned char m)
{
	int i;
	
	for (i = 0; i < m; ++i)
	{
		data_out[i] = data_lhs[i] + data_rhs[i];
	}
}

void VectorAddScalar(float *data_out, float *data_lhs, float data_rhs, unsigned char m)
{
	int i;
	
	for (i = 0; i < m; ++i)
	{
		data_out[i] = data_lhs[i] + data_rhs;
	}
}

void VectorSubtract(float *data_out, float *data_lhs, float *data_rhs, unsigned char m)
{
	int i;
	
	for (i = 0; i < m; ++i)
	{
		data_out[i] = data_lhs[i] - data_rhs[i];
	}
}

void VectorSubtractScalar(float *data_out, float *data_lhs, float data_rhs, unsigned char m)
{
	int i;
	
	for (i = 0; i < m; ++i)
	{
		data_out[i] = data_lhs[i] - data_rhs;
	}
}

void VectorSum(float *data_out, float *data_lhs, unsigned char m)
{
	int i;
	
	data_out[0] = 0;
	
	for (i = 0; i < m; ++i)
	{
		data_out[0] += data_lhs[i];
	}
}
