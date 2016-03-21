import numpy as np
import math

def LinearCalibration (point_2d, point_3d):
	num_point = point_2d.shape[0]

	# Initialize the A matrix
	A = np.zeros(shape=(2*num_point,12))

	# Assign rows to A each at a time
	for i in range(num_point):
		M_i = point_3d[i,:].T
		c_i = point_2d[i,0]
		r_i = point_2d[i,1]

		A[2*i,0:3] = M_i.T
		A[2*i,3] = 1
		A[2*i,8:11] = -c_i*M_i.T
		A[2*i,11] = -c_i

		A[2*i+1,4:7] = M_i.T
		A[2*i+1,7] = 1
		A[2*i+1,8:11] = -r_i*M_i.T
		A[2*i+1,11] = -r_i
	# Do SVD to A
	U, D, S = np.linalg.svd(A)
	V_prime = S.T[:,-1]
	# Scale V_prime
	alpha = 1/np.linalg.norm(V_prime[8:11])
	V= V_prime * alpha

	P = np.resize(V,(3,4))
	return P

def RANSAC(data_2d, data_3d, K, threshold, s):
	N = data_2d.shape[0]
	best_subset_list = []
	all_inliers_list = []
	largest_count = 0
	for i in range(s):
		# Draw K random samples
		K_list = np.random.randint(0,N,K)
		point_2d = data_2d[K_list,:]
		point_3d = data_3d[K_list,:]
		# Estiamte P
		P = LinearCalibration(point_2d, point_3d)

		# Count the inliers
		inliers_list = []
		for j in range(N):
			p_2d = data_2d[j,:]
			p_3d = data_3d[j,:]

			# Calculate projected 2D points
			lambda_c_r_1 = np.dot(P,np.append(p_3d,1))
			p_2d_prj = lambda_c_r_1[0:2]/lambda_c_r_1[2]
			# print p_2d, p_2d_prj
			if np.linalg.norm(p_2d - p_2d_prj) <= threshold:
				inliers_list.append(j)

		if len(inliers_list)>largest_count:
			best_subset_list = K_list.copy()
			all_inliers_list = list(inliers_list)
			largest_count = len(inliers_list)

	print "RANSAC solution based on "+str(len(all_inliers_list))+" inliers.\n"
	# Recompute the best P
	Best_P = LinearCalibration(data_2d[all_inliers_list,:],data_3d[all_inliers_list,:])
	return Best_P

def P2Parameters(P):
	R = np.zeros(shape=(3,3))
	T = np.zeros(shape=(3,1))
	R[2,:] = P[2,0:3]
	T[2] = P[2,3]
	c_0 = P[0,0:3].dot(P[2,0:3])
	r_0 = P[1,0:3].dot(P[2,0:3])
	s_xf = math.sqrt(P[0,0:3].dot(P[0,0:3])-c_0*c_0)
	s_yf = math.sqrt(P[1,0:3].dot(P[1,0:3])-r_0*r_0)
	T[0] = (P[0,3]-c_0*T[2])/s_xf
	T[1] = (P[1,3]-r_0*T[2])/s_yf
	R[0,:] = (P[0,0:3]-c_0*R[2,:])/s_xf
	R[1,:] = (P[1,0:3]-r_0*R[2,:])/s_yf

	print "R: \n", R
	print "T: \n", T
	print "s_x*f, s_y*f:\n", [s_xf, s_yf]
	print "c_0, r_0: \n", [c_0, r_0]
	return [R, T, s_xf, s_yf, c_0, r_0]


###############################################
############# Main function starts here
###############################################

# Change filenames to select data sets
filename_data_2d = "Left_2Dpoints.txt"
filename_data_3d = "bad_3dpts.txt"
data_2d = np.genfromtxt(filename_data_2d)
data_3d = np.genfromtxt(filename_data_3d)

point_2d = data_2d
point_3d = data_3d

P = LinearCalibration(point_2d, point_3d)
print "P from linear camera calibration: \n", P

P2Parameters(P)

Best_P = RANSAC(data_2d,data_3d,10, 2, 1000)
print "P from RANSAC: \n", Best_P

P2Parameters(Best_P)