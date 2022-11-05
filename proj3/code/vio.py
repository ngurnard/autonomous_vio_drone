#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    new_p = p + v*dt + 0.5 * (q.as_matrix() @ (a_m - a_b) + g) * dt**2
    new_v = v + (q.as_matrix() @ (a_m - a_b) + g) * dt
    new_q = q * Rotation.from_rotvec(((w_m - w_b) * dt).flatten())

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return: error state covariance matrix update
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    a_diff = a_m - a_b
    a_cross = np.array([[0, -a_diff[2, 0], a_diff[1,0]],
                        [a_diff[2,0], 0, -a_diff[0,0]],
                        [-a_diff[1,0], a_diff[0,0], 0]])

    Fx = np.block([[np.eye(3), np.eye(3) * dt, np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3)),  np.zeros((3,3))],
                    [np.zeros((3,3)), np.eye(3), (-q.as_matrix() @ a_cross)*dt, -q.as_matrix() * dt, np.zeros((3,3)), np.eye(3)*dt],
                    [np.zeros((3,3)), np.zeros((3,3)), (Rotation.from_rotvec(((w_m - w_b) * dt).flatten())).as_matrix().transpose(), np.zeros((3,3)), -np.eye(3)*dt, np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]])
    # print("fx: ", Fx.shape) # should be (18,18)

    Fi = np.block([[np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
                    [np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.eye(3)],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))]])
    # print("Fi: ", Fi.shape) # should be (18,12)

    Qi = np.block([[accelerometer_noise_density**2 * dt**2 * np.eye(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))],
                    [np.zeros((3,3)), gyroscope_noise_density**2 * dt**2 * np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), accelerometer_random_walk**2 * dt * np.eye(3), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), gyroscope_random_walk**2 * dt * np.eye(3)]])
    # print("Qi: ", Qi.shape) # should be (12,12)

    new_covariance = Fx @ error_state_covariance @ Fx.transpose() + Fi @ Qi @ Fi.transpose() # return an 18x18 covariance matrix - error state covariance matrix update
    return new_covariance


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements - single image measurement
    :param Pw: 3x1 vector world coordinate - coordinates of the point with respect to the world coordinate system
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix - associated with the image measurement covariance
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    Pc = q.as_matrix().transpose() @ (Pw - p) # world to camera frame - (3,1)
    innovation = uv - np.array([[Pc[0,0]], [Pc[1,0]]])/Pc[2,0] # (2,1)

    # In order to account for outliers we compare the magnitude of this vector to a specified error
    # threshold, if the vector magnitude is greater than this quantity, no update is performed and the original
    # state and covariance are returned, otherwise we compute updated versions of the nominal state and error
    # state covariance
    if (np.linalg.norm(innovation) > error_threshold): # not an inlier
        return nominal_state, error_state_covariance, innovation
    else: # is an inlier
        # Compute the relevant covariance matrix H
        dz_dPc = 1/Pc[2,0] * np.array([[1, 0, -Pc[0,0]/Pc[2,0]], [0, 1, -Pc[1,0]/Pc[2,0]]]) # slide 58 of lecture 20 - (2,3)
        dPc_dDeltaTheta = np.array([[0, -Pc[2,0], Pc[1,0]],
                                    [Pc[2,0], 0, -Pc[0,0]],
                                    [-Pc[1,0], Pc[0,0], 0]]) # slide 59 of lecture 20 - (3,3)
        dPc_dDeltaP = -q.as_matrix().transpose() # slide 59 of lecture 20 - (3,3)
        dz_dDeltaTheta = dz_dPc @ dPc_dDeltaTheta  # (2,3)
        dz_dDeltaP = dz_dPc @ dPc_dDeltaP # (2,3)
        Ht = np.block([dz_dDeltaP, np.zeros((2,3)), dz_dDeltaTheta, np.zeros((2,3)), np.zeros((2,3)), np.zeros((2,3))]) # should be (2,18)

        # Compute EKF update delta_x
        Kt = error_state_covariance @ Ht.transpose() @ np.linalg.inv(Ht @ error_state_covariance @ Ht.transpose() + Q) # first compute the Kalman gain (18,2)
        delta_x = Kt @ innovation # compute the EKF update delta_x. Should be (18,1)

        # Update nominal state, x, using delta_x
        p_new = p + delta_x[:3] # new estimate of the position because the error = estimate - previous_estimate
        v_new = v + delta_x[3:6] # new estimate of the velocity
        a_b_new = a_b + delta_x[9:12]# new accelerometer bias estimate
        w_b_new = w_b + delta_x[12:15] # new gyrometer bias estimate
        g_new = g + delta_x[15:18] # new gravity bias estimate
        # if np.all(g_new != g):
        #     print(np.all(g_new == g))
        # else:
        #     # print(".")
        q_new = q * Rotation.from_rotvec(delta_x[6:9,0]) # new quaternion estimate

        # Update error state covariance matrix
        error_state_covariance_new = (np.eye(18) - Kt @ Ht) @ error_state_covariance @ (np.eye(18) - Kt @ Ht).transpose() + Kt @ Q @ Kt.transpose()

        return (p_new, v_new, q_new, a_b_new, w_b_new, g_new), error_state_covariance_new, innovation
