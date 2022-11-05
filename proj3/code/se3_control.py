import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # 2.3e-08 N/(rad/s)**2 (equivalent to Kf - forces)
        self.k_drag          = quad_params['k_drag']   #  7.8e-11 Nm/(rad/s)**2 (equivalent to Km - moment)
        # self.rotor_speed_min = 0    # rad/s
        # self.rotor_speed_max = 2500 # rad/s

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        
        # Original gains from waypoint traj -- constant velocity speed controller (proj1_1) -- tuned for speed 2.5 m/s (gemetric controller)
        # self.Kp = 0.5*np.array([[6, 0, 0],
        #                    [0, 6, 0],
        #                    [0, 0, 6]])
        # self.Kd = 0.5*np.array([[3.5, 0, 0],
        #                    [0, 3.5, 0],
        #                    [0, 0, 3.5]])
        # self.Kr = 0.5*np.array([[430, 0, 0],
        #                    [0, 430, 0],
        #                    [0, 0, 132]]) # Kp equivalent
        # self.Kw = 0.5*np.array([[23, 0, 0],
        #                    [0, 23, 0],
        #                    [0, 0, 15]]) # Kd equivalent

        # New gains for project 3
        # self.Kp = 0.5*np.array([[5, 0, 0],
        #                    [0, 5, 0],
        #                    [0, 0, 5]])
        # self.Kd = 0.5*np.array([[3.5, 0, 0],
        #                    [0, 3.5, 0],
        #                    [0, 0, 3.5]])
        # self.Kr = 0.5*np.array([[3750, 0, 0],
        #                    [0, 3750, 0],
        #                    [0, 0, 135]]) # Kp equivalent
        # self.Kw = 0.5*np.array([[100, 0, 0],
        #                    [0, 100, 0],
        #                    [0, 0, 20]]) # Kd equivalent

        self.Kp = 0.6*np.array([[5, 0, 0],
                                [0, 5, 0],
                                [0, 0, 5]])
        self.Kd = 0.6*np.array([[3.5, 0, 0],
                                [0, 3.5, 0],
                                [0, 0, 3.5]])
        self.Kr = 0.5*np.array([[3750, 0, 0],
                                [0, 3750, 0],
                                [0, 0, 135]]) # Kp equivalent
        self.Kw = 0.5*np.array([[100, 0, 0],
                                [0, 100, 0],
                                [0, 0, 20]]) # Kd equivalent


    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
                q, quaternion [i,j,k,w]-
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        # STUDENT CODE HERE
        r_ddot_des = flat_output['x_ddot'] - self.Kd @ (state['v'] - flat_output['x_dot']) - self.Kp @ (state['x'] - flat_output['x']) # (3, )
        # print("r_ddot_des: \n", r_ddot_des)
        force_des = self.mass*r_ddot_des + np.array([0, 0, self.mass*self.g]) # (1x3)
        # print("force_des: \n", force_des)
        u1 = (Rotation.from_quat(state['q']).as_matrix() @ np.array([[0], [0], [1]])).transpose() @ force_des # (1x1)
        # print("u1: \n:", u1)
        b3_des = force_des/np.linalg.norm(force_des) # (3,)
        # print("b3_des: \n", b3_des)
        # print("SE3CONTROL: ", flat_output['yaw'])
        a_phi = np.array([[np.cos(flat_output['yaw']),
                           np.sin(flat_output['yaw']),
                           0]])
        b2_des = np.cross(b3_des, a_phi)/np.linalg.norm(np.cross(b3_des, a_phi)) # (1,3)
        # print("b2_des: \n", b2_des)
        b1_des = np.cross(b2_des, b3_des) # (1,3)
        # print("b1_des: \n", b1_des)
        R_des = np.concatenate((b1_des.reshape(3,-1), b2_des.reshape(3,-1), b3_des.reshape(3,-1)), axis = 1)
        # print("R_des: \n", R_des)
        e_R = 0.5*(R_des.transpose() @ Rotation.from_quat(state['q']).as_matrix() - Rotation.from_quat(state['q']).as_matrix().transpose() @ R_des) # Skew-symmetric matrix
        e_R = np.array([[e_R[2,1]],
                        [e_R[0,2]],
                        [e_R[1,0]]])
        # print("New e_R: \n", e_R)
        e_omega = state['w']# - flat_output['yaw_dot']
        # print("e_omega: \n", e_omega)

        u2 = self.inertia @ (-self.Kr@e_R.reshape(-1,1) - self.Kw@e_omega.reshape(-1,1)) #???

        wrench = np.array([u1[0], u2[0][0], u2[1][0], u2[2][0]])
        # print("wrench: \n", wrench)
        gamma = self.k_drag/self.k_thrust
        mixer = np.array([[self.k_thrust, self.k_thrust, self.k_thrust, self.k_thrust],
                          [0, self.arm_length*self.k_thrust, 0, -self.arm_length*self.k_thrust],
                          [-self.arm_length*self.k_thrust, 0, self.arm_length*self.k_thrust, 0],
                          [gamma*self.k_thrust, -gamma*self.k_thrust, gamma*self.k_thrust, -gamma*self.k_thrust]])

        omegas_sq1 = np.linalg.inv(mixer) @ wrench

        omegas_sq = np.clip(omegas_sq1, self.rotor_speed_min**2, self.rotor_speed_max**2)

        omegas = np.sqrt(omegas_sq)

        cmd_motor_speeds = np.array([[omegas[0],omegas[1],omegas[2],omegas[3]]]).flatten()
        cmd_thrust = u1
        cmd_moment = omegas[0:4].reshape(4,)**2*self.k_drag
        cmd_q = Rotation.as_quat(Rotation.from_matrix(R_des))


        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}

        return control_input
