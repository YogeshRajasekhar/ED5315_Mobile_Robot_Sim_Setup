import numpy as np

class matrix_calculator:

    def f_p2(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled by right and left wheel
        delta_d = (left_dist_moved + right_dist_moved)/2
        delta_theta = (right_dist_moved - left_dist_moved)/L

        #calcualting the jacobian of the states with respect to the previous state
        F_p = np.array(([1,0,-(delta_d*np.sin(pose[2]+delta_theta/(2)))],[0,1,(delta_d*np.cos(pose[2]+delta_theta/(2)))],[0,0,1]))

        #########################################################

        return F_p
    def f_p(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled by right and left wheel
        DL=left_dist_moved
        DR=right_dist_moved
        delta_theta = (DR - DL)/L
        #R= (DL/delta_theta)
        theta = pose[2]

        if abs(DL - DR) < 1e-6:  # Straight line
            d = DL
            theta = pose[2]
            F_p = np.array([
                [1, 0, -d * np.sin(theta)],
                [0, 1,  d * np.cos(theta)],
                [0, 0,  1]
            ])

        else:

            sigma1 = ((L/2) - ((DL*L)/(DL-DR)))

            f11 = 1
            f12 = 0
            f13 = (np.cos(theta - ((DL-DR)/L)) * sigma1) - (np.cos(theta) * sigma1)

            f21 = 0
            f22 = 1
            f23 = (np.sin(theta - ((DL-DR)/L)) * sigma1) - (np.sin(theta) * sigma1)

            f31 = 0
            f32 = 0
            f33 = 1

            F_p = np.array(([f11,f12,f13],[f21,f22,f23],[f31,f32,f33]))
        

        #########################################################

        return F_p
    
    def f_u2(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled y right and left wheel
        delta_d = (left_dist_moved + right_dist_moved)/2
        delta_theta = (right_dist_moved - left_dist_moved)/L
        
        #calcualting the components of the jacobain of the state with respect to the control inputs 
        f11= (1/2)*(np.cos(pose[2]+delta_theta/2)) - (delta_d/(2*L))*np.sin(pose[2]+delta_theta/2)
        f12= (1/2)*(np.cos(pose[2]+delta_theta/2)) + (delta_d/(2*L))*np.sin(pose[2]+delta_theta/2)

        f21 = (1/2)*(np.sin(pose[2]+delta_theta/2)) + (delta_d/(2*L))*np.cos(pose[2]+delta_theta/2)
        f22 = (1/2)*(np.sin(pose[2]+delta_theta/2)) - (delta_d/(2*L))*np.cos(pose[2]+delta_theta/2)

        f31 = (1/L)
        f32 = -(1/L)

        F_u = np.array(([f11,f12],[f21,f22],[f31,f32]))

        #########################################################

        return F_u
    
    def f_u(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled y right and left wheel
        #delta_d = (left_dist_moved + right_dist_moved)/2
        DL=left_dist_moved
        DR=right_dist_moved
        delta_theta = (DR - DL)/L
        #R= (DL/delta_theta)
        theta = pose[2]

        if abs(DL - DR) < 1e-6:  # Straight line
            theta = pose[2]
            F_u = np.array([
                [0.5 * np.cos(theta), 0.5 * np.cos(theta)],
                [0.5 * np.sin(theta), 0.5 * np.sin(theta)],
                [0, 0]
            ])

        else:

            sigma1 = ((L)/(DL-DR)) - (DL*L)/((DL - DR)**2)
            sigma5 = theta - ((DL - DR)/L)
            sigma4 = (L/2) - ((DL*L)/(DL-DR))
            sigma2 = (np.cos(sigma5)*sigma4)/L
            sigma3 = (np.sin(sigma5)*sigma4)/L
            
            #calcualting the components of the jacobain of the state with respect to the control inputs 
            f11= sigma2 - ((DL*L*np.sin(sigma5))/((DL-DR)**2)) + ((DL*L*np.sin(theta))/((DL-DR)**2))
            f12= (np.sin(theta)*sigma1) - (np.sin(sigma5)*sigma1) - sigma2

            f21 = sigma3 + ((DL*L*np.cos(sigma5))/((DL-DR)**2)) - ((DL*L*np.cos(theta))/((DL-DR)**2))
            f22 = (np.cos(sigma5)*sigma1) - (np.cos(theta)*sigma1) - sigma3

            f31 = (1/L)
            f32 = -(1/L)

            F_u = np.array(([f11,f12],[f21,f22],[f31,f32]))

        #########################################################

        return F_u
