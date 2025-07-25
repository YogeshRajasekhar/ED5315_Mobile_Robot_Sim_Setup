import numpy as np

class update_eq:

        def update(self,pose,covariance,meas,R,I):
        
                # <------------ Write your code here ------------>
                # Define the update equation for the pose and covariance
                # Use numpy arrays as matrices for calculations
                hx = np.array([
                (pose[0]**2 + pose[1]**2),
                pose[2]])
            
                y=meas-hx #Convert to measurement domain #meas is probably an array
                y[1] = ((y[1] + np.pi) % (2 * np.pi)) - np.pi
                #y[2]= ((y[2]+np.pi)%(2*np.pi))-np.pi
                H=np.array([
                        [2*pose[0],2*pose[1],0],
                        [0,0,1]])
                s=np.matmul(np.matmul(H,covariance),((H).transpose()))+R
                k=np.matmul(np.matmul(covariance,(H.transpose())),(np.linalg.inv(s)))
                pose=pose+np.matmul(k,y)
                pose[2] = ((pose[2] + np.pi) % (2 * np.pi)) - np.pi
                covariance=np.matmul((I-np.matmul(k,H)),covariance)

                # <------------ End of your code ------------>

                return pose, covariance