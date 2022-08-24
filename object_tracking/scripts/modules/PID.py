#-*-utf-8-*-
'''
Class for PID Loop
'''
import numpy as np
def computeAvgDer(error_list, dts,current_index):
    '''
    Helper function to compute average velocity across the last amount of frames
    '''
    err = 0.0
    for i in range(current_index, -len(error_list)+current_index+1, -1):
        err += (error_list[i]-error_list[i-1])/dts[i]
    return err / len(error_list)
class PID:
    '''
    main pid class
    '''
    def __init__(self, deadband, p_coeff, i_coeff, d_coeff, i_max, sample_size=1):
        '''
        I think a pretty straightforward initialization function
        '''
        #set vars
        self.deadband = deadband
        self.p_coeff = p_coeff
        self.i_coeff = i_coeff
        self.d_coeff = d_coeff
        self.i_max = i_max

        #create lists to store past errors and dts so that velocity calculation can be made (probably not gonna be useful)
        self.sample_size = sample_size+1
        self.previous_errors = np.zeros(self.sample_size)
        self.previous_dts = np.zeros(self.sample_size)
        self.current_sample_index = 0

        self.integral = 0

    def compute(self, error, dt, vel_est = None):
        '''
        computes the command based on fed error and time difference
        can feed vel_est if another source is giving velocity estimate
        '''

        #p term
        p_term = self.p_coeff * error

        #adds to the integral
        self.integral += error * dt
        self.integral = max(-self.i_max, min(self.i_max, self.integral))

        #i term
        i_term = self.i_coeff * self.integral

        #adds to error list for derivative calculation
        self.previous_errors[self.current_sample_index] = error
        self.previous_dts[self.current_sample_index] = dt
        #d term
        d_term = 0
        #if using an exterior velocity estimate just use that 
        if vel_est != None:
            d_term = vel_est * self.d_coeff
        #else compute the average vel from the last specified samples
        else:
            d_term = self.d_coeff * computeAvgDer(self.previous_errors, self.previous_dts, self.current_sample_index)

        #increase and wrap the sample index to use the list as a ring
        self.current_sample_index += 1
        self.current_sample_index = self.current_sample_index % self.sample_size
        
        #return the output
        return p_term + i_term + d_term

    def feed_sample(self,error,dt):
        '''
        useful if you want to feed a sample without actually using the output to continue updating the pd loop
        '''
        self.integral += error * dt
        self.integral = max(-self.i_max, min(self.i_max, self.integral))

        self.previous_errors[self.current_sample_index] = error
        self.previous_dts[self.current_sample_index] = dt

        self.current_sample_index += 1
        self.current_sample_index = self.current_sample_index % self.sample_size

    #gets the avg computed vel (mainly just useful for debugging)
    def get_vel(self):
        return computeAvgDer(self.previous_errors, self.previous_dts, self.current_sample_index)


