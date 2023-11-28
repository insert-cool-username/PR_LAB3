import numpy as np
import math
from MCLocalization import MCLocalization

class PFMBL(MCLocalization):
    """
    Particle Filter Map Based Localization class.

    This class defines a Map Based Localization using a Particle Filter. It inherits from :class:`MCLocalization`, so the Prediction step is already implemented.
    It needs to implement the Update function, and consecuently the Weight and Resample functions.
    """
    def __init__(self, zf_dim, M, *args) -> None:
        
        self.zf_dim = zf_dim  # dimensionality of a feature observation
        
        self.M = M
        self.nf = len(M)
        super().__init__(*args)
        self.zk = []
        self.Rf = []


    def Weight(self, z, R): 
        """
        Weight each particle by the liklihood of the particle being correct.
        The probability the particle is correct is given by the probability that it is correct given the measurements (z). 

        
        :param z: measurement vector
        :param R: measurement noise covariance
        :return: None
        """
         # To be completed by the student
        def pdf(mean, sigma, x):
            """Compute the PDF for a normal distribution. A lot faster that scipy.stats.norm(mean, sigma).pdf(x)"""
            return 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (x-mean)**2 / (2 * sigma**2))
        for i , particle in enumerate(self.particles):
            for landmark_position, measured_distance in z:
                calculated_distance = np.linalg.norm(particle[:2] - landmark_position)
                error = pdf(measured_distance,R,calculated_distance)
                #prob = (1.0 / (1+error))
                self.particle_weights[i] = self.particle_weights[i] * error 

        # print(self.particle_weights)
        sum_weights = sum(self.particle_weights)
        self.particle_weights = [w / sum_weights for w in self.particle_weights]
        #print(self.particles[0] , self.particle_weights[0])
    
    
    def Resample(self):
        """
        Resample the particles based on their weights to ensure diversity and prevent particle degeneracy.

        This function implements the resampling step of a particle filter algorithm. It uses the weights
        assigned to each particle to determine their likelihood of being selected. Particles with higher weights
        are more likely to be selected, while those with lower weights have a lower chance.

        The resampling process helps to maintain a diverse set of particles that better represents the underlying
        probability distribution of the system state. 

        After resampling, the attributes 'particles' and 'weights' of the ParticleFilter instance are updated
        to reflect the new set of particles and their corresponding weights.

        :return: None
        """
        
        #print(self.particle_weights)
        num_particles = len(self.particles)
        sum_weights = sum(self.particle_weights)
        self.particle_weights = [w / sum_weights for w in self.particle_weights]
        indices = np.random.choice(num_particles, num_particles, p=self.particle_weights)
        #print(indices)
        print("Resample")
        self.particles = [self.particles[idx] for idx in indices]
        self.particle_weights = np.ones(len(self.particles)) / len(self.particles)
        #print(self.particle_weights)
        return 
    
    def Update(self, z, R):
        """
        Update the particle weights based on sensor measurements and perform resampling.

        This function adjusts the weights of particles based on how well they match the sensor measurements.
       
        The updated weights reflect the likelihood of each particle being the true state of the system given
        the sensor measurements.

        After updating the weights, the function may perform resampling to ensure that particles with higher
        weights are more likely to be selected, maintaining diversity and preventing particle degeneracy.
        
        :param z: measurement vector
        :param R: the covariance matrix associated with the measurement vector

        """
        self.Weight(z,R)
        print("Update")
        if self.k % 500==0:
            self.Resample()

    def Localize(self):
        uk, Qk = self.GetInput()

        if uk.size > 0:
            self.Prediction(uk, Qk)
        if self.k==0 or self.k%250 == 0:
            self.zf, self.Rf = self.GetMeasurements()
            if len(self.zf) > 0:
                self.Update(self.zf, self.Rf)

        self.PlotParticles()
        return self.get_mean_particle()