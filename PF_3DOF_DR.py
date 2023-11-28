import numpy as np
import scipy
from MCLocalization import MCLocalization
from DifferentialDriveSimulatedRobot import *
from Pose3D import *

class PF_3DOF_DR(MCLocalization):
    def __init__(self, *args):
        super().__init__( *args)

        self.dt = 0.1  # dt is the sampling time at which we iterate the DR
        self.wheelRadius = 0.1  # wheel radius
        self.wheelBase = 0.5  # wheel base
        self.robot.pulse_x_wheelTurns = 4096  # number of pulses per wheel turn
        

    def GetInput(self):
        """
        Get the input for the motion model.

        :return: * **uk, Qk**. uk: input vector (:math:`u_k={}^B[\Delta x~\Delta y]^T`), Qk: covariance of the input noise
        """

        k , Qk_pulses = DifferentialDriveSimulatedRobot.ReadEncoders(self)


        #Obtenemos el desplazamiento en cada rueda
        d_l = k[1]*(1/self.robot.pulse_x_wheelTurns)*(2*np.pi)*(self.wheelRadius)
        d_r = k[0]*(1/self.robot.pulse_x_wheelTurns)*(2*np.pi)*(self.wheelRadius)
        #Obtenemos el desplazamiento del robot
        d = (d_r+d_l)/2

        delta_theta = (d_r - d_l)/self.wheelBase
        
        uk_x= (d*np.cos(delta_theta))
        uk_y = (d*np.sin(delta_theta))


        uk=np.array([uk_x, uk_y, delta_theta]).reshape(3,1)
        uk = Pose3D(uk)

        #mandar otro QK
        sigma = 0.1
        Qk = np.diag([sigma**2 , sigma**2,np.deg2rad(0)**2])

        return uk , Qk
    
    
    def MotionModel(self, particle, u, noise):
        # **To be completed by the student**.
        #hacer un oplus del estado actual de la particula más el u y el ruido.
        
        particle = particle.oplus((u+noise))

        return particle
    


if __name__ == '__main__':

    M = [np.array([[-40, 5]]).T,
           np.array([[-5, 40]]).T,
           np.array([[-5, 25]]).T,
           np.array([[-3, 50]]).T,
           np.array([[-20, 3]]).T,
           np.array([[40,-40]]).T]  # feature map. Position of 2 point features in the world frame.

    #Simulation:
    xs0 = np.zeros((6, 1))
    kSteps = 5000
    index = [IndexStruct("x", 0, None), IndexStruct("y", 1, None), IndexStruct("yaw", 2, 0)]
    robot = DifferentialDriveSimulatedRobot(xs0, M)  # instantiate the simulated robot object
    
    # Particle Filter
    x0 = Pose3D(np.zeros((3,1)))  # initial guess
    P0 = np.diag([2**2, 2**2, np.deg2rad(20)**2]) # Initial uncertainty
    n_particles = 50

    #create array of n_particles particles distributed randomly around x0 with covariance P

    particles_np = np.random.multivariate_normal(x0.reshape(3,) , P0 ,size=(n_particles)) 
    
    #print(particles_np.shape)
    particles = []
    for i in range(particles_np.shape[0]):
        particles.append(Pose3D(particles_np[i,:].reshape(3,1)))
    
    usk=np.array([[0.5, 0.03]]).T
    pf = PF_3DOF_DR(index, kSteps, robot, particles)
    pf.LocalizationLoop(x0, usk)

    exit(0)
