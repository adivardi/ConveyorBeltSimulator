import numpy as np
from pykalman import KalmanFilter
from IPython import embed

dt = 100.0/1000.0
conv_speed = (np.pi/4,  1, 0.2)

class BoxKF:
  """ Defines a Kalman Filter for a box on a conveyor """
  def __init__(self, dt, conv_speed, x0 = np.zeros((1,4)), covar0 = 5 * np.eye(4).reshape((1,4,4))):
    self.dt = dt
    self.conv_speed = conv_speed
    self.state_dim = 4

    # --- parameters ---
    self.A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 0, 0], [0, 0, 0, 0]])
    self.B = np.array([[0,0], [0,0], [1,0], [0,1]])
    self.u = np.array([conv_speed[1] * np.cos(conv_speed[0]), conv_speed[1] * np.sin(conv_speed[0])])    #conveyer speed factored into x and y
    self.C = np.eye(4)
    self.d = np.zeros(4,)
    self.R = 0.7*np.eye(4)   # state (process) noise covariance
    self.Q = 1.0*np.eye(4)   # observation noise covariance
    self.x0 = x0
    self.covar0 = covar0
    transition_matrix = self.A
    transition_offset = np.matmul(self.B, self.u)        # B*u = b*v_conv
    observation_matrix = self.C
    observation_offset = self.d
    transition_covariance = self.R
    observation_covariance = self.Q
    initial_state_mean = self.x0
    initial_state_covariance = self.covar0

    self.kf = KalmanFilter(transition_matrix, observation_matrix, transition_covariance, observation_covariance, transition_offset, observation_offset, initial_state_mean, initial_state_covariance)

    self.states_means = self.x0       # list of predictions of states means from KF
    self.states_covars = self.covar0  # list of predictions of states covariances from KF
    self.observations = self.x0       # list of the observations

  def update(self, observation):
    """ Updates the Kalman Filter using a new observation """
    # run the Kalman filer
    new_mean, new_covar = (self.kf.filter_update(self.states_means[-1,:], self.states_covars[-1,:], observation))  #can add transition_offset=... to use offset based on actual control u if u isnt constant

    # update observations, means and covariances lists
    self.observations = np.append(self.observations, observation.reshape((1,self.state_dim)), axis=0)
    self.states_means = np.append(self.states_means, new_mean.reshape((1,self.state_dim)), axis=0)
    self.states_covars = np.append(self.states_covars, new_covar.reshape((1,self.state_dim,self.state_dim)), axis=0)

  def observe(self, observation_ideal):
    """ Corrupts the ideal observation by noise, jumping or hijacking """
    observation_ideal = observation_ideal.reshape(4,)   # make sure observayion has the correct dimensions

    # add Gaussian noise
    noise = np.random.normal(self.observe.noiseMean, self.observe.noiseVar, 4)
    observation = observation_ideal + noise

    return observation

  observe.noiseMean = 0
  observe.noiseVar = 0.1

  def predict(self, steps, x, y):
    means = np.array([[x,y,0,0]])
    covars = 5 * np.eye(4).reshape((1,4,4))

    for t in range(steps):
      if t == 0:
        continue

      new_observation = np.array([means[t-1,0], means[t-1,1], means[t-1,2], means[t-1,3]])
      new_mean, new_covar = (self.kf.filter_update(means[t-1,:], covars[t-1,:], new_observation))

      means = np.append(means, new_mean.reshape((1,4)), axis=0)
      covars = np.append(covars, new_covar.reshape((1,4,4)), axis=0)

    return means
