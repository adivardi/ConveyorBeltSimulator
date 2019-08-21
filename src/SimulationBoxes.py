import pygame
import random
import math
import numpy as np
import KFPredictor as KF
from IPython import embed

def addVectors((angle1, length1), (angle2, length2)):
  """ Returns the sum of two vectors """
  x  = math.sin(angle1) * length1 + math.sin(angle2) * length2
  y  = math.cos(angle1) * length1 + math.cos(angle2) * length2
  angle  = 0.5 * math.pi - math.atan2(y, x)
  length = math.hypot(x, y)
  return (angle, length)

def collide(p1, p2):
  """ Tests whether two particles overlap
      If they do, make them bounce, i.e. update their angle, speed and position """
  dx = p1.x - p2.x
  dy = p1.y - p2.y

  dist = math.hypot(dx, dy)
  if dist < p1.size + p2.size:
    # angle = math.atan2(dy, dx) + 0.5 * math.pi
    # total_mass = p1.mass + p2.mass

    # (p1.angle, p1.speed) = addVectors((p1.angle, p1.speed*(p1.mass-p2.mass)/total_mass), (angle, 2*p2.speed*p2.mass/total_mass))
    # (p2.angle, p2.speed) = addVectors((p2.angle, p2.speed*(p2.mass-p1.mass)/total_mass), (angle+math.pi, 2*p1.speed*p1.mass/total_mass))
    # elasticity = p1.elasticity * p2.elasticity
    # p1.speed *= elasticity
    # p2.speed *= elasticity

    tangent = math.atan2(dy, dx)
    angle = 0.5 * math.pi + tangent

    angle1 = 2*tangent - p1.angle
    angle2 = 2*tangent - p2.angle
    elasticity = p1.elasticity * p2.elasticity
    speed1 = p2.speed*elasticity
    speed2 = p1.speed*elasticity

    (p1.angle, p1.speed) = (angle1, speed1)
    (p2.angle, p2.speed) = (angle2, speed2)

    # correcting for boxes sticking to each other
    overlap = 0.5*(p1.size + p2.size - dist + 0.01)
    p1.x += math.sin(angle)*overlap
    p1.y += math.cos(angle)*overlap
    p2.x -= math.sin(angle)*overlap
    p2.y -= math.cos(angle)*overlap

    p1.collided = True
    p2.collided = True

size_m = 0.3
size_s = 0.2
# w_m = 1 #0.25 #(15 deg)
# w_s = 0.1
class Box:
  instanceCounter = 0

  def __init__(self,nLane, dt, env_speed):
    self.id = Box.instanceCounter
    lane = random.randint(0, nLane)
    self.sizeX = size_m + random.uniform(-size_s,size_s)
    self.sizeY = size_m + random.uniform(-size_s,size_s)
    self.size = math.sqrt((0.5*self.sizeX)**2 + (0.5*self.sizeY)**2)

    # centeroid position
    self.x = max(lane*2*size_m + 0.5*self.sizeX, self.size+0.1)
    self.y = max(0.5*self.sizeY, self.size+0.1)

    self.speed = 0
    self.angle = 0
    self.free = True
    self.crossedLimits = False
    self.collided = False

    self.mass = random.randint(1, 20)
    self.elasticity = 0.9
    self.colour = (0, 0, 255)   # blue

    kf_x0 = np.array([[self.x, self.y, 0, 0]])
    kf_covar0 = 5 * np.eye(4).reshape((1,4,4))
    self.kalmanFilter = KF.BoxKF(dt, env_speed, kf_x0, kf_covar0)
    self.colourKF = (0, 255, 0)   # green
    self.colourObs = (255, 0, 0)  # red

    Box.instanceCounter += 1

    print ("New Box %s: pos: (%s , %s) ,  size: (%s , %s), radius: %s" % (self.id, self.x, self.y, self.sizeX, self.sizeY, self.size))

  def move(self, dt, env_speed, perp_speed):
    """ Move a box and update the Kalman filter """
    self.speed = perp_speed[0] + random.uniform(-perp_speed[1], perp_speed[1])
    self.angle = random.uniform(0, math.pi*2)

    env_speed_rand = (env_speed[0], env_speed[1] + random.uniform(-env_speed[2], env_speed[2]))
    (self.angle, self.speed) = addVectors((self.angle, self.speed), env_speed_rand)

    self.x += dt * self.speed * math.sin(self.angle)
    self.y += dt * self.speed * math.cos(self.angle)

    print("Box %s pos: (%s , %s)" % (self.id, self.x, self.y))
    print("Box %s KF pos: (%s , %s)" % (self.id, self.kalmanFilter.states_means[-1,0], self.kalmanFilter.states_means[-1,1]))
    print("Box %s KF error: %s" % (self.id, math.sqrt((self.x - self.kalmanFilter.states_means[-1,0])**2 + (self.y - self.kalmanFilter.states_means[-1,1])**2)))

    # update KF for next step
    self.updateKalman()
    # use KF to predict the next N steps
    # self.predictKalman(30)


  def updateKalman(self):
    observation_ideal = np.array([self.x, self.y, self.speed * math.sin(self.angle), self.speed * math.cos(self.angle)])  # observation of box state - the position and velocity of the box
    print("Observation: %s" % (observation_ideal))
    observation_real = self.kalmanFilter.observe(observation_ideal)  # corrupts the ideal observation by noise, jumping or hijacking
    print("Observation real: %s" % (observation_real))
    self.kalmanFilter.update(observation_real)

  def predictKalman(self, N):
    self.means = self.kalmanFilter.predict(N,self.x,self.y)

  # if checkCrossedCache(box) == False:
  #   # move along acis
  #   v_axis = v_axis_m + uniform(-v_axis_s, v_axis_s)
  #   d_axis = (dt/1000.0)*v_axis
  #   dx = d_axis*np.cos(alpha)
  #   dy = d_axis*np.sin(alpha)
  #   box[0] += dx; box[1] += dy; box[2] += dx; box[3] += dy;

  #   # move perpndicular to axis
  #   v_perp = v_perp_m + uniform(-v_perp_s, v_perp_s)
  #   d_perp = (dt/1000.0)*v_perp
  #   dx = -d_perp*np.sin(alpha)
  #   dy = d_perp*np.cos(alpha)
  #   box[0] += dx; box[1] += dy; box[2] += dx; box[3] += dy;

    # rotate box
    # box = rotateBox(box)  # not working properly when displaying because opencv only displays horizontal rectangles
  # return box

# def rotateBox(box):
#   center = np.array([box[0]+box[2],box[1]+box[3]])/2
#   w = w_m + uniform(-w_s, w_s)
#   dw = (dt/1000.0)*w

#   # rotate 1st corner
#   box2 = np.array(box)
#   box2[0] = center[0] + (np.cos(dw)*(box[0] - center[0])) - (np.sin(dw)*(box[1] - center[1]))
#   box2[1] = center[1] + (np.sin(dw)*(box[0] - center[0])) + (np.cos(dw)*(box[1] - center[1]))
#   # rotate 2nd corner
#   box2[2] = center[0] + (np.cos(dw)*(box[2] - center[0])) - (np.sin(dw)*(box[3] - center[1]))
#   box2[3] = center[1] + (np.sin(dw)*(box[2] - center[0])) + (np.cos(dw)*(box[3] - center[1]))
#   return box2

class Environment:
  """ Defines the boundary of a simulation and its properties """

  def __init__(self, (width, height), res, dt, nLane, env_speed, perp_speed, env_limits):
    self.width = width
    self.height = height
    self.boxes = []   # list of all boxes
    self.dt = dt

    self.env_limits = env_limits  # must be 2d array of shape N x 4 , where N is the number of limits
    self.nLanes = nLane
    self.env_speed = env_speed   # angle and speed of conveyer
    self.perp_speed = perp_speed

    self.colour = (0,0,0)
    self.elasticity = 0.75
    self.res = res

    # setup screen
    self.screen = pygame.display.set_mode((width*res, height*res))
    pygame.display.set_caption('Conveyer Simulation')
    self.screenSize = math.hypot(width*res, height*res)

    self.boxesKFs = []  # list of Kalman filters, one for each box

  def addBoxes(self, n=1, **kargs):
    """ Add n boxes with properties given by keyword arguments """
    for i in range(n):
      box = Box(self.nLanes, self.dt, self.env_speed)
      box.colour = kargs.get('colour', (0, 0, 255))
      self.boxes.append(box)

      # add the Kalmann filter of the box to the list
      self.boxesKFs.append(box.kalmanFilter)

  def moveBoxes(self):
    """  Moves boxs and tests for collisions with the walls and each other """
    for i, box in enumerate(self.boxes):
      if box.free and (not box.crossedLimits):
        box.move(self.dt, self.env_speed, self.perp_speed)
      elif box.collided == True:
        box.move(self.dt, self.env_speed, self.perp_speed)
        box.collided = False

      self.bounce(box)
      for box2 in self.boxes[i+1:]:
        collide(box, box2)

  def bounce(self, box):
    """ Tests whether a box has hit the boundaries of the environment, i.e. the rectangle frame """
    if box.x > self.width - box.size:
      box.x = 2*(self.width - box.size) - box.x
      box.angle = - box.angle
      box.speed *= self.elasticity
      box.free = True   # don't stop when hitting the top

    elif box.x < box.size:
      print "BOX IN LEFT WALL!!!!!!!!!!!!!"
      box.x = 2*box.size - box.x
      box.angle = - box.angle
      box.speed *= self.elasticity
      box.free = False

    if box.y > self.height - box.size:
      box.y = 2*(self.height - box.size) - box.y
      box.angle = math.pi - box.angle
      box.speed *= self.elasticity
      box.free = False

    elif box.y < box.size:
      print "BOX IN UPPER WALL!!!!!!!!!!!!!"
      box.y = 2*box.size - box.y
      box.angle = math.pi - box.angle
      box.speed *= self.elasticity
      box.free = False

    box.crossedLimits = self.checkCrossedLimits(box)

  def checkCrossedLimits(self, box):
    """ Check whether a box center has hit the limits of the environment, defined in self.env_limits """
    crossed = False
    for i in range(0,self.env_limits.shape[0]):
      C = np.array([box.x,box.y])
      AC = C - self.env_limits[i][:2]
      BC = C - self.env_limits[i][2:]
      crossC = AC[0] * BC[1] - AC[1] * BC[0]
      if crossC < 0:
        crossed = True

    return crossed

  def findBox(self, x, y):
    """ Returns any box that occupies position x, y """
    for box in self.boxes:
      if math.hypot(box.x - x, box.y - y) <= box.size:
        return box
    return None

  def displayEnv(self):
    self.screen.fill(self.colour)  #empty screen
    pygame.draw.line(self.screen, (0, 255, 255), (0,0), (self.screenSize*math.sin(self.env_speed[0]), self.screenSize*math.cos(self.env_speed[0])), 1)

    for i in range(0,self.env_limits.shape[0]):
      pygame.draw.line(self.screen, (255, 255, 255), (int(self.res*self.env_limits[i][0]),int(self.res*self.env_limits[i][1])), (int(self.res*self.env_limits[i][2]),int(self.res*self.env_limits[i][3])), 1)

    self.displayText("Blue = Box", 6.5*self.res, 5*self.res)
    self.displayText("Green = KF Prediction", 6.5*self.res, 5.3*self.res)
    self.displayText("Red = Observation", 6.5*self.res, 5.6*self.res)

  def displayText(self, text, x, y, color = (255, 255, 255)):
    try:
        text = str(text)
        pygame.font.init()
        font = pygame.font.SysFont('Arial', 15)
        text = font.render(text, True, color)
        self.screen.blit(text, (x, y))
    except Exception, e:
        print 'Font Error'
        raise e

  def displayBox(self, box):
    print("displayBox pos: (%s , %s)" % (box.x,box.y))
    pygame.draw.circle(self.screen, box.colour, (int(self.res*box.x), int(self.res*box.y)), int(self.res*box.size), 3)

  def displayBoxRect(self, box):
    pygame.draw.rect(self.screen, box.colour, pygame.Rect((self.res*(box.x - 0.5*box.sizeX), self.res*(box.y - 0.5*box.sizeY)), (self.res*box.sizeX, self.res*box.sizeY)), 1)

  def displayBoxPrediction(self, box):
    print("displayBoxPrediction pos: %s" % box.kalmanFilter.states_means[-1,0:2])
    pygame.draw.circle(self.screen, box.colourKF, (int(self.res*box.kalmanFilter.states_means[-1,0]), int(self.res*box.kalmanFilter.states_means[-1,1])), int(self.res*box.size), 2)

  def displayBoxObservation(self, box):
    pygame.draw.circle(self.screen, box.colourObs, (int(self.res*box.kalmanFilter.observations[-1,0]), int(self.res*box.kalmanFilter.observations[-1,1])), int(self.res*box.size), 1)


  def displaypredictedMeans(self,box):
    for i in range(0,box.means.shape[0]):
      pygame.draw.circle(self.screen, (250,250,0), (int(self.res*box.means[i,0]), int(self.res*box.means[i,1])), int(self.res*box.size), 1)