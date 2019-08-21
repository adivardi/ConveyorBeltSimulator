import pygame
import SimulationBoxes as sim
import math
import time
import numpy as np
from IPython import embed

# --- simulation constants ---
freq = 10         # in Hz
dt = 1000.0/freq  # in ms
(width, height) = (8, 6)  # size of the environment (in m)
res = 150   # res pixels per meter
nLanes = 4
conv_speed = (math.pi/4,  1, 0.5)   # angle and speed of conveyer (mean and std)
perp_speed = (0, 1) # mean and std of speed perpendicular to conveyer
new_box_interval = 5000 # in ms

def main():
  # create environment and add 1st box
  env = sim.Environment((width, height), res, dt/1000.0, nLanes, conv_speed, perp_speed, np.array([[11,0,0,9]]))
  # env.addBoxes(1)

  try:
    input("Press enter to start")
  except SyntaxError:
    pass

  t = 0
  running = True
  while running:# and (not rospy.is_shutdown()):
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False

    # display environment
    env.displayEnv()

    # add box every second
    if (t % new_box_interval == 0):
      env.addBoxes(1)

    # update boxes and update Kalman filters
    env.moveBoxes()

    # display boxes
    for box in env.boxes:
      env.displayBox(box)
      env.displayBoxRect(box)
      env.displayBoxPrediction(box)
      env.displayBoxObservation(box)
      # env.displaypredictedMeans(box)

    pygame.display.flip() # show screen

    time.sleep(dt/1000.0)

    t += dt

    # try:
    #   input("Press enter to continue")
    # except SyntaxError:
    #   pass


if __name__ == "__main__":
  main()
