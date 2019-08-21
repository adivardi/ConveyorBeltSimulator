# #!/usr/bin/env python
# import rospy
# from boxes_pos_mapping.msg import ConveyerSimMessage
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
res = 100   # res pixels per meter
nLanes = 8
conv_speed = (math.pi/4,  1, 0.2)   # angle and speed of conveyer (mean and std)
perp_speed = (0, 1) # mean and std of speed perpendicular to conveyer


# def pub(t,boxes):
#   pub.sim_msg.time = t
#   pub.publisher.publish(pub.sim_msg)

#   # boxesArray =
#   # for box in boxes:

# pub.publisher = rospy.Publisher('conveyerSim', ConveyerSimMessage, queue_size=10)
# pub.sim_msg = ConveyerSimMessage()

def main():
  #init node
  # rospy.init_node('conveyerSim', anonymous=True)
  # rate = rospy.Rate(freq) # 50hz

  # create environment
  env = sim.Environment((width, height), res, dt, nLanes, conv_speed, perp_speed, np.array([[11,0,0,9]]))
  env.addBoxes(1)

  t = 0
  running = True
  while running:# and (not rospy.is_shutdown()):
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False

    if (t % 1000 == 0):
      env.addBoxes(1)

    env.moveBoxes()
    env.displayEnv()

    for box in env.boxes:
      env.displayBox(box)
      env.displayBoxRect(box)

    pygame.display.flip() # show screen

    # publish boxes
    # pub(t, env.boxes)

    # rate.sleep()
    time.sleep(dt/1000.0)

    t += dt


if __name__ == "__main__":
  # try:
  main()
  # except rospy.ROSInterruptException:
    # pass
