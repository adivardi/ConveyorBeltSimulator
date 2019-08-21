import numpy as np
from IPython import embed
from random import seed, uniform, randint
import cv2
import time

seed(2)

#___constants___
# batch_size = 1
dt = 100 #(in ms)

alpha = np.pi/4 #(45 deg)
cache_limits = np.array([11,0,0,9])

size_m = 0.3
size_s = 0.2

nLane = 10
v_axis_m = 1
v_axis_s = 0.3
v_perp_m = 0
v_perp_s = 1
w_m = 1 #0.25 #(15 deg)
w_s = 0.1

img_width = 800
img_height = 600
res = 100   # res pixels per meter

def generateBox(boxes):
  lane = randint(int(-0.5*nLane), int(0.5*nLane))
  x0 = lane*2*size_m
  y0 = 0
  sizeX = size_m + uniform(-size_s,size_s)
  sizeY = size_m + uniform(-size_s,size_s)
  box = np.array([[x0,y0,x0+sizeX,y0+sizeY]])
  boxes = np.append(boxes,box, axis=0)
  print("Generated Boxes : \n %s" %boxes)
  return boxes

def checkCrossedCache(box):
  crossed = False
  for i in [0,2]:
    C = box[i:i+2]
    AC = C - cache_limits[:2]
    BC = C - cache_limits[2:]
    crossC = AC[0] * BC[1] - AC[1] * BC[0]
    if crossC < 0:
      crossed = True

  if crossed:
    print("Box crossed the cache limits")
    return True
  else:
    return False

def moveBox(box):
  if checkCrossedCache(box) == False:
    # move along axis
    v_axis = v_axis_m + uniform(-v_axis_s, v_axis_s)
    d_axis = (dt/1000.0)*v_axis
    dx = d_axis*np.cos(alpha)
    dy = d_axis*np.sin(alpha)
    box[0] += dx; box[1] += dy; box[2] += dx; box[3] += dy;

    # move perpndicular to axis
    v_perp = v_perp_m + uniform(-v_perp_s, v_perp_s)
    d_perp = (dt/1000.0)*v_perp
    dx = -d_perp*np.sin(alpha)
    dy = d_perp*np.cos(alpha)
    box[0] += dx; box[1] += dy; box[2] += dx; box[3] += dy;

    # rotate box
    # box = rotateBox(box)  # not working properly when displaying because opencv only displays horizontal rectangles
  return box

def rotateBox(box):
  center = np.array([box[0]+box[2],box[1]+box[3]])/2
  w = w_m + uniform(-w_s, w_s)
  dw = (dt/1000.0)*w

  # rotate 1st corner
  box2 = np.array(box)
  box2[0] = center[0] + (np.cos(dw)*(box[0] - center[0])) - (np.sin(dw)*(box[1] - center[1]))
  box2[1] = center[1] + (np.sin(dw)*(box[0] - center[0])) + (np.cos(dw)*(box[1] - center[1]))
  # rotate 2nd corner
  box2[2] = center[0] + (np.cos(dw)*(box[2] - center[0])) - (np.sin(dw)*(box[3] - center[1]))
  box2[3] = center[1] + (np.sin(dw)*(box[2] - center[0])) + (np.cos(dw)*(box[3] - center[1]))
  return box2

def moveAll(boxes):
  for i in range(0,boxes.shape[0]):
    boxes[i] = moveBox(boxes[i])
  print("Moved boxes: \n %s" %boxes)
  return boxes

def drawBoxes(boxes):
  drawBoxes.image = np.zeros((img_height,img_width,3), np.uint8);
  for i in range(0,boxes.shape[0]):
    cv2.rectangle(drawBoxes.image, (int(res*boxes[i][0]),int(res*boxes[i][1])), (int(res*boxes[i][2]),int(res*boxes[i][3])), (255,0,0), 2)
  cv2.line(drawBoxes.image, (0, 0), (int(drawBoxes.hypotenuse*np.cos(alpha)), int(drawBoxes.hypotenuse*np.sin(alpha))),(0, 255, 255), 1)
  cv2.line(drawBoxes.image, (int(res*cache_limits[0]),int(res*cache_limits[1])), (int(res*cache_limits[2]),int(res*cache_limits[3])),(255, 255, 255), 1)

drawBoxes.image = np.zeros((img_height,img_width,3), np.uint8)
drawBoxes.hypotenuse = np.sqrt(img_width*img_width + img_height*img_height)

def main():
  t  = 0
  print ("Press any key to start and 'q' to exit")

  # generate first box
  boxes = np.zeros((1,4))
  boxes = generateBox(boxes)
  boxes = boxes[1:]
  print("Boxes : \n %s" %boxes)

  drawBoxes(boxes)
  cv2.imshow("Viz", drawBoxes.image)
  cv2.waitKey(0)

  while True:
    t += dt
    print("Time: %s" %(t%1000))

    boxes = moveAll(boxes)

    if (t % 1000 == 0):
      boxes = generateBox(boxes)

    drawBoxes(boxes)

    cv2.imshow("Viz", drawBoxes.image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
      break

    time.sleep(dt/1000.0)

if __name__ == "__main__":
  main()