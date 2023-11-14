import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os
import time

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

def dispColor(img, x, y):
    print("Color", x, y)
    s = ""
    for j in range(min(x[0], x[1]), max(x[0],x[1])+1,15):
      s += "    " + str(j) + "      "
    for i in range(min(y[0],y[1]), max(y[0],y[1])+1,5):
      s += "\n" + str(i) + " "
      for j in range(min(x[0], x[1]), max(x[0],x[1]),15):
         s += str(img[i][j]) + " "
    print(s)
    with open('coneData.txt', 'w') as f:
      f.write(s)   

def findYellow(image):
    # Create a mask for the yellow color
   lower_yellow = np.array([20, 100, 100])
   upper_yellow = np.array([30, 255, 255])
   # yellowLower = (22, 93, 0)
   # yellowUpper = (45, 255, 255)
   lower_yellow = np.array([0, 38, 100])
   upper_yellow = np.array([100,160, 200])   
   mask = cv2.inRange(image, lower_yellow, upper_yellow)
   # Apply the mask to the image
   filtered_image = cv2.bitwise_and(image, image, mask=mask)
   # Display the filtered image
   cv2.imshow('Filtered Image', filtered_image)

def onMouse(event, x, y, flags, param):
    global mouseResult
    if event == cv2.EVENT_LBUTTONDOWN:
        d = depth_frame.get_distance(x,y)
        s = str(color_image[y][x])
        mouseResult = {"x": x, "y": y, "time": time.time(), "s":"(%d,%d) d:%.2f %s" % (x, y, d * 100, s)}
        print(mouseResult['s'])

def label(cv2_im, obj, bbox):
   global lastX, lastY, count
   x0, y0 = int(bbox.xmin), int(bbox.ymin)
   x1, y1 = int(bbox.xmax), int(bbox.ymax)
   lastX = int((x0+x1)/2)
   lastY = int((y0+y1)/2)
   percent = int(100 * obj.score)
   d = depth_frame.get_distance(lastX, lastY) * 100
   label = "%2d%% %s (%3d,%3d) %5.2f cm" % (percent, labels.get(obj.id, obj.id), lastX, lastY, d)
   cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
   cv2_im = cv2.putText(
      cv2_im, label, (5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
   )
   if count == 10:
      dispColor(cv2_im, (x0, x1), (y0, y1))
   count += 1
   return cv2_im

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
       bbox = obj.bbox.scale(scale_x, scale_y)
       cv2_img = label(cv2_im, obj, bbox)      
    return cv2_img

# Global variables
lastX = 0
lastY = 0
count = 0
mouseResult = {}
raspi = True
# Set varaibles for model processing
model = os.path.join(".", "conesandcubes_b1.tflite")
labels = os.path.join(".", "labels.txt")
top_k = 1 # Number of objects to find
threshold = 0.1

# Load the model to find the obkect
print("Loading {} with {} labels.".format(model, labels))
interpreter = make_interpreter(model)
interpreter.allocate_tensors()
labels = read_label_file(labels)
inference_size = input_size(interpreter)
print("Tensor loaded")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == "RGB Camera":
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
    
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        if raspi:
            color_image = np.asanyarray(color_frame.get_data())
            cv2_im_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
            run_inference(interpreter, cv2_im_rgb.tobytes())
            objs = get_objects(interpreter, threshold)[: top_k]
            #bbox = obj.bbox.scale(scale_x, scale_y)
            height, width, channels = color_image.shape
            scale_x, scale_y = width / inference_size[0], height / inference_size[1]
            for obj in objs:
               bbox = obj.bbox.scale(scale_x, scale_y)
               x0, y0 = int(bbox.xmin), int(bbox.ymin)
               x1, y1 = int(bbox.xmax), int(bbox.ymax)
               lastX = int((x0+x1)/2)
               lastY = int((y0+y1)/2)
               percent = int(100 * obj.score)
               d = depth_frame.get_distance(lastX, lastY) * 100
               label = labels.get(obj.id, obj.id)
               print("%d%% %s (%d,%d) %.2f" % (percent, label, lastX, lastY, d))
            continue
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(
                color_image,
                dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                interpolation=cv2.INTER_AREA,
            )
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        cv2_im_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, threshold)[: top_k]
        cv2_im = append_objs_to_img(color_image, inference_size, objs, labels)
        cv2.circle(color_image, (lastX, lastY), 5, (255, 255, 255), -1)
        cv2.circle(depth_colormap, (lastX, lastY), 5, (255, 255, 255), -1)
        
        if len(mouseResult) >= 4: # and mouseResult['time'] + 1 < time.time():
            cv2.circle(color_image, (mouseResult['x'], mouseResult['y']), 5, (0, 255, 0), -1)
            cv2.circle(depth_colormap, (mouseResult['x'], mouseResult['y']), 5, (0, 255, 0), -1)
            cv2.putText(color_image, mouseResult['s'], (5, 30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
        findYellow(color_image)
        cv2.imshow("Color", color_image)
        cv2.imshow("Depth", depth_colormap)
        cv2.setMouseCallback("Color", onMouse)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == 113:
            break
finally:
    # Stop streaming
    pipeline.stop()
