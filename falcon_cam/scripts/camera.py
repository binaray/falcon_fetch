# running on python 3 - open cv 4.2.0
import numpy as np
import cv2 as cv
import csv
import time

cap = cv.VideoCapture(0)
#width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
#height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
width = cap.set(3,640)
height = cap.set(4,480)
# Define the codec and create VideoWriter object
codec = cv.VideoWriter_fourcc(*'H264')
out = cv.VideoWriter('/home/ubuntu/Videos/falcon/output.avi', codec, 30.0, (640,  480))

log_file = open('/home/ubuntu/Videos/falcon/output.csv', 'w')
file_writer = csv.writer(log_file)
start_time = time.time()
i = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    frame = cv.flip(frame, 0)
    # write the flipped frame
    out.write(frame)
    # write to csv file
    current_time = time.time()
    timestamp = current_time - start_time
    file_writer.writerow([timestamp])
    print("frame "+str(i)+" received")
    i+=1
    # show image feed
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# Release everything if job is finished
cap.release()
out.release()
cv.destroyAllWindows()
log_file.close()
