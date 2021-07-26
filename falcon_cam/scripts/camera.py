# running on python 3 - open cv 4.2.0
import numpy as np
import cv2 as cv
import csv
import time

cap = cv.VideoCapture(4)
width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
# Define the codec and create VideoWriter object
codec = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter('/home/ubuntu/Videos/falcon/output.avi', codec, 30.0, (width,  height))

log_file = open('/home/ubuntu/Videos/falcon/output.csv', 'w')
file_writer = csv.writer(log_file)
start_time = time.time()

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
    # show image feed
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# Release everything if job is finished
cap.release()
out.release()
cv.destroyAllWindows()
log_file.close()
