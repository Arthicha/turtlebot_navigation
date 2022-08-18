import numpy as np
import cv2 as cv
import torch
import sys

# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

cap = cv.VideoCapture(2)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    # Display the resulting frame
    results = model(frame)
    person = results.pandas().xyxy["name"=="person"].to_numpy()
    if person.shape[0] != 0:
        cx = 0.5*(person[:,0]+person[:,2])
        cy = 0.5*(person[:,1]+person[:,3])
        cxy = np.stack([cx,cy])
        try:
            for i in range(person.shape[0]):
                frame = cv.circle(frame, (int(cxy[0,i]),int(cxy[1,i])), 2, (255,0,0), 10)
        except:
            pass
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()