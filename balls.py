import cv2
import numpy as np

def detect_yellow_balls(frame):
    # Convert to HSV colour kaanke lighting change thava thi HSV ma aetlu farak naa pade
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Define HSV range for yellow colour (adjusted for lighting conditions)
    yellow1 = np.array([35, 100, 100])
    yellow2 = np.array([65, 100, 100])

    # Threshold the HSV image to get only yellow colours
    range = cv2.inRange(hsv, yellow1, yellow2)

    # Reduce noise using Gaussian Blur
    blur = cv2.GaussianBlur(range,(11,11),sigmaX=10,sigmaY=10)
    #sigma x ane aigma y aaju baaju nu circle na pixels ne use karine mean laine
    #wadhaare blur thay aena maate che
    cv2.imshow('Processed Mask (Debug)', blur)

    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1,#to control resolution in the accumulator
        minDist=30,
        param1=50,#for edges
        param2=50,#select circles
        minRadius=10,
        maxRadius=100
    )

    if circles is not None:
        circles = np.uint16(np.around(circles)) 
        #unsigned in etle che kaanke drawing circles waara function ma non negative values joie che
        for (x, y, r) in circles[0, :]:
            # Draw the outer circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 10)
            # Draw the center of the circle
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
    return frame


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    output_frame = detect_yellow_balls(frame)

    cv2.imshow('Yellow Ball Detection (q to exit)', output_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
#https://www.youtube.com/watch?v=sCsaP9wUAkg 
