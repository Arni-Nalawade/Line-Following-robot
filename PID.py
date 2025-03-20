import cv2 as cv
import numpy as np
import utility

curveList = []
avgVal = 10 #10 values in advance path

def getLaneCurve(img,display=2):

    #backup copies for functions
    imgCopy = img.copy()
    imgResult = img.copy()

    ##Part 1: Red line detection
    imgthres = utility.thresholding(img)
    
    ##Part 2: Perspective Warp
    hT,wT,c = img.shape
    points = utility.valTrackbars()
    imgWarp = utility.warpImg(imgthres,points,wT,hT)
    imgWarppoints = utility.drawPoints(imgCopy,points)

    ##Part 3: Curve Detection
    middlePoint,imgHist = utility.getHistogram(imgWarp,display=True,minPer=0.5,region=4)
    curveAveragePoint,imgHist = utility.getHistogram(imgWarp,display=True,minPer=0.9,region=1)
    curveRaw = curveAveragePoint-middlePoint

    ##Part4: Optimization
    curveList.append(curveRaw)
    if len(curveList)>avgVal:
        curveList.pop(0)
    curve = int(sum(curveList)/len(curveList))

    #Display for debug
    if display != 0:
        imgInvWarp = utility.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv.cvtColor(imgInvWarp, cv.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        #cv2.putText(imgResult, 'FPS ' + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 50, 50), 3);
    if display == 2:
        imgStacked = utility.stackImages(0.7, ([img, imgWarppoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        cv.imshow('ImageStack', imgStacked)
    elif display == 1:
        cv.imshow('Result', imgResult)

    #Display robot processing feed view
    #cv.imshow('Thres',imgthres)
    #cv.imshow('Warp',imgWarp)
    #cv.imshow('Draw',imgWarppoints)
    #cv.imshow('Histogram',imgHist)
    
    return curve

if __name__ == '__main__':
    cap=cv.VideoCapture('vid1.mp4') #For test video feed
    #cap=cv.VideoCapture(0) #For webcam feed

    #Acertain trackpoints using trackbars
    initialTrackBarVals = [171,80,96,214]
    utility.initializeTrackbars(initialTrackBarVals)

    #Continuous camera feed
    frameCounter=0
    while True:
        #Looping video feed
        frameCounter += 1
        if cap.get(cv.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0

        success,img = cap.read()
        img = cv.resize(img,(480,240))
        curve = getLaneCurve(img,display=2) #0=run,1/2=debug
        print(curve)
        #cv.imshow('vid',img)
        cv.waitKey(1)