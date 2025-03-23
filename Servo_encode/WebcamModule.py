import cv2
 
cap=cv2.VideoCapture(1)
 
def getImg(display= False,size=[480,240]):
    _, img = cap.read()
    img = cv2.resize(img,(480,480))
    if display:
        cv2.imshow('IMG',img)
    return img
 
if __name__ == '__main__':
    while True:
        img = getImg(True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break