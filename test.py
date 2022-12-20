import os,sys 
import cv2 
import numpy as np

if __name__ == "__main__":
    while True:
        img = np.full((210, 425, 3), 128, dtype=np.uint8)
        img = cv2.resize(img, (500, 300))
        key = cv2.waitKey(100)
        print(key)
        if key == 27:
            break
        cv2.imshow("img",img)