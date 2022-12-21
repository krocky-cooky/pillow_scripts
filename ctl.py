import os,sys
import serial
import cv2 
import numpy as np 
aruco = cv2.aruco

BACKGROUND_LOOPCNT = 5
SERIAL_PORT = "COM3"


class PillowDevice(object):
    def __init__(
        self,
        port,
        rate = 115200,
    ):
        self.serial = serial.Serial(port,rate)
        self.current_position = "center"
        self.pos2step = {
            "left": 4800,
            "right": 0,
            "center": 2400
        }
    
    def move(self,position):
        if not position in self.pos2step.keys():
            print("there is no matching key")
            return
        target_step = self.pos2step[position]
        step = target_step - self.pos2step[self.current_position]
        if step == 0:
            print("the same position")
            return
        self.send_command(
            step = step,
        )
        self.current_position = position
    
    def send_command(self,step):
        command = f"c {step}"
        self.serial.write(command.encode())
    
    def lift_up(self):
        command = "w 15000"
        self.serial.write(command.encode())

    def lift_down(self):
        command = "s 15000"
        self.serial.write(command.encode())


def get_marker_center(corners, ids):
    ret = {}
    if ids is not None:
        for corner,idx in zip(corners,ids):
            center = corner[0].mean(axis = 0)
            ret[idx[0]] = center 
    return ret




def main():
    cap = cv2.VideoCapture(1)
    background_frame = None
    fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    device = PillowDevice(port = SERIAL_PORT)
    move_flag = False
    projection_ratio_buff = np.array([0.5 for i in range(5)])

    while True:
        ret, frame = cap.read()
        key = cv2.waitKey(100)
        if key == 27: #esc
            break
        elif key == 13: #enter
            background_frame = frame
        elif key == 32: #space
            move_flag = True
        elif key == 91: #up
            device.lift_up()
        elif key == 93: #down
            device.lift_down()
        
        #背景の適用
        if background_frame is not None:
            for _ in range(BACKGROUND_LOOPCNT):
                fgbg.apply(background_frame)
        subtracted_frame = fgbg.apply(frame)

        #マーカー検出
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame,dictionary)
        center_dict = get_marker_center(corners,ids)

        if 0 in center_dict.keys() and 1 in center_dict.keys():
            line_frame = np.zeros_like(subtracted_frame)

            #線の描画
            cv2.line(
                line_frame,
                tuple(center_dict[0].astype(np.int32)),
                tuple(center_dict[1].astype(np.int32)),
                255,
                thickness = 20,
            )

            contour_frame = np.zeros_like(subtracted_frame)
            contours, hierarchy = cv2.findContours(
                subtracted_frame,
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(
                contour_frame,
                contours,
                -1,
                255,
                -1
            )

            target_frame = np.where((contour_frame == 255) & (line_frame == 255), 255, 0)
            contours, hierarchy = cv2.findContours(
                target_frame.astype(np.uint8),
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            contours = sorted(contours,key = lambda contour: -cv2.contourArea(contour))
            output_area = np.zeros_like(subtracted_frame)
            cv2.drawContours(
                output_area,
                contours[:2],
                -1,
                255,
                -1
            )
            cv2.imshow("output_area",output_area.astype(np.uint8))

            if np.any(target_frame == 255):
                #重心の算出
                M = cv2.moments(output_area.astype(np.uint8))
                if M["m00"] != 0:
                    x,y= int(M["m10"]/M["m00"]) , int(M["m01"]/M["m00"])
                    cv2.circle(frame, (x,y), 10, 255, -1)
                    
                    #枕に対する位置を算出
                    zero2one_vector = center_dict[1]-center_dict[0]
                    zero2one_vector_norm = zero2one_vector / np.linalg.norm(zero2one_vector)
                    zero2target_vector = np.array([x,y])-center_dict[0]
                    zero2target_projection = np.dot(zero2one_vector_norm,zero2target_vector)*zero2one_vector_norm
                    projection_ratio = np.linalg.norm(zero2target_projection) / np.linalg.norm(zero2one_vector)
                    projection_ratio_buff = np.append(projection_ratio_buff,projection_ratio)[1:] 

                    position = "uncear"
                    if np.all(projection_ratio_buff >= 0.67):
                        position = "left"
                    elif np.all(projection_ratio_buff <= 0.32):
                        position = "right"
                    elif np.all(projection_ratio_buff > 0.32) and np.all(projection_ratio_buff < 0.67):
                        position = "center"
                    

                    if move_flag:
                        if position != "unclear":
                            move_flag = False
                            device.move(position)

            
        
        cv2.imshow('frame',frame)
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()