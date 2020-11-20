#!/usr/bin/env python3
import time
import numpy as np
import cv2 as cv
import sys
from matplotlib import pyplot as plt
import pyrealsense2
import rospy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

#CV SETUP
print("CV INIT")
print("Version: " + sys.version)
sift = cv.SIFT_create()
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

# Source image read
sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])


# img2 = cv.imread('machine.jpg',1) # trainImage
# img2 = cv.resize(img2, (900, 1200))

# Video Read/Write
video = cv.VideoCapture('Video.mp4')
fourcc = cv.VideoWriter_fourcc(*'XVID')
write = cv.VideoWriter('output.avi',fourcc, 20.0, (1800,1200))


class Vending:
    def __init__(self):
        self.letter = "a"
        self.number = "1"
        self.keypad = cv.filter2D(cv.imread('keypad_crop.png', 1), -1, sharpen_kernel)
        self.black = np.zeros((300, 200, 3), np.uint8)
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.intrinsic_callback)
        rospy.Subscriber("/camera/convert/depth", Image, self.depth_callback)
        rospy.Subscriber("/camera/convert/image", Image, self.image_callback)
        self.keys = keys = {
                        "a":((0/6, 1/6), (0/3, 1/3)),
                        "b":((1/6, 2/6), (0/3, 1/3)),
                        "c":((2/6, 3/6), (0/3, 1/3)),
                        "d":((3/6, 4/6), (0/3, 1/3)),
                        "e":((4/6, 5/6), (0/3, 1/3)),
                        "f":((5/6, 6/6), (0/3, 1/3)),
                        "1":((0/6, 1/6), (1/3, 2/3)),
                        "3":((1/6, 2/6), (1/3, 2/3)),
                        "5":((2/6, 3/6), (1/3, 2/3)),
                        "7":((3/6, 4/6), (1/3, 2/3)),
                        "9":((4/6, 5/6), (1/3, 2/3)),
                        "2":((0/6, 1/6), (2/3, 3/3)),
                        "4":((1/6, 2/6), (2/3, 3/3)),
                        "6":((2/6, 3/6), (2/3, 3/3)),
                        "8":((3/6, 4/6), (2/3, 3/3)),
                        "0":((4/6, 5/6), (2/3, 3/3))
                    }
        # self.key_files={}
        # for x in range(0,10):
        #     self.key_files[str(x)]=cv.imread(str(x)+".jpg", 1)
        # for x in range(ord('a'), ord('g')):
        #     self.key_files[chr(x)]=cv.imread(chr(x)+".jpg", 1)
        # self.contour_keys()


    def intrinsic_callback(self,data):
        self.depth_intrins = data
    
    def depth_callback(self, data):
        self.depth_data = np.array(list(data.data)).reshape((720,1280)).astype("uint8")

    def image_callback(self,data):
        print("Processing frame")
        self.cv_image = data.data
        self.cv_image = np.array(list(self.cv_image)).reshape((720,1280,3)).astype("uint8")
        cv.imshow('frame', self.cv_image)
        cv.waitKey(1)
        success, dst_keypad, output = self.match(self.cv_image, self.keypad, 10)
        if success:
            # cv.imshow('frame', output)
            # cv.waitKey(1)
            letter_coord, self.cv_image = self.key_match(dst_keypad, output, self.cv_image, self.letter)
            number_cord, self.cv_image = self.key_match(dst_keypad, output, self.cv_image, self.letter)
            print("3d coordinates: ", self.map_2d_to_3d(letter_coord[0], letter_coord[1], self.depth_data, self.depth_intrins))
        # cv.imshow('frame', self.cv_image)
        # cv.waitKey(1)

    def match(self, frame, reference, match_count, output_matches=False):
        success = True
        MIN_MATCH_COUNT = match_count
        # cv.imshow('frame', frame)
        # cv.waitKey(1)
        # gray = cv.cvtColor(reference, cv.COLOR_BGR2GRAY) 
        # ret, reference = cv.threshold(gray, 0, 255, cv.THRESH_OTSU | cv.THRESH_BINARY_INV)
        # reference = cv.cvtColor(reference, cv.COLOR_GRAY2BGR)
        # cv.imshow('reference', reference)
        # cv.waitKey(0)
        kp1, des1 = sift.detectAndCompute(reference, None)
        kp2, des2 = sift.detectAndCompute(frame,None)
        

        matches = flann.knnMatch(des1,des2,k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        # matched_img = reference
        # matched_img = cv.drawMatches(reference,kp1,frame,kp2,good,None)
        # cv.imshow("RESULT KPS", matched_img)
        # cv.waitKey(0)
        print("NUM MATCHES: ", len(good))
        if len(good)>=MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h,w,d = reference.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # if MIN_MATCH_COUNT < 30:
            #     print(pts)
            # print("EM: ", M)
            dst = cv.perspectiveTransform(pts,M)
            dst[1][0][0] = dst[0][0][0]
            dst[2][0][0] = dst[3][0][0]
            dst[2][0][1] = dst[1][0][1]
            dst[3][0][1] = dst[0][0][1]
            dst = dst.astype(int)
            dim = [dst[0][0][0],dst[2][0][0],dst[0][0][1],dst[2][0][1]]
            # dim = [dst[2][0][0],dst[0][0][0],dst[2][0][1],dst[0][0][1]]
            print("DIM: ", dim)
            if output_matches:
                frame = cv.polylines(frame,[np.int32(dst)],True,255,3, cv.LINE_AA)
                # frame = cv.rectangle(frame, (dim[0], dim[2]), (dim[1], dim[3]), (255, 0, 0), 2)
                draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask, # draw only inliers
                            flags = 2)
                output = cv.drawMatches(reference,kp1,frame,kp2,good,None,**draw_params)
            else:
                output = frame[dim[2]:dim[3], dim[0]:dim[1]]
                # ratio = (x_2-x_1)/(y_2-y_1)
                # if ratio > 0.6 or ratio < 0.5:
                #     success = False
                #     print("Bad scale -{}- match rejected".format(ratio))
        else:
            dim = None
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
            if output_matches:
                matchesMask = None
                draw_params = dict(matchColor = (0,0,255), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask, # draw only inliers
                            flags = 2)
                output = cv.drawMatches(reference,kp1,frame,kp2,good,None,**draw_params)
                success = False
            else:
                output = self.black
                cv.putText(output, 'No Match', (20,150), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)
                success = False
        self.output = output
        return success, dim, output

    def contour_keys(self):
        self.key={}
        for x in self.key_files:
            image = self.key_files[x]
            image = cv.filter2D(image, -1, sharpen_kernel)
            self.key[x]=self.contour_image(image, True)
    
    def contour_image(self, image, blur=False):
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image = cv.resize(image, (600,800))
        if blur:
            image = cv.blur(image, (5,5))
        ret, thresh = cv.threshold(image, 127, 255,0)
        contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
        contours = self.sort_contours(contours)
        # return contours
        approx=[]
        for c in contours:
            accuracy= 0.05 * cv.arcLength(c, True)
            approx.append(cv.approxPolyDP(c,accuracy,True))
        # contoured = np.array(approx).reshape((-1,1,2)).astype(np.int32)
        return approx

    def sort_contours(self, contours):
        area = []
        for cont in contours:
            area.append(cv.contourArea(cont))
        return [x for _, x in sorted(zip(area, contours), key=lambda pair: pair[0])]

    def key_match(self, dst_keypad, keypad, machine, letter):
        width = keypad.shape[1]
        height = keypad.shape[0]
        dst_key=np.array([width*self.keys[letter][0][0], width*self.keys[letter][0][1], height*self.keys[letter][1][0], height*self.keys[letter][1][1]])
        dst_key=dst_key.astype(int)
        dim = list()
        for x,y in enumerate(dst_key):
            dim.append(y + dst_keypad[int(x/2)*2])
        cv.rectangle(machine, (dim[0], dim[2]), (dim[1], dim[3]), (0, 255, 0), 2)
        center = (dim[1] + dim[0])/2, (dim[3] + dim[2])/2
        center = int(center[0]), int(center[1])
        # print("Input Width: {}, Input Height: {}".format(width, height))
        # print("x_1: {}, x_2: {}, y_1: {}, y_2: {}".format(dim[0][0][0],dim[1],dim[2],dim[3])) 
        # cv.rectangle(image, (dim[0], dim[1]), (dim[2], dim[3]), (255, 0, 0), 2)
        return center, machine

    def key_contour(self, frame, character):
        frame_contours = self.contour_image(frame)
        minimum = (frame_contours[0], 200)
        for x in frame_contours[-18:]:
            ret = cv.matchShapes(x,character[-1],1,2)
            if ret < minimum[1]:
                minimum = (x, ret)
            white = np.ones((800,600))
            cv.drawContours(white, x, -1, (0, 255, 0), 1)
            # cv.imshow('cont', white)
            # cv.waitKey(0)
            # print( ret )
        print("RET: ", minimum[1])
        return minimum[0]

    def map_2d_to_3d(self, x, y, depth, intrins):
        # print("X: {}, Y: {}".format(x,y))
        depth = depth[x][y]
        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = intrins.width
        _intrinsics.height = intrins.height
        _intrinsics.ppx = intrins.K[2]
        _intrinsics.ppy = intrins.K[5]
        _intrinsics.fx = intrins.K[0]
        _intrinsics.fy = intrins.K[4]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in intrins.D]
        result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        #result[0]: right, result[1]: down, result[2]: forward
        return result[2], -result[0], -result[1]

if __name__ == "__main__":
    rospy.init_node('video')
    example = Vending()
    rospy.spin()