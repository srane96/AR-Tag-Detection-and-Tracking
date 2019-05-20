
# coding: utf-8

# In[ ]:


# ======================================================================================================================================================================= #
#-------------> Project 01 <---------------#
# ======================================================================================================================================================================= #
# Course    :-> ENPM673 - Perception for Autonomous Robots
# Date      :-> 27 February 2019
# Authors   :-> Siddhesh(UID: 116147286), Sudharsan(UID: 116298636), Niket Shah(UID: 116345156), 
# ======================================================================================================================================================================= #

# ======================================================================================================================================================================= #
# Import Section for Importing library.
# ======================================================================================================================================================================= #
import cv2 as cv
import numpy as np
import math
import copy
import time
import sys

# ==================================================================================================================================================================== #
# Import Video files and required Image
# ==================================================================================================================================================================== #
tag = cv.VideoCapture('Input Sequences/Tag0.mp4')           # Enter the file location Address of the video file
img = cv.imread('Reference Images/Lena.png')                # Enter the template image file location address. 
img_grayscale = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
(img_x, img_y, ch) = img.shape
ref_x_points, ref_y_points = 400, 400

threeDimAxis = np.float32([[0, 0, 0], [0, 500, 0], [500, 500, 0], [500, 0, 0], [0, 0, -300], [0, 500, -300], [500, 500, -300], [500, 0, -300]])  # Change these values to
# Function Definitions:  Calculating the H matrix
# ======================================================================================================================================================================= #
def getHMatrix(orientation, camera_x, camera_y):
    # ================================================================================================================================================================ #
    # World coordinates - come from video.
    # ================================================================================================================================================================ #
    if orientation == 'bR':
        xw1, yw1 = approx[0][0][0], approx[0][0][1]
        xw2, yw2 = approx[1][0][0], approx[1][0][1]
        xw3, yw3 = approx[2][0][0], approx[2][0][1]
        xw4, yw4 = approx[3][0][0], approx[3][0][1]
    elif orientation == 'bL':
        xw1, yw1 = approx[1][0][0], approx[1][0][1]
        xw2, yw2 = approx[2][0][0], approx[2][0][1]
        xw3, yw3 = approx[3][0][0], approx[3][0][1]
        xw4, yw4 = approx[0][0][0], approx[0][0][1]
    elif orientation == 'tL':
        xw1, yw1 = approx[2][0][0], approx[2][0][1]
        xw2, yw2 = approx[3][0][0], approx[3][0][1]
        xw3, yw3 = approx[0][0][0], approx[0][0][1]
        xw4, yw4 = approx[1][0][0], approx[1][0][1]
    elif orientation == 'tR':
        xw1, yw1 = approx[3][0][0], approx[3][0][1]
        xw2, yw2 = approx[0][0][0], approx[0][0][1]
        xw3, yw3 = approx[1][0][0], approx[1][0][1]
        xw4, yw4 = approx[2][0][0], approx[2][0][1]

    # ================================================================================================================================================================ #
    # Camera coordinates - our target frame.
    # ================================================================================================================================================================ #
    xc1, yc1 = 0, 0
    xc2, yc2 = camera_x, 0
    xc3, yc3 = camera_x, camera_y
    xc4, yc4 = 0, camera_y

    # ================================================================================================================================================================ #
    # Calculating the A Matrix
    # ================================================================================================================================================================ #
    A = [[xw1, yw1, 1, 0, 0, 0, -xc1*xw1, -xc1*yw1, -xc1],
         [0, 0, 0, xw1, yw1, 1, -yc1*xw1, -yc1*yw1, -yc1],
         [xw2, yw2, 1, 0, 0, 0, -xc2*xw2, -xc2*yw2, -xc2],
         [0, 0, 0, xw2, yw2, 1, -yc2*xw2, -yc2*yw2, -yc2],
         [xw3, yw3, 1, 0, 0, 0, -xc3*xw3, -xc3*yw3, -xc3],
         [0, 0, 0, xw3, yw3, 1, -yc3*xw3, -yc3*yw3, -yc3],
         [xw4, yw4, 1, 0, 0, 0, -xc4*xw4, -xc4*yw4, -xc4],
         [0, 0, 0, xw4, yw4, 1, -yc4*xw4, -yc4*yw4, -yc4]]

    # ================================================================================================================================================================ #
    # Computing Singular Value Decomposition & Homography Matrix - using vh matrix's last row.
    # ================================================================================================================================================================ #
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    H = np.array(vh[8, :]/vh[8, 8]).reshape((-1, 3))
    inv_H = np.linalg.inv(H)
    return (H, inv_H)

# ======================================================================================================================================================================= #
# Function Definitions:  get the orientation of the tag with respect to video frame
# ======================================================================================================================================================================= #
def getOrientation(input_image):
    org = {}
    lst = []
    sum = 0
    # ================================================================================================================================================================ #
    # checking for the orientation of the tag's white corner within the four points marked in contour
    # ================================================================================================================================================================ #
    #Top Left Orientation: TL
    for i in range(100, 151):
        for j in range(100, 151):
            sum += input_image[i, j]
    tL = sum/2500
    org[tL] = 'tL'
    lst.append(tL)
    # ================================================================================================================================================================ #
    #Bottom Right Orientation: BR
    sum = 0
    for i in range(250, 301):
        for j in range(250, 301):
            sum += input_image[i, j]
    bR = sum/2500
    org[bR] = 'bR'
    lst.append(bR)
    # ================================================================================================================================================================ #
    #Bottom Left Orientation: BL
    sum = 0
    for i in range(250, 301):
        for j in range(100, 151):
            sum += input_image[i, j]
    bL = sum/2500
    org[bL] = 'bL'
    lst.append(bL)
    # ================================================================================================================================================================ #
    #Top Rigth  Orientation: TR
    sum = 0
    for i in range(100, 151):
        for j in range(250, 301):
            sum += input_image[i, j]
    tR = sum/2500
    org[tR] = 'tR'
    lst.append(tR)
    return org[max(lst)]
# ======================================================================================================================================================================= #
# Function Definitions:  get the tag ID
# ======================================================================================================================================================================= #
def getTagId(input_image, orientationID):
    id = ''
    if orientationID == 'tL':
        keys = ['BL', 'BR', 'TR', 'TL']
    elif orientationID == 'bL':
        keys = ['TL', 'BL', 'BR', 'TR']
    elif orientationID == 'bR':
        keys = ['TR', 'TL', 'BL', 'BR']
    elif orientationID == 'tR':
        keys = ['BR', 'TR', 'TL', 'BL']
    orientation = {'BL': [150, 200, 200, 250], 'BR': [200, 250, 200, 250], 'TR': [200, 250, 150, 200], 'TL': [150, 200, 150, 200], }
    # loop through each orientation to find out the tag ID
    sum = 0
    for o in range(0, 4):
        for i in range(orientation[keys[o]][0], orientation[keys[o]][1]):
            for j in range(orientation[keys[o]][2], orientation[keys[o]][3]):
                sum += input_image[i, j]
        bL = sum/2500
        if bL > 180:
            id = id + '1'
        else:
            id = id + '0'
        sum = 0
    return id

# ======================================================================================================================================================================= #
# Function Definitions:  get the K,[R|t] matrices  of the tag with respect to video frame
# ======================================================================================================================================================================= #
def getKRTMatrix(H, inv_H):
    K_mat = np.array([[1406.08415449821, 0, 0], [2.20679787308599, 1417.99930662800, 0], [1014.13643417416, 566.347754321696, 1]]).T
    inv_K_mat = np.linalg.inv(K_mat)
    B_mat = np.matmul(inv_K_mat, inv_H)
    b1 = B_mat[:, 0].reshape(3, 1)
    b2 = B_mat[:, 1].reshape(3, 1)
    r3 = np.cross(B_mat[:, 0], B_mat[:, 1])
    b3 = B_mat[:, 2].reshape(3, 1)
    scalar = 2/(np.linalg.norm(inv_K_mat.dot(b1))+np.linalg.norm(inv_K_mat.dot(b2)))
    t = scalar*b3
    r1 = scalar*b1
    r2 = scalar*b2
    r3 = (r3 * scalar * scalar).reshape(3, 1)
    R_mat = np.concatenate((r1, r2, r3), axis=1)
    return R_mat, t, K_mat

# ======================================================================================================================================================================= #
# Function Definitions:  Draw the 3D structure, having the borders of a cuboid
# ======================================================================================================================================================================= #
def draw3D(frame, threeDimPoints):
    threeDimPoints = np.int32(threeDimPoints).reshape(-1, 2)
    frame = cv.drawContours(frame, [threeDimPoints[:4]], -1, (0, 255, 0), 3)   # Ground plane
    for i, j in zip(range(4), range(4, 8)):                                    # Z Axis planes
        frame = cv.line(frame, tuple(threeDimPoints[i]), tuple(threeDimPoints[j]), (0, 0, 255), 3)
    frame = cv.drawContours(frame, [threeDimPoints[4:]], -1, (255, 0, 0), 3)   # Top plane
    return frame

# ======================================================================================================================================================================= #
# change the scaling of the 3D structure
while True:
    # Reading each key frame ( kf_ ) from the video
    ret, key_frame = tag.read()
    if ret == None or ret == False: break
    # ================================================================================================================================================================ #
    # For better Contouring converting each key frame into grayscale and apply thresholding.
    # ================================================================================================================================================================ #
    # storing a copy of original key frame for future use
    kf_original = key_frame.copy()
    # Extracting Video Frame Size
    rows, cols, chs = key_frame.shape
    kf_grayscale = cv.cvtColor(key_frame, cv.COLOR_BGR2GRAY)
    _, kf_threshold = cv.threshold(kf_grayscale, 200, 255, 0)
    _, contours, heirarchy = cv.findContours(kf_threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # ================================================================================================================================================================ #
    # shortlisting Contour candidates with area approximately equal to the Tag area among the multiple cntours detected.
    # ================================================================================================================================================================ #
    # To keep track of contour with maximum area
    max_area_contour = np.zeros((1, 1, 2), dtype=int)
    for contour in contours:
        peri = cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, 0.01 * peri, True)
        area = cv.contourArea(contour)
        if area > 2000 and area < 22600:
            max_area_contour = contour
            if len(approx) == 4:                            # Filtering Contours with 4 corners
                # Draw the selected Contour matching the criteria fixed
                cv.drawContours(key_frame, [contour], 0, (0, 0, 225), 2)
                # ==================================================================================================================================================== #
                # Computing H matrix to project tag
                H_tag, inv_H_tag = getHMatrix('bR', ref_x_points, ref_y_points)
                # ==================================================================================================================================================== #
                # Computing Keyframe
                tag_unwarpped = cv.warpPerspective(key_frame, H_tag, (ref_x_points, ref_y_points))
                tag_unwarpped_grayscale = cv.cvtColor(tag_unwarpped, cv.COLOR_BGR2GRAY)
                _, tag_unwarpped_threshold = cv.threshold(tag_unwarpped_grayscale, 200, 255, cv.THRESH_BINARY)
                # ==================================================================================================================================================== #
                # Computing the orientation of the tag using thresholded unwarpped tag image 
                orientationID = getOrientation(tag_unwarpped_threshold)
                font = cv.FONT_HERSHEY_SIMPLEX
                # ==================================================================================================================================================== #
                # Computing the tag ID of the thresholded unwarpped tag image (This is done only once)
                # ==================================================================================================================================================== #
                
                tag_id = getTagId(tag_unwarpped_threshold, orientationID)
                tx1, ty1 = approx[0][0][0], approx[0][0][1]
                cv.putText(kf_original, "Tag ID: " + tag_id, (tx1-50, ty1-50), font, 1, (0, 0, 225), 2, cv.LINE_AA)
                # ==================================================================================================================================================== #
                # Using the orientation information obtained above lets recalculate the H and inv_H matrices
                H_image, inv_H_image = getHMatrix(orientationID, img_x, img_y)
                # ==================================================================================================================================================== #
                # Preparing the image and projecting into the tag using the recalculated Homography matrix
                warpped_image = cv.warpPerspective(img, inv_H_image, (cols, rows))
                warpped_image_grayscale = cv.cvtColor(warpped_image, cv.COLOR_BGR2GRAY)
                # ==================================================================================================================================================== #
                # Creating Mask and merging the key frame with the warpped Image using bitwise_and and add opencv functions
                _, warpped_image_threshold = cv.threshold(warpped_image_grayscale, 0, 250, cv.THRESH_BINARY_INV)
                kf_slotted = cv.bitwise_and(kf_original, kf_original, mask=warpped_image_threshold)
                result = cv.add(kf_slotted, warpped_image)
                # ==================================================================================================================================================== #
                # Computing the Projection Matrix for placing 3D objects
                R_mat, t, K_mat = getKRTMatrix(H_image, inv_H_image)
                threeDimPoints, jacobian = cv.projectPoints(threeDimAxis,R_mat,t,K_mat,np.zeros((1,4)))
                threeDframe = draw3D(key_frame, threeDimPoints)
    
    # ================================================================================================================================================================ #
    # Display the Original Keyframe and Final Result
    cv.imshow("Original Key Frame", kf_original)
    cv.imshow("Result", result)
    cv.imshow("3D Projection Cube", threeDframe)
    key = cv.waitKey(1)
    if key == 27:
        break
    
tag.release()
cv.destroyAllWindows()

