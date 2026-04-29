import cv2
import numpy as np
import os
import glob

cam_images_folder_name = 'cam_3'
cam_images_folder_name_calibrated = f'{cam_images_folder_name}_c'

CHECKERBOARD = (5,6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objpoints = []
imgpoints = []

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

os.makedirs(cam_images_folder_name_calibrated, exist_ok=True)

images = glob.glob(f'./{cam_images_folder_name}/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        print(f"Found corners in {fname}")
    else:
        print(f"No corners found in {fname}")
    cv2.imwrite(cam_images_folder_name_calibrated + '/' + os.path.basename(fname), img)

h,w = img.shape[:2]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("\nCamera matrix:")
print(mtx)
print("\nDistortion coefficients:")
print(dist)
