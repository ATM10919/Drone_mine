
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
x={}
y={}
z={}

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


calib_path  = ""
#camera_matrix, camera_distortion = load_camera_calibration(calib_path)
#if camera_matrix is not None and camera_distortion is not None:
 #       print("Camera calibration data loaded successfully")
        # Proceed with the rest of the code
  #  else:
   #     print("Exiting...")
camera_matrix  = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion  = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')



#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0




def detect_markers():
#    time.sleep(2)
    # Initialize the camera
    cap = cv2.VideoCapture(-1)  # Use the default camera (change if needed)

    # Load the ArUco dictionary
    ARUCO_DICT = {
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
        
    }

    max_count = 0
    max_id = None
    marker_counts = {}
    j=0
    intrinsic_camera = np.array(((6.544049058541622799e+02,0.000000000000000000e+00,3.303444950017052975e+02), (0.000000000000000000e+00,6.588586261380071392e+02,2.369498437380653968e+02), (0, 0, 1)))
    distortion = np.array((6.644481757468594617e-03,4.951723638263357263e-01,3.232678306110137460e-04,-3.967100922106183390e-03,-2.219747198210956451e+00))
    marker_size=150
    matrix_coefficients=camera_matrix
    distortion_coefficients=camera_distortion
    

    while(j==0):
        # Capture a frame from the camera
        ret, frame = cap.read()

        if ret:        # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            time.sleep(5)
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for aruco_type, dictionary_id in ARUCO_DICT.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
            aruco_params = cv2.aruco.DetectorParameters()

            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if len(corners) > 0:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
#                    print(marker_id)
                    ret= cv2.aruco.estimatePoseSingleMarkers(corners[i],marker_size, matrix_coefficients, distortion_coefficients)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    markerPoints=ret[2]
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                    key=marker_id
                    if key in x:
                     # If the key exists, append the value to the list
                       x[key].append(round(tvec[0],2))
                    else:
                       x[key]=[round(tvec[0],2)]
                    if key in y:
                     # If the key exists, append the value to the list
                       y[key].append(round(tvec[1],2))
                    else:
                       y[key]=[round(tvec[1],2)]
                    if key in z:
                     # If the key exists, append the value to the list
                       z[key].append(round(tvec[2],2))
                    else:
                       z[key]=[round(tvec[2],2)]
        
                    #x[key]=tvec[0]
                    #y[key]=tvec[1]
                    #z[key]=tvec[2]
                    print(str_position)
                    
                    if marker_id in marker_counts:
                        marker_counts[marker_id] += 1
                    else:
                        marker_counts[marker_id] = 1

                    if marker_counts[marker_id] > max_count:
                        max_count = marker_counts[marker_id]
                        max_id = marker_id

                    print(f"Detected ArUco Marker {marker_id}")

                    # Draw the marker borders
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                j=j+1
        # Display the frame
        #cv2.imshow("ArUco Marker Detection", frame)
        
        #j=j+1

        # Press 'q' to exit
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #time.sleep(2)
    print(f"Maximum occurrence: Marker {max_id} ({max_count} occurrences)")
    print(x)
    print(y)
    print(z)
    X=x[max_id]
    Y=y[max_id]
    Z=z[max_id]
    sumx=0
    sumy=0
    sumz=0
    for i in range(len(X)):
        sumx=sumx+X[i]
        sumy=sumy+Y[i]
        sumz=sumz+Z[i]
    x_v=round(sumx/max_count,2)
    y_v=round(sumy/max_count,2)
    z_v=round(sumz/max_count,2)
   # print(max_id)
    if max_id==0:    
        return max_id,x_v,y_v,z_v
    else:
        return 0,0,0,0
    # Release the camera and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

#a=detect()
#b=int(a[0])
#print(type(b))
#print(a)
















