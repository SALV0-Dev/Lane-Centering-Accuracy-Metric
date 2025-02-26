
import cv2 
import cv2.aruco as aruco
import numpy as np

def calibratedCenter(raw_center, middle_x, middle_y):
    new_center_x = None
    new_center_y = None

    sum_vector_component_y = int(abs(raw_center[1] - 240)*0.895) #0.9075 # 0.92 -1
    sum_vector_component_x = int(abs(raw_center[0] - 320)*0.895) #0.9075 # 0.92 -1

    if raw_center[1] >= middle_y:
        new_center_y = middle_y + sum_vector_component_y
    else:
        new_center_y = middle_y - sum_vector_component_y


    
    if raw_center[0] > middle_x:
        new_center_x = middle_x + sum_vector_component_x
    else:
        new_center_x = middle_x - sum_vector_component_x

    

    return (new_center_x, new_center_y)
    

def set_camera_settings(camera_index=0, fps=30, exposure_time=1/60):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    cap.set(cv2.CAP_PROP_FPS, fps)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    exposure_ms = exposure_time * 1000
    
    # set the exposure time, fixes camera flicekring
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure_ms)
    
    print(f"Camera FPS set to: {cap.get(cv2.CAP_PROP_FPS)}")
    print(f"Camera Exposure set to: {cap.get(cv2.CAP_PROP_EXPOSURE)} ms")
    
    detector = cv2.aruco.ArucoDetector(dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100))

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        if ids is not None:
            for corner, ids in zip(corners, ids):

                # draw the marker squar
                cv2.polylines(frame, [np.int32(corner)], True, (0, 255, 0), 2)
                
                # manually calculate the center of the marker
                corner = corner.reshape((4, 2))
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                center = (center_x, center_y)

                center = calibratedCenter(center, frame_width/2, frame_height/2)

                if center[0] is not None and center[1] is not None:
                    center = (int(center[0]), int(center[1]))
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    print(center)
                       
        cv2.imshow('Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    light_frequency = 60  # set to 50 if lights operate at 50Hz
    camera_index = 0      
    
    if light_frequency == 60:
        fps = 30  # or 60
        exposure_time = 1 / 60 
    else:
        fps = 25  # or 50
        exposure_time = 1 / 50  

    # Set camera settings with defined variables
    set_camera_settings(camera_index=camera_index, fps=fps, exposure_time=exposure_time)

