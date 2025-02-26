import cv2
import numpy as np

def set_camera_settings_and_take_picture(camera_index=0, fps=30, exposure_time=1/60, output_file='processed_track.jpg'):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    exposure_ms = exposure_time * 1000
    
    # this fixed the flickering issues
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure_ms)
    
    print(f"Camera FPS set to: {cap.get(cv2.CAP_PROP_FPS)}")
    print(f"Camera Exposure set to: {cap.get(cv2.CAP_PROP_EXPOSURE)} ms")
    

    kernel_3x3 = np.ones((3, 3), np.uint8)
    ret, frame = cap.read()

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, frame = cv2.threshold(frame, 120, 255, cv2.THRESH_BINARY)

    # Pipeline for better track detection 
    frame= cv2.erode(frame, kernel_3x3, iterations=1)
    frame = cv2.dilate(frame, kernel_3x3, iterations=1)     
    frame= cv2.erode(frame, kernel_3x3, iterations=1)
    
    if not ret:
        print("Error: Could not read frame.")
    else:
        cv2.imwrite(output_file, frame)
        print(f"Picture saved as {output_file}")

        cv2.imshow('Captured Image', frame)
        cv2.waitKey(0) 

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # should be 60
    light_frequency = 60  
    camera_index = 0      # camera_index = 1
    

    if light_frequency == 60:
        fps = 30  # or 60
        exposure_time = 1 / 60 
    else: # one of the two
        fps = 25  # or 50
        exposure_time = 1 / 50  

    set_camera_settings_and_take_picture(camera_index=camera_index, fps=fps, exposure_time=exposure_time)
