import cv2
import numpy as np
import math
import statistics

def point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False

    px, py = polygon[0]
    for i in range(1, n + 1):
        sx, sy = polygon[i % n]
        if y > min(py, sy):
            if y <= max(py, sy):
                if x <= max(px, sx):
                    if py != sy:
                        intersect = (y - py) * (sx - px) / (sy - py) + px
                    if px == sx or x <= intersect:
                        inside = not inside
        px, py = sx, sy

    return inside

def find_closest_black_point(trackSnapshot, point, slope, margin=10):
    x, y = point
    dx = 1
    dy = slope * dx

    norm = np.sqrt(dx ** 2 + dy ** 2)
    if norm != 0:
        dx /= norm
        dy /= norm
    else:
        return None, None
    
    point1 = None
    point2 = None

    search_range = 500
    for i in range(search_range):
        x1 = int(x + i * dx)
        y1 = int(y + i * dy)

        if 0 <= y1 < trackSnapshot.shape[0] and 0 <= x1 < trackSnapshot.shape[1] and trackSnapshot[y1, x1] == 0:
            point1 = (x1, y1)
            break

    for i in range(search_range):
        x2 = int(x - i * dx)
        y2 = int(y - i * dy)

        if 0 <= y2 < trackSnapshot.shape[0] and 0 <= x2 < trackSnapshot.shape[1] and trackSnapshot[y2, x2] == 0:
            point2 = (x2, y2)
            break

    return point1, point2

# for slope == 0
def find_closest_black_point_vertical(trackSnapshot, point, margin=10):
    x, y = point
    
    point1 = None
    point2 = None

    search_range = 500
    for i in range(search_range):
        x1 = x
        y1 = y - i

        if 0 <= y1 < trackSnapshot.shape[0] and 0 <= x1 < trackSnapshot.shape[1] and trackSnapshot[y1, x1] == 0:
            point1 = (x1, y1)
            break

    for i in range(search_range):
        x2 = x
        y2 = y + i

        if 0 <= y2 < trackSnapshot.shape[0] and 0 <= x2 < trackSnapshot.shape[1] and trackSnapshot[y2, x2] == 0:
            point2 = (x2, y2)
            break

    return point1, point2

def detect_and_draw_lines(trackSnapshot, back, rover_front):
    if rover_front[0] == back[0]:  # x1-x2 = 0, diving by 0 not possible
        slope = np.inf
    else:
        slope = (back[1] - rover_front[1]) / (back[0] - rover_front[0])

    if slope == 0: # same y value, y2- y1 = 0, cannot divide -1 by 0 for perpendicular, hence we make a dedicated function for this
        closest_point_1, closest_point_2 = find_closest_black_point_vertical(trackSnapshot, rover_front)
    elif np.isnan(slope):
        return None, None
    else:
        closest_point_1, closest_point_2 = find_closest_black_point(trackSnapshot, rover_front, -1 / slope)

    return closest_point_1, closest_point_2

def calibratedCenter(raw_center, middle_x, middle_y):
    new_center_x = None
    new_center_y = None

    sum_vector_component_y = int(abs(raw_center[1] - 240)*0.895) # 0.9075 # 0.92 -1
    sum_vector_component_x = int(abs(raw_center[0] - 320)*0.895) # 0.9075 # 0.92 -1

    if raw_center[1] >= middle_y:
        new_center_y = middle_y + sum_vector_component_y
    else:
        new_center_y = middle_y - sum_vector_component_y


    
    if raw_center[0] > middle_x:
        new_center_x = middle_x + sum_vector_component_x
    else:
        new_center_x = middle_x - sum_vector_component_x

    
    return (new_center_x, new_center_y)

def Euclidean_distance(pixel1, pixel2):
    x1, y1 = pixel1
    x2, y2 = pixel2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

def error_color_gradient (error_percentage):
    error_percentage = max(0, min(100, error_percentage))

    if error_percentage <= 10: # for green to Yellow
        green = 255
        red = int(255 * (error_percentage / 10))
        blue = 0
    elif 10 < error_percentage <= 20: # for Yellow to Orange
        red = 255
        green = int(255 - 255 * ((error_percentage - 10) / 10))
        blue = 0
    elif 20 < error_percentage <= 30: #for Orange to Red
        red = 255
        green = int(255 * (1 - (error_percentage - 20) / 10))
        blue = 0
    else: # beyond 30% just keep it red
        red = 255
        green = 0
        blue = 0

    return (blue, green, red)


camera_index = 0
cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

fps = 30
cap.set(cv2.CAP_PROP_FPS, fps)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

trackSnapshot = cv2.imread("processed_track.jpg", cv2.IMREAD_GRAYSCALE)
trajectoryAnalysis = cv2.imread("processed_track.jpg")

if trackSnapshot is None:
    print("Error: Could not read track snapshot.")
    exit()

img_height, img_width = trackSnapshot.shape

if frame_width != img_width or frame_height != img_height:
    print("Using different types of camera frames")

detector = cv2.aruco.ArucoDetector(dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100))


previous_lane_width = int(0) 
all_errors = []

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        centers = []
        for corner, id in zip(corners, ids):
            cv2.polylines(frame, [np.int32(corner)], True, (200, 200, 0), 2)

            corner = corner.reshape((4, 2))
            center_x = int(np.mean(corner[:, 0]))
            center_y = int(np.mean(corner[:, 1]))
            center = (center_x, center_y)

            center = calibratedCenter(center, frame_width/2, frame_height/2)

            if center[0] is not None and center[1] is not None:
                center = (int(center[0]), int(center[1]))
                centers.append((center, id[0]))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)


        rover_front = next((center for center, id in centers if id == 0), None)
        rover_back = next((center for center, id in centers if id == 1), None)
        
        if rover_front and rover_back:
            cv2.line(frame, rover_front, rover_back, (0, 0, 255), 2)
            closest_point_1, closest_point_2 = detect_and_draw_lines(trackSnapshot, rover_back, rover_front)
            if closest_point_1 is not None and closest_point_2 is not None:
                cv2.line(frame, rover_front, closest_point_1, (0, 255, 0), 2)
                cv2.line(frame, rover_front, closest_point_2, (0, 255, 0), 2)

                distance_point2 = int(Euclidean_distance(rover_front, closest_point_2))
                distance_point1 = int(Euclidean_distance(rover_front, closest_point_1))

                # ----------------------------------------------------------------------------------------------
                # this is the error handling for the tight curve problem where boundaries are detected far away #
                if distance_point2 + distance_point1 > 115: ## this number 115 was carefully chosen after looking at (distance_point1 + distance_rigth) values at different points in track      
                    if max(distance_point1, distance_point2) == distance_point2: 
                        distance_point2 = previous_lane_width - distance_point1
                        cv2.line(frame, rover_front, closest_point_1, (0, 255, 0), 2)
                        cv2.line(frame, rover_front, closest_point_2, (0, 0, 255), 2)
                    else:
                        distance_point1 = previous_lane_width - distance_point2
                        cv2.line(frame, rover_front, closest_point_1, (0, 0, 255), 2)
                        cv2.line(frame, rover_front, closest_point_2, (0, 255, 0), 2)

                    previous_lane_width = previous_lane_width            
                else:
                    cv2.line(frame, rover_front, closest_point_1, (0, 255, 0), 2)
                    cv2.line(frame, rover_front, closest_point_2, (0, 255, 0), 2)
                    previous_lane_width = distance_point1 + distance_point2
                # I am sure there is a way around this, but right now I can't figure it out   
                # ------------------------------------------------------------------------------------------------

                if distance_point2 + distance_point1 != 0:
                    error_percentage = round(abs((distance_point2/(distance_point1+distance_point2)) - 0.5)*100 ,2)
                    print("you are off from the center by", error_percentage, "%")
                    all_errors.append(error_percentage)
                    color = error_color_gradient(error_percentage)
                    cv2.circle(trajectoryAnalysis, rover_front, 3, color, -1)
                    

                else:
                    print("rover_front center must be inside the boundary, you are drunk and made a really bad turn")


    cv2.imshow('Camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        mean_error = statistics.mean(all_errors)
        std_deviation = statistics.stdev(all_errors)
        print(f"Mean error percentage: {mean_error:.2f}")
        print(f"Standard deviation of error percentage: {std_deviation:.2f}")

        output_file = "Trajectory Analysis.jpg"
        cv2.imwrite(output_file, trajectoryAnalysis)
        print(f"Picture saved as {output_file}")

        break

cap.release()
cv2.destroyAllWindows()
