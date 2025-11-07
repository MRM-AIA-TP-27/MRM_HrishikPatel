import cv2
import numpy
camera_matrix = numpy.array([
    [1000.0, 0.0, 640.0],  
    [0.0, 1000.0, 360.0],  
    [0.0, 0.0, 1.0]        
], dtype=numpy.float32)

distance = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=numpy.float32) 
scale = .05
Aruco= [
    ("4x4_50", cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)),
    ("5x5_50", cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)),
    ("6x6_50", cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50))
]
def estimate_and_draw_pose(frame, camera_matrix, dist_coeffs, marker_size):
    #detect aruco markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for name, dictionary in Aruco:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs
            )
            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_size * 0.75)
                marker_id = ids[i][0]
                tvec = tvecs[i][0]
                depth_z = tvec[2]
                distance = numpy.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
                text = (
                    f"ID: {marker_id} ({name}), "
                    f"Depth: {depth_z:.2f} m, "
                    f"Dist: {distance:.2f} m"
                )
                top_left_corner = tuple(corners[i][0][0].astype(int))
                text_size, _ = cv2.getTextSize(text, cv2.FONT_ITALIC, 0.5, 1)
                text_w, text_h = text_size
                text_x, text_y = top_left_corner[0], top_left_corner[1] - 10
                if text_y < text_h + 10:
                    text_y = top_left_corner[1] + 20
                cv2.rectangle(
                    frame, 
                    (text_x, text_y - text_h - 5), 
                    (text_x + text_w, text_y + 5), 
                    (0, 255, 255),
                    cv2.FILLED
                )
                cv2.putText(
                    frame, 
                    text, 
                    (text_x, text_y), 
                    cv2.FONT_ITALIC, 
                    0.5, 
                    (255, 0, 0),
                    2, 
                    cv2.LINE_AA
                )
    return frame
def main():
    #webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    print(f"Marker Size set to: {scale} meters.")
    print("Press 'q' to exit the video stream.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to grab frame.")
                break
            processed_frame = estimate_and_draw_pose(
                frame, 
                camera_matrix, 
                distance, 
                scale
            )
            cv2.imshow('ArUco Pose Estimation', processed_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
