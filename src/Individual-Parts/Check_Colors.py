from picamera2 import Picamera2
import cv2
import numpy as np

def nothing(x):
    pass

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

# Create window with trackbars
cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Trackbars", 400, 300)
cv2.createTrackbar("H Lower", "Trackbars", 0, 179, nothing)  # H is 0-179!
cv2.createTrackbar("H Upper", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S Lower", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("S Upper", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Lower", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V Upper", "Trackbars", 255, 255, nothing)

while True:
    frame = picam2.capture_array()
    
    # IMPORTANT: Match the robot code's conversion
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # RGB → BGR
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    # BGR → HSV
    
    # Get trackbar values
    h_l = cv2.getTrackbarPos("H Lower", "Trackbars")
    h_u = cv2.getTrackbarPos("H Upper", "Trackbars")
    s_l = cv2.getTrackbarPos("S Lower", "Trackbars")
    s_u = cv2.getTrackbarPos("S Upper", "Trackbars")
    v_l = cv2.getTrackbarPos("V Lower", "Trackbars")
    v_u = cv2.getTrackbarPos("V Upper", "Trackbars")
    
    lower = np.array([h_l, s_l, v_l])
    upper = np.array([h_u, s_u, v_u])
    
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(hsv, hsv, mask=mask)
    
    # Display current HSV values on frame
    cv2.putText(frame, f"HSV: ({h_l},{s_l},{v_l}) - ({h_u},{s_u},{v_u})", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)
    cv2.imshow("HSV", hsv)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):  # Press 'p' to print current values
        print(f"Lower: ({h_l}, {s_l}, {v_l})")
        print(f"Upper: ({h_u}, {s_u}, {v_u})")

picam2.stop()
cv2.destroyAllWindows()