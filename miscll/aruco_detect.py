# Define the known relationship between the marker size and the distance
# In this case, the marker size is 0.1 meters and the camera has a FOV of 60 degrees
marker_size = 0.0762  # meters
fov_degrees = 60

while True:
    # Read the current frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect the Aruco markers in the image
    corners, ids, _ = cv2.aruco.detectMarkers(frame, cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250))

    # If markers were detected
    if ids is not None:
        # Get the size of the first marker in pixels
        marker_size_pixels = cv2.norm(corners[0][0][0] - corners[0][0][1])

        # Convert the marker size from pixels to meters using the known relationship
        # and the FOV of the camera
        fov_radians = fov_degrees * (math.pi / 180)
        distance = marker_size / (2 * math.tan(fov_radians / 2) * (marker_size_pixels / frame.shape[1]))
        
        # Print the estimated distance
        print("Estimated distance:", distance, "meters")

    # Display the webcam feed with the detected markers
    cv2.imshow("Webcam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()
