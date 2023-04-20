# import libraries
import cv2
import face_recognition

# distance from camera to object(face) measured
# centimeter
Known_distance = 76.2
  
# width of face in the real world or Object Plane
# centimeter
Known_width = 14.3

# Get a reference to webcam 
video_capture = cv2.VideoCapture(0)

#Output
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')# note the lower case

frame_width = int(video_capture.get(3))
frame_height = int(video_capture.get(4))

# out = cv2.VideoWriter(r'/Users/venkateshchandra/Desktop/Computer_vision/webcam.mp4',fourcc , 10, (frame_width,frame_height), True)
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
  
    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length

# Initialize variables
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
  
    distance = (real_face_width * Focal_Length)/face_width_in_frame
  
    # return the distance
    return distance
face_locations = []

while True:
    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_frame = frame[:, :, ::-1]
    
    # Find all the faces in the current frame of video
    face_locations = face_recognition.face_locations(rgb_frame)

    ref_image_face_width= 0
    # Display the results
    for top, right, bottom, left in face_locations:
        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        print(left, top)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, 'Face', (top + 6, right - 6), font, 0.5, (0, 0, 255), 1)
        ref_image_face_width = right-left
    # Display the resulting image
    cv2.imshow('Video', frame)
    
    #Record
    # out.write(frame)
    
    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
# out.release()
cv2.destroyAllWindows()