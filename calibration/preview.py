from pseyepy import Camera
import cv2

# Connect one camera
cam = Camera(0, fps=90, resolution=Camera.RES_SMALL, colour=True)

while True:
    frame, _ = cam.read()

    # Convert RGB to BGR for OpenCV display
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Optional: resize display
    frame = cv2.resize(frame, (640, 480))

    cv2.putText(
        frame,
        "cam_1",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.end()
cv2.destroyAllWindows()