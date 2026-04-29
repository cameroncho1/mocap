from pseyepy import Camera
import cv2

cams = Camera(fps=90, resolution=Camera.RES_SMALL, colour=True)

while True:
    frame, _ = cams.read()
    frame = cv2.resize(frame, (640, 480))
    cv2.imshow('Camera Preview', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cams.end()
cv2.destroyAllWindows()