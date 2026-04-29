from pseyepy import Camera
import cv2
import time
import os

cam_index = 3

cams = Camera(fps=90, resolution=Camera.RES_SMALL, colour=True)

os.makedirs(f'cam_{cam_index}', exist_ok=True)

for i in range(30):
    frame, timestamp = cams.read()
    
    # Show preview
    preview = cv2.resize(frame, (640, 480))
    cv2.putText(preview, f'Capturing: {i+1}/30', (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Preview', preview)
    cv2.waitKey(1)
    
    filename = f'cam_{cam_index}/image_{i}.jpg'
    cv2.imwrite(filename, frame)
    print(f"Saved {filename}")
    time.sleep(0.5)

cams.end()
cv2.destroyAllWindows()