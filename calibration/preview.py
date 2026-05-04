from pseyepy import Camera
import cv2
import numpy as np

# Connect all 4 cameras
cams = Camera([0, 1, 2, 3], fps=90, resolution=Camera.RES_SMALL, colour=True)

while True:
    frames, _ = cams.read()

    thumb_w, thumb_h = 640, 480
    thumbs = []

    for i, frame in enumerate(frames):
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        thumb = cv2.resize(frame, (thumb_w, thumb_h))
        cv2.putText(thumb, f'cam_{i+1}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        thumbs.append(thumb)

    # 2x2 grid
    top_row = np.hstack([thumbs[0], thumbs[1]])
    bot_row = np.hstack([thumbs[2], thumbs[3]])
    grid = np.vstack([top_row, bot_row])

    cv2.imshow('All cameras', grid)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cams.end()
cv2.destroyAllWindows()