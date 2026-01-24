import numpy as np 
import cv2


# ---- CONFIG ----
WIDTH  = 720   # change this
HEIGHT = 480   # change this
FORMAT = "YUYV"  # try: YUYV or UYVY
FILE   = "out1.yuv"  # your raw file
# ----------------

# Read raw data
with open(FILE, "rb") as f:
    raw = f.read()

# YUV422 = 2 bytes per pixel
expected_size = WIDTH * HEIGHT * 2
if len(raw) != expected_size:
    print(f"WARNING: file size {len(raw)} != expected {expected_size}")

# Convert to numpy
yuv = np.frombuffer(raw, dtype=np.uint8)
yuv = yuv.reshape((HEIGHT, WIDTH, 2))

# Convert to RGB
if FORMAT == "YUYV":
    rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_UYVY)
elif FORMAT == "UYVY":
    rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_UYVY)
else:
    raise ValueError("Unknown format")

# COLOR_YUV2RGB_UYVY

# Save image
cv2.imwrite("output.png", rgb)

# Show image
cv2.imshow("Frame", rgb)
cv2.waitKey(0)
cv2.destroyAllWindows()