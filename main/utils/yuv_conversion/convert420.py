import numpy as np
import cv2

W, H = 720, 480
FILE = "out.yuv"   # <-- change to your filename

raw = np.frombuffer(open(FILE, "rb").read(), dtype=np.uint8)

expected = W * H * 3 // 2  # YUV420
print("bytes:", raw.size, "expected:", expected)
if raw.size < expected:
    raise ValueError("File too small for a single 720x480 YUV420 frame.")
if raw.size > expected:
    # If your file contains multiple frames, this just grabs the first one
    raw = raw[:expected]

# OpenCV expects a 2D array of shape (H*1.5, W) for YUV420 variants
yuv = raw.reshape((H * 3 // 2, W))

conversions = {
    "I420": cv2.COLOR_YUV2RGB_I420,  # planar: Y + U + V
    "YV12": cv2.COLOR_YUV2RGB_YV12,  # planar: Y + V + U
    "NV12": cv2.COLOR_YUV2RGB_NV12,  # semi-planar: Y + interleaved UV
    "NV21": cv2.COLOR_YUV2RGB_NV21,  # semi-planar: Y + interleaved VU
}

for name, code in conversions.items():
    try:
        rgb = cv2.cvtColor(yuv, code)
        out = f"out_{name}.png"
        cv2.imwrite(out, rgb)
        print("wrote", out)
    except cv2.error as e:
        print("failed", name, e)

# Extra: save luma only (debug timing/stride)
Y = raw[:W*H].reshape((H, W))
cv2.imwrite("luma.png", Y)
print("wrote luma.png")
