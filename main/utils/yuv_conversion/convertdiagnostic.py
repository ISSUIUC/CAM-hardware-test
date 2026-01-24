import numpy as np
import cv2

W, H = 720, 480
raw = np.frombuffer(open("out1.yuv", "rb").read(), dtype=np.uint8)

for stride in [736, 752, 768, 800, 832]:
    try:
        Y = np.zeros((H, W), dtype=np.uint8)
        for row in range(H):
            Y[row] = raw[row*stride : row*stride + W]
        cv2.imwrite(f"luma_stride_{stride}.png", Y)
        print("wrote stride", stride)
    except:
        pass
