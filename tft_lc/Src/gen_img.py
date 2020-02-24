#!/usr/bin/env python

# vim: set ai et ts=4 sw=4:

import cv2
import sys
import os
import numpy as np

img = cv2.imread("pif.png")
img = cv2.resize(img, (128,128)) 
img = cv2.flip( img, 1 )
img = cv2.flip( img, 0 )
# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

for y in range(0, img.shape[0]):
    s = "{"
    for x in range(0, img.shape[1]):
        (b, g, r) = img[x, y,:]
        color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3)
        # for right endiness, so ST7735_DrawImage would work
        color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8)
        s += "0x{:04X},".format(color565)
    s += "},"
    print(s)

print("};")
cv2.imshow('image',img)
k = cv2.waitKey(0)
if k == 27:         # wait for ESC key to exit
    cv2.destroyAllWindows()
