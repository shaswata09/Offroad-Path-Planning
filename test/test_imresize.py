import cv2

filename = "/home/shaswata/Pictures/Screenshot from 2021-11-23 13-26-12.png"
W = 1000.0
oriimg = cv2.imread(filename)
# height, width, depth = oriimg.shape

print(oriimg.shape)

newimg = cv2.resize(oriimg, (1500, 1500))
print(newimg.shape)

cv2.imshow("Show by CV2", newimg)
cv2.waitKey(0)
# cv2.imwrite("resizeimg.jpg", newimg)
