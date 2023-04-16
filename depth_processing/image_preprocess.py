import numpy as np
import cv2
import matplotlib.pyplot as plt

data_right = {}
data_left = {}
normalized_right = np.zeros((640, 480))
normalized_left = np.zeros((640, 480))
for i in range(1, 10):
    fd = open(f'img{i}_l_Depth.raw', 'rb')
    rows = 640
    cols = 480
    f = np.fromfile(fd, dtype=np.uint16, count=rows * cols)  
    img = np.reshape(f, (rows, cols))
    fd.close()
    img[img > 1400] = 0
    img[img < 900] = 0
    data_left[f'img{i}_l'] = img
# for i in range(10):
#     img = cv2.imread(f'img{i}_l')
#     img = np.asarray(img)
#     img[img > 1400] = 0
#     img[img < 900] = 0
#     data_left[f'img{i}_l'] = img
for i in range(1, 10):
    # normalized_right += data_right[f'img{i}_r']
    normalized_left += data_left[f'img{i}_l']

# normalized_right /= 10
normalized_left /= 10
plt.imshow(normalized_left)
plt.savefig('left.png')