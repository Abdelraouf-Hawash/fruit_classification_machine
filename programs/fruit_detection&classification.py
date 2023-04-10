# BY: Abdelraouf Hawash
# DATE: 9/4/2023
# GMAIL: abdelraouf.hawash@gmail.com

import cv2
import numpy as np
import tensorflow as tf

# remember to select your cam ID
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap.set(3, 640)
cap.set(4, 480)

model = tf.keras.models.load_model('./models/lemon_quality')

def nothing(x):
    pass

cv2.namedWindow("tracking")
cv2.createTrackbar("LH", "tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "tracking", 100, 255, nothing)
cv2.createTrackbar("LV", "tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "tracking", 255, 255, nothing)
cv2.createTrackbar("US", "tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "tracking", 255, 255, nothing)

while (cap.isOpened):
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    l_h = cv2.getTrackbarPos("LH", "tracking")
    l_s = cv2.getTrackbarPos("LS", "tracking")
    l_v = cv2.getTrackbarPos("LV", "tracking")

    u_h = cv2.getTrackbarPos("UH", "tracking")
    u_s = cv2.getTrackbarPos("US", "tracking")
    u_v = cv2.getTrackbarPos("UV", "tracking")

    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_h, u_h])

    mask = cv2.inRange(hsv, l_b, u_b)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
            if cv2.contourArea(contour) < 2000: continue
            x, y, w, h = cv2.boundingRect(contour)
            fruit = frame[y:y+h, x:x+w]
            fruit = cv2.resize(fruit,(180,180))
            fruit = cv2.cvtColor(fruit, cv2.COLOR_BGR2RGB)
            fruit = tf.expand_dims(fruit, 0)
            predictions = model.predict(fruit)
            score = tf.nn.softmax(predictions[0])
            index = np.argmax(score)
            if index :
                fruit_is_normal = True
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2) # green rectangle for normal
            else:
                fruit_is_normal = False
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2) # red rectangle for abnormal

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    k=cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
