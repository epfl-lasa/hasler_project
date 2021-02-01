import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64MultiArray


def detect_blur_fft(image, size=60, thresh=10, vis=False):
    # grab the dimensions of the image and use the dimensions to
    # derive the center (x, y)-coordinates
    (h, w) = image.shape
    (cX, cY) = (int(w / 2.0), int(h / 2.0))
    # compute the FFT to find the frequency transform, then shift
    # the zero frequency component (i.e., DC component located at
    # the top-left corner) to the center where it will be more
    # easy to analyze
    fft = np.fft.fft2(image)
    fftShift = np.fft.fftshift(fft)

    # compute the magnitude spectrum of the transform
    magnitude = 20 * np.log(np.abs(fftShift))

    fftShift[cY - size:cY + size, cX - size:cX + size] = 0
    fftShift = np.fft.ifftshift(fftShift)
    recon = np.fft.ifft2(fftShift)

    # compute the magnitude spectrum of the reconstructed image,
    # then compute the mean of the magnitude values
    magnitude = 20 * np.log(np.abs(recon))
    mean = np.mean(magnitude)
    # the image will be considered "blurry" if the mean value of the
    # magnitudes is less than the threshold value
    return (mean, mean <= thresh)


cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
# cap = cv2.VideoCapture(0)
kernel = np.ones((5 ,5), np.uint8)

rospy.init_node('tools_tracker', anonymous=True)
rate = rospy.Rate(30) # 10hz

pub = rospy.Publisher('surgical_task/tools_tip', Float64MultiArray, queue_size=1)

msg = Float64MultiArray()


blue_position = np.array([0, 0, 0])
green_position = np.array([0, 0, 0])

alpha = 0

while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    variance = cv2.Laplacian(gray, cv2.CV_64F).var()
    print("Variance", variance)
    # print("Bou", detect_blur_fft(gray))
    img_size = frame.shape


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

    # blue range
    lower_blue = np.array([98,49,0]) 
    upper_blue = np.array([139,255,255]) 
    # lower_blue = np.array([0,110,73]) 
    # upper_blue = np.array([139,255,255]) 
    lower_blue = np.array([79,71,54]) 
    upper_blue = np.array([144,255,255]) 
    lower_blue = np.array([150,60,0]) 
    upper_blue = np.array([255,255,255]) 

    # red range
    lower_red = np.array([0,115,0]) 
    upper_red = np.array([255,255,255]) 

    # green range
    lower_green = np.array([67, 47, 0]) 
    upper_green = np.array([83, 255, 255]) 
    lower_green = np.array([50,40, 0]) 
    upper_green = np.array([97, 255, 255]) 



    # preparing the masks to overlay 
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue) 
    mask_green = cv2.inRange(hsv, lower_green, upper_green) 


    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
    # mask_blue = cv2.medianBlur(mask_blue,9)

    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    variance_green = cv2.Laplacian(mask_green, cv2.CV_64F).var()
    print("Variance green", variance_green)
    variance_blue = cv2.Laplacian(mask_blue, cv2.CV_64F).var()
    print("Variance red", variance_blue)
    # mask_green = cv2.medianBlur(mask_green,9)

    # variance = cv2.Laplacian(mask_green, cv2.CV_64F).var()
    # print("Variance", variance)

    contours, hierarchy = cv2.findContours(mask_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # mask_green = cv2.drawContours(mask_green, contours, -1, (255,0,0), 3)

    # find the biggest countour (c) by the area
    if len(contours)!=0:
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)
        print("blue:",M["m00"])
        if(M["m00"]>500 and M["m00"]<40000 and variance_blue < 1000):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # print(cX,cY)
            # print(cX-img_size[0]/2, img_size[0], img_size[1])
            blue_position = alpha*blue_position+(1-alpha)*np.array([-2.0*float(cY-img_size[0]/2.0)/img_size[0], 2.0*float(cX-img_size[1]/2.0)/img_size[1],1])
            cX = int(blue_position[1]*img_size[1]/2)+img_size[1]/2
            cY = -int(blue_position[0]*img_size[0]/2)+img_size[0]/2
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(frame, "c_b", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            blue_position[2] = 0
    else:
        blue_position[2] = 0


    mask_green_2 = mask_green
    contours, hierarchy = cv2.findContours(mask_green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # mask_green = cv2.drawContours(mask_green, contours, -1, (255,0,0), 3)

    # find the biggest countour (c) by the area
    if len(contours)!=0:
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)
        print("green:",M["m00"])
        if(M["m00"]>500 and M["m00"]<40000 and variance_green < 1000):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # print(cX,cY)
            # print(cX-img_size[0]/2, img_size[0], img_size[1])
            green_position = alpha*green_position+(1-alpha)*np.array([-2.0*float(cY-img_size[0]/2.0)/img_size[0], 2.0*float(cX-img_size[1]/2.0)/img_size[1],1])
            cX = int(green_position[1]*img_size[1]/2)+img_size[1]/2
            cY = -int(green_position[0]*img_size[0]/2)+img_size[0]/2
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(frame, "c_g", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            green_position[2] = 0
    else:
        green_position[2] = 0



    # x,y,w,h = cv2.boundingRect(c)
    # cv2.rectangle(mask_green, (x, y), (x + w, y + h), (255, 0,0), 5)
    # center = (x,y)
    # print(center)

    # for c in contours:
    #     # print()
    #     print(cv2.contourArea(c))
    #     if cv2.contourArea(c) <= 100 :
    #         continue 


    # The black region in the mask has the value of 0, 
    # so when multiplied with original image removes all non-blue regions 
    result = cv2.bitwise_and(frame, frame, mask = mask_blue | mask_green)


    # M = cv2.moments(mask_blue)
    # if(M["m00"]>100000):
    #     # print "blue: ", M["m00"]
    #     cX = int(M["m10"] / M["m00"])
    #     cY = int(M["m01"] / M["m00"])
    #     blue_position = alpha*blue_position+(1.0-alpha)*np.array([-2.0*float(cY-img_size[0]/2.0)/img_size[0], 2.0*float(cX-img_size[1]/2.0)/img_size[1],1])
    #     # print(blue_position[0], blue_position[1])
    #     cX = int(blue_position[1]*img_size[1]/2)+img_size[1]/2
    #     cY = -int(blue_position[0]*img_size[0]/2)+img_size[0]/2
    #     cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
    #     # print(cX, cY)
    #     cv2.putText(frame, "c_b", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # else:
    #     blue_position[2] = 0

    # M = cv2.moments(mask_green)
    # if(M["m00"]>100000):
    #     # print "green: ", M["m00"]
    #     cX = int(M["m10"] / M["m00"])
    #     cY = int(M["m01"] / M["m00"])
    #     # print(cX,cY)
    #     # print(cX-img_size[0]/2, img_size[0], img_size[1])
    #     green_position = alpha*green_position+(1-alpha)*np.array([-2.0*float(cY-img_size[0]/2.0)/img_size[0], 2.0*float(cX-img_size[1]/2.0)/img_size[1],1])
    #     cX = int(green_position[1]*img_size[1]/2)+img_size[1]/2
    #     cY = -int(green_position[0]*img_size[0]/2)+img_size[0]/2
    #     cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
    #     cv2.putText(frame, "c_g", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # else:
    #     green_position[2] = 0

    tools_pose = np.concatenate((blue_position, green_position), axis=None)

    msg.data = tools_pose # assign the array with the value you want to send
    pub.publish(msg)

    cv2.imshow('frame', frame) 
    cv2.imshow('mask_blue', mask_blue) 
    cv2.imshow('mask_green_2', mask_green) 
    cv2.imshow('result', result) 

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rate.sleep()

    # M = cv2.moments(mask)
    # cX = int(M["m10"] / M["m00"])
    # cY = int(M["m01"] / M["m00"])
    # cv2.circle(result, (cX, cY), 5, (255, 255, 255), -1)
    # cv2.putText(result, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # ret, frame = cap.read()
    # rangomax = np.array([255, 50, 50]) # B, G, R
    # rangomin = np.array([51, 0, 0])
    # mask = cv2.inRange(frame, rangomin, rangomax)
    # # reduce the noise
    # opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # x, y, w, h = cv2.boundingRect(opening)

    # cv2.rectangle(frame, (x, y), (x+w, y + h), (0, 255, 0), 1)
    # cv2.circle(frame, (x+w/2, y+h/2), 5, (0, 0, 255), -1)

    # cv2.imshow('camera', frame)

    # k = cv2.waitKey(1) & 0xFF

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()