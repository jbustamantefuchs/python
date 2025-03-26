import sys
import cv2
import jetson.utils  
from jetson_inference import detectNet
import os; os.system("pip install paho-mqtt")
import paho.mqtt.client as mqtt


cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("Error")
    sys.exit()
net = detectNet("ssd-mobilenet-v2", threshold=0.5)


broker = "localhost" 
port = 1883 
topic = "presence" 
client = mqtt.Client()
client.connect(broker, port, 60)


cv2.namedWindow("Video with DetectNet", cv2.WINDOW_NORMAL)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cv2.resizeWindow("Video with DetectNet", frame_width, frame_height)


while True:
    ret, frame = cap.read()
    if not ret:  
        print("Cant Read Frame.")
        break
    frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = jetson.utils.cudaFromNumpy(frame_rgba)
    detections = net.Detect(img)
    frame_bgr = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2BGR)
    persona_detectada = False
    for detection in detections:
        if detection.ClassID == 1:
            persona_detectada = True
            cv2.rectangle(frame_bgr, 
                          (int(detection.Left), int(detection.Top)),
                          (int(detection.Right), int(detection.Bottom)),
                          (0, 255, 0), 2)


            label = f"Person {detection.Confidence:.2f}"
            cv2.putText(frame_bgr, label, 
                        (int(detection.Left), int(detection.Top) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
    if persona_detectada:
        client.publish(topic, "1")
    else:
        client.publish(topic, "0")


    cv2.imshow("Video with DetectNet", frame_bgr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()

