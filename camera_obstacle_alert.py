# camera_obstacle_alert.py
import cv2
import time
import pyttsx3  

engine = pyttsx3.init()
engine.setProperty('rate', 150)   

def speak(text):
    engine.say(text)
    engine.runAndWait()

cap = cv2.VideoCapture(0) 
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# ---------- 参数设置 ----------
AREA_THRESHOLD = 15000    
COOLDOWN = 3               
last_alert_time = 0

print("quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blur, 50, 150)

    dilated = cv2.dilate(edges, None, iterations=2)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    danger_detected = False

    for c in contours:
        area = cv2.contourArea(c)
        if area > AREA_THRESHOLD:
            danger_detected = True
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "DANGER", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    current_time = time.time()
    if danger_detected and (current_time - last_alert_time > COOLDOWN):
        print("Dangerous! Obstacle detected.")
        speak("Dangerous, obstacle ahead")
        last_alert_time = current_time

    cv2.imshow("Camera Obstacle Alert", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
