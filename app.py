
import cv2
import torch
import numpy as np
from flask import Flask, render_template, Response, request
import subprocess


app = Flask(__name__)

model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
print('regresamos a flask')
cv2.destroyAllWindows()
def generate_frames():
    camera = cv2.VideoCapture(0)

    while True:
        success, frame = camera.read()

        if not success:
            break
        else:
            results = model(frame)

            for detection in results.xyxy[0]:
                class_id = int(detection[5])
                confidence = float(detection[4])
                class_name = model.names[class_id]
                
                print(f"Clase: ",class_name)
                camera.release()
              
                
                if class_name:
                   print('entrando')                
                 


            np.squeeze(results.render())

            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    camera.release()
    cv2.destroyAllWindows()

def run_detector_process():
    subprocess.run(['python3', 'detectorPro.py'])
    
    
    
@app.route('/start_detection', methods=['POST'])
def start_detection():
    print('Entro a start_detection')
    if 'enviar' in request.form:
      
        subprocess.Popen(['python3', 'detectorPro.py'])

    return render_template('index.html')


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0')



