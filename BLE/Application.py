from flask import Flask, render_template, Response, send_file
import time
import os

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')   # Template is shown below

def gen_frames():
    """Continuously read /tmp/latest.jpg and stream it as MJPEG"""
    while True:
        if os.path.exists('/tmp/latest.jpg'):
            # Read the latest image file
            with open('/tmp/latest.jpg', 'rb') as f:
                frame = f.read()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)  # ~30fps

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
