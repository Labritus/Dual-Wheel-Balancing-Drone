from flask import Flask, render_template, Response, send_file
import time
import os

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')   # 下面贴模板

def gen_frames():
    """不断读取 /tmp/latest.jpg 并以 MJPEG 推送"""
    while True:
        if os.path.exists('/tmp/latest.jpg'):
            # 读取最新文件
            with open('/tmp/latest.jpg','rb') as f:
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
