from flask import Flask, render_template, Response, send_file, jsonify, request
import time
import os
import json
import threading

# 导入蓝牙库
try:
    import bluetooth
except ImportError:
    print("请安装pybluez库: pip install pybluez")
    print("如果安装失败，可能需要安装系统依赖: sudo apt-get install libbluetooth-dev")

app = Flask(__name__)

# 全局变量存储蓝牙连接
bt_socket = None
bt_connected = False
bt_device_address = None
bt_lock = threading.Lock()  # 用于线程安全操作

# WHEELTEC小车使用的蓝牙服务UUID和特征UUID
SERVICE_UUID = '0000ffe0-0000-1000-8000-00805f9b34fb'
CMD_CHAR = '0000ffe1-0000-1000-8000-00805f9b34fb'
SPEED_CHAR = '0000ffe2-0000-1000-8000-00805f9b34fb'  # 可选的速度特征

@app.route('/')
def index():
    return render_template('index.html')

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

# 新增: 蓝牙设备扫描API
@app.route('/scan_bluetooth', methods=['GET'])
def scan_bluetooth():
    try:
        print("开始扫描蓝牙设备...")
        nearby_devices = bluetooth.discover_devices(duration=8, lookup_names=True)
        devices = [{"address": addr, "name": name} for addr, name in nearby_devices]
        print(f"找到 {len(devices)} 个蓝牙设备")
        return jsonify({"success": True, "devices": devices})
    except Exception as e:
        print(f"扫描蓝牙设备时出错: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

# 新增: 蓝牙连接API
@app.route('/connect_bluetooth', methods=['POST'])
def connect_bluetooth():
    global bt_socket, bt_connected, bt_device_address
    
    data = request.get_json()
    if not data or 'address' not in data:
        return jsonify({"success": False, "error": "需要提供设备地址"})
    
    device_address = data['address']
    port = 1  # RFCOMM端口，可能需要根据您的设备调整
    
    try:
        with bt_lock:
            if bt_connected:
                # 如果已经连接，先断开
                try:
                    bt_socket.close()
                except:
                    pass
                bt_connected = False
                
            print(f"尝试连接到设备: {device_address}")
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.connect((device_address, port))
            bt_socket = sock
            bt_connected = True
            bt_device_address = device_address
            print(f"成功连接到设备: {device_address}")
            
        return jsonify({"success": True, "message": f"已连接到设备: {device_address}"})
    except Exception as e:
        print(f"连接到设备时出错: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

# 新增: 断开蓝牙连接API
@app.route('/disconnect_bluetooth', methods=['POST'])
def disconnect_bluetooth():
    global bt_socket, bt_connected, bt_device_address
    
    try:
        with bt_lock:
            if bt_connected and bt_socket:
                bt_socket.close()
                bt_connected = False
                print(f"已断开与设备的连接: {bt_device_address}")
                bt_device_address = None
                
        return jsonify({"success": True, "message": "已断开蓝牙连接"})
    except Exception as e:
        print(f"断开连接时出错: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

# 新增: 发送蓝牙命令API
@app.route('/send_command', methods=['POST'])
def send_command():
    global bt_socket, bt_connected
    
    data = request.get_json()
    if not data or 'code' not in data:
        return jsonify({"success": False, "error": "需要提供命令代码"})
    
    code = data['code']
    
    try:
        with bt_lock:
            if not bt_connected or not bt_socket:
                return jsonify({"success": False, "error": "没有活跃的蓝牙连接"})
            
            print(f"发送命令代码: {code}")
            # 发送单字节命令
            bt_socket.send(bytes([code]))
            
        return jsonify({"success": True, "message": "命令已发送"})
    except Exception as e:
        print(f"发送命令时出错: {str(e)}")
        with bt_lock:
            bt_connected = False  # 如果发送失败，将连接状态设为断开
        return jsonify({"success": False, "error": str(e)})

# 新增: 设置速度API
@app.route('/set_speed', methods=['POST'])
def set_speed():
    global bt_socket, bt_connected
    
    data = request.get_json()
    if not data or 'speed' not in data:
        return jsonify({"success": False, "error": "需要提供速度值"})
    
    speed = data['speed']
    
    try:
        with bt_lock:
            if not bt_connected or not bt_socket:
                return jsonify({"success": False, "error": "没有活跃的蓝牙连接"})
            
            print(f"设置速度: {speed}")
            # 发送速度值（单字节）
            bt_socket.send(bytes([speed]))
            
        return jsonify({"success": True, "message": "速度已设置"})
    except Exception as e:
        print(f"设置速度时出错: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

# 新增: 获取蓝牙连接状态API
@app.route('/bluetooth_status', methods=['GET'])
def bluetooth_status():
    global bt_connected, bt_device_address
    
    with bt_lock:
        status = {
            "connected": bt_connected,
            "device_address": bt_device_address if bt_connected else None
        }
    
    return jsonify(status)

if __name__ == '__main__':
    print("启动Flask应用...")
    print("请确保已安装必要的蓝牙库")
    print("如果出现蓝牙相关错误，请运行: sudo apt-get install libbluetooth-dev python3-dev")
    print("然后安装pybluez: pip install pybluez")
    app.run(host='0.0.0.0', port=5000)