from flask import Flask, render_template, Response, send_file, jsonify, request
import time
import os
import json
import threading

# 导入蓝牙库
try:
    # 尝试导入PyBluez
    import bluetooth
except ImportError:
    print("请安装pybluez库: pip install pybluez")
    print("如果安装失败，可能需要安装系统依赖: sudo apt-get install libbluetooth-dev python3-dev")
    bluetooth = None  # 确保变量已定义

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
    global bluetooth
    
    if bluetooth is None:
        return jsonify({"success": False, "error": "蓝牙库未正确加载。请确保已安装pybluez"})
    
    try:
        print("开始扫描蓝牙设备...")
        # 增加搜索时间以确保能找到设备
        nearby_devices = bluetooth.discover_devices(duration=10, lookup_names=True)
        devices = [{"address": addr, "name": name} for addr, name in nearby_devices]
        
        # 筛选出WHEELTECH-IOS设备或所有设备（如果没有找到特定设备）
        wheeltech_devices = [d for d in devices if "WHEELTECH" in d.get("name", "").upper()]
        if wheeltech_devices:
            print(f"找到 {len(wheeltech_devices)} 个WHEELTECH设备")
            return jsonify({"success": True, "devices": wheeltech_devices})
        else:
            print(f"找到 {len(devices)} 个蓝牙设备，但没有WHEELTECH设备")
            return jsonify({"success": True, "devices": devices})
    except Exception as e:
        print(f"扫描蓝牙设备时出错: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

# 新增: 蓝牙连接API
@app.route('/connect_bluetooth', methods=['POST'])
def connect_bluetooth():
    global bt_socket, bt_connected, bt_device_address, bluetooth
    
    if bluetooth is None:
        return jsonify({"success": False, "error": "蓝牙库未正确加载。请确保已安装pybluez"})
    
    data = request.get_json()
    if not data or 'address' not in data:
        return jsonify({"success": False, "error": "需要提供设备地址"})
    
    device_address = data['address']
    
    # 尝试几个可能的端口
    ports = [1, 2, 3, 4]
    
    for port in ports:
        try:
            with bt_lock:
                if bt_connected:
                    # 如果已经连接，先断开
                    try:
                        bt_socket.close()
                    except:
                        pass
                    bt_connected = False
                    
                print(f"尝试连接到设备: {device_address}，端口: {port}")
                sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                # 设置超时以避免长时间等待
                sock.settimeout(3)
                sock.connect((device_address, port))
                sock.settimeout(None)  # 恢复默认
                bt_socket = sock
                bt_connected = True
                bt_device_address = device_address
                print(f"成功连接到设备: {device_address}，端口: {port}")
                
            return jsonify({"success": True, "message": f"已连接到设备: {device_address}"})
        except Exception as e:
            print(f"连接到端口 {port} 时出错: {str(e)}")
            # 继续尝试下一个端口
    
    # 如果所有端口都失败
    return jsonify({"success": False, "error": "无法连接到设备，请确保设备已打开并在范围内"})

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
    
    # 检查蓝牙库是否正确加载
    if bluetooth is None:
        print("警告: 蓝牙库未加载。蓝牙功能将不可用。")
        print("请运行以下命令安装必要的依赖:")
        print("sudo apt-get update")
        print("sudo apt-get install -y libbluetooth-dev python3-dev")
        print("pip3 install pybluez")
        print("如果仍有问题，可以尝试从源代码安装:")
        print("pip3 install git+https://github.com/pybluez/pybluez.git#egg=pybluez")
    else:
        print("蓝牙库已加载。检查蓝牙适配器...")
        try:
            devices = bluetooth.discover_devices(duration=1, lookup_names=False)
            print(f"蓝牙适配器工作正常，可以扫描设备。")
        except Exception as e:
            print(f"检查蓝牙适配器时出错: {str(e)}")
            print("请确保蓝牙适配器已启用:")
            print("sudo rfkill unblock bluetooth")
            print("sudo hciconfig hci0 up")
    
    # 设置调试模式以便查看更多错误信息
    app.run(host='0.0.0.0', port=5000, debug=True)