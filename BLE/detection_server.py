#!/usr/bin/env python3
import asyncio
import websockets
import json
import cv2
import numpy as np
import base64
import time
from threading import Thread, Lock
import subprocess
import signal
import os
import sys

# Global variables
client_connected = False
frame_lock = Lock()
latest_frame = None
people_count = 0
detection_process = None
running = True

async def websocket_server(websocket, path):
    global client_connected, detection_process, running
    
    print(f"New client connected: {websocket.remote_address}")
    client_connected = True
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                command = data.get('command')
                
                if command == 'start_stream':
                    print("Starting people detection stream")
                    # Start detection if not already running
                    if detection_process is None:
                        start_detection_process()
                    
                    # Send initial frame request
                    await send_frame(websocket)
                
                elif command == 'next_frame':
                    # Send the next frame
                    await send_frame(websocket)
                
                elif command == 'stop_stream':
                    print("Stopping people detection stream")
                    # We'll keep the detection process running but stop sending frames
            
            except json.JSONDecodeError:
                print(f"Error decoding message: {message}")
    
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        client_connected = False

async def send_frame(websocket):
    global latest_frame, people_count
    
    with frame_lock:
        if latest_frame is None:
            # Send a black frame if no detection frame is available
            black_frame = np.zeros((240, 320, 3), dtype=np.uint8)
            _, buffer = cv2.imencode('.jpg', black_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_data = buffer.tobytes()
        else:
            # Use the latest detection frame
            _, buffer = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_data = buffer.tobytes()
    
    # Send the frame as binary data
    await websocket.send(frame_data)
    
    # Send stats as JSON
    stats = {
        'type': 'stats',
        'people_count': people_count,
        'timestamp': time.time()
    }
    await websocket.send(json.dumps(stats))

def start_detection_process():
    global detection_process
    
    # Modify the detection program to output to a pipe
    # You'll need to modify the C++ code to write frames to stdout or a named pipe
    # Here we assume it has been modified to write to a named pipe
    
    # Create a named pipe for communication with the C++ program
    pipe_path = "/tmp/detection_pipe"
    if not os.path.exists(pipe_path):
        os.mkfifo(pipe_path)
    
    # Start the C++ detection program
    detection_process = subprocess.Popen(
        ["/path/to/people_detection_program"],  # Change this to the actual path
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
        shell=False
    )
    
    # Start a thread to read from the detection process
    detection_thread = Thread(target=read_detection_output, daemon=True)
    detection_thread.start()
    
    print("Detection process started")

def read_detection_output():
    global latest_frame, people_count, running
    
    # Open the named pipe for reading
    pipe_path = "/tmp/detection_pipe"
    pipe_fd = os.open(pipe_path, os.O_RDONLY | os.O_NONBLOCK)
    
    buffer = b""
    frame_separator = b"FRAME_END"
    
    while running:
        try:
            # Try to read from the pipe
            data = os.read(pipe_fd, 65536)  # Read in chunks
            if data:
                buffer += data
                
                # Process complete frames
                while frame_separator in buffer:
                    frame_end = buffer.find(frame_separator)
                    frame_data = buffer[:frame_end]
                    buffer = buffer[frame_end + len(frame_separator):]
                    
                    # The frame data should be in the format: <people_count>:<jpg_data>
                    parts = frame_data.split(b":", 1)
                    if len(parts) == 2:
                        try:
                            count = int(parts[0])
                            jpg_data = parts[1]
                            
                            # Decode the JPEG data
                            img_array = np.frombuffer(jpg_data, dtype=np.uint8)
                            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                            
                            if img is not None:
                                with frame_lock:
                                    latest_frame = img
                                    people_count = count
                        except Exception as e:
                            print(f"Error processing frame: {e}")
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
        except (BlockingIOError, OSError) as e:
            # No data available or pipe closed
            time.sleep(0.1)
    
    # Close the pipe
    os.close(pipe_fd)

def cleanup():
    global running, detection_process
    
    running = False
    
    # Terminate the detection process if it's running
    if detection_process:
        detection_process.terminate()
        try:
            detection_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            detection_process.kill()
        detection_process = None
    
    # Remove the named pipe
    pipe_path = "/tmp/detection_pipe"
    if os.path.exists(pipe_path):
        os.unlink(pipe_path)

def signal_handler(sig, frame):
    print("Received shutdown signal")
    cleanup()
    sys.exit(0)

if __name__ == "__main__":
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start WebSocket server
    start_server = websockets.serve(websocket_server, "0.0.0.0", 8765)
    
    print("WebSocket server running on ws://0.0.0.0:8765")
    
    try:
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
    finally:
        cleanup()