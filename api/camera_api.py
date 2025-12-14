import cv2
import numpy as np
import zmq
import time
import threading

class RobotImageReceiver:
    """机器人图像接收器"""
    
    def __init__(self, server_ip="10.192.1.3", port=5556):
        self.server_ip = server_ip
        self.port = port
        self.running = False
        self.current_frame = None
        self.lock = threading.Lock()
        self.thread = None
        self.frame_count = 0
    
    def start(self):
        """启动接收线程"""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print(f"✅ 图像接收器已启动: {self.server_ip}:{self.port}")
    
    def _receive_loop(self):
        """接收循环"""
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(f"tcp://{self.server_ip}:{self.port}")
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        time.sleep(0.2)
        
        try:
            while self.running:
                message = socket.recv()
                header_size = 12
                if len(message) < header_size:
                    continue
                
                jpg_data = message[header_size:]
                np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    with self.lock:
                        self.current_frame = frame
                        self.frame_count += 1
        except Exception as e:
            print(f"接收错误: {e}")
        finally:
            socket.close()
            context.term()
    
    def get_frame(self):
        """获取最新帧"""
        with self.lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def stop(self):
        """停止接收"""
        self.running = False