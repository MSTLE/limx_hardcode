      
import json
import uuid
import threading
import time
import websocket

class RobotClient:
    def __init__(self, ip="10.192.1.2", port=5000):
        self.server_url = f"ws://{ip}:{port}"
        self.ws = None
        self.ws_thread = None
        self.accid = None
        self.connected = False
        
        # 用于存储待处理的请求响应 {guid: {event: threading.Event, response: dict}}
        self._pending_requests = {}
        self._lock = threading.Lock()

    def connect(self):
        """启动 WebSocket 连接"""
        # 禁用详细的调试日志
        websocket.enableTrace(False)
        self.ws = websocket.WebSocketApp(
            self.server_url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_close=self._on_close,
            on_error=self._on_error
        )
        
        # 在单独线程运行 websocket，避免阻塞主程序
        self.ws_thread = threading.Thread(target=self._run_forever, daemon=True)
        self.ws_thread.start()
        
        # 等待连接建立和 accid 获取
        print(f"Connecting to robot at {self.server_url}...")
        timeout = 5
        start_time = time.time()
        
        # 阻塞等待连接成功（或者直到超时）
        while not self.connected and (time.time() - start_time < timeout):
            time.sleep(0.1)
            
        if self.connected:
            print(f"Robot connected! ACCID: {self.accid}")
        else:
            print("Connection timeout. Please check IP or Network.")

    def _run_forever(self):
        """WebSocket 运行循环"""
        try:
            # 简化配置，避免 sockopt 问题
            self.ws.run_forever()
        except Exception as e:
            print(f"WebSocket run_forever error: {e}")
            self.connected = False

    def disconnect(self):
        """断开连接"""
        if self.ws:
            self.ws.close()
        self.connected = False
        print("Disconnected from robot.")

    # --- 内部回调 ---
    def _on_open(self, ws):
        print("WebSocket Connection Opened")

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
            title = data.get("title", "")
            msg_guid = data.get("guid", "")
            
            # 自动捕获 ACCID (通常在第一条消息或心跳中包含)
            if "accid" in data and data["accid"] and not self.accid:
                self.accid = data["accid"]
                self.connected = True
            
            # 处理同步等待的 Request-Response
            with self._lock:
                if msg_guid in self._pending_requests:
                    req = self._pending_requests[msg_guid]
                    req['response'] = data
                    req['event'].set() # 唤醒等待的线程

            # 如果是通知类消息（如 robot_info），可以在这里处理
            # if title == "notify_robot_info":
            #     pass 
                
        except Exception as e:
            print(f"Error parsing message: {e}")

    def _on_close(self, ws, close_code=None, close_msg=None):
        self.connected = False
        if close_code:
            print(f"WebSocket Connection Closed: {close_code} - {close_msg}")
        else:
            print("WebSocket Connection Closed")

    def _on_error(self, ws, error):
        print(f"WebSocket Error: {error}")
        self.connected = False

    # --- 核心发送逻辑 ---
    def send_command(self, title, data=None, wait_for_response=False, timeout=3.0):
        """
        发送指令的通用方法。
        :param title: 协议标题
        :param data: 数据字典
        :param wait_for_response: True则阻塞等待结果，False则只发送
        :param timeout: 等待超时时间
        :return: 响应数据的 data 字段 (如果 wait_for_response=True) 或 guid
        """
        if not self.connected:
            print("Error: Robot not connected.")
            return None

        guid = str(uuid.uuid4())
        payload = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "data": data if data else {}
        }

        # 如果需要等待响应，先注册事件
        event = threading.Event()
        if wait_for_response:
            with self._lock:
                self._pending_requests[guid] = {'event': event, 'response': None}

        try:
            self.ws.send(json.dumps(payload))
        except Exception as e:
            print(f"Send failed: {e}")
            # 如果发送失败，清理 pending request
            if wait_for_response:
                with self._lock:
                    self._pending_requests.pop(guid, None)
            return None

        if wait_for_response:
            # 阻塞等待，直到 _on_message 收到回包或超时
            is_set = event.wait(timeout)
            response_data = None
            
            with self._lock:
                if guid in self._pending_requests:
                    resp = self._pending_requests.pop(guid)
                    if is_set and resp['response'] is not None:
                        # 成功拿到响应
                        response_data = resp['response'].get('data', {})
                    else:
                        print(f"Timeout waiting for response: {title}")
            return response_data
        
        return guid

    # --- 封装API (按照飞书中接口说明的顺序) ---

    def set_damping(self):
        """
        1. 阻尼模式 (request_damping)
        进入阻尼模式机器人所有电机停止主动运动，摆动时有明显阻尼感
        :return: 响应结果，成功返回 "success"，失败返回 "fail_motor"
        """
        return self.send_command("request_damping", {}, wait_for_response=True)

    def set_stand_mode(self):
        """
        2. 站立模式 (request_prepare)
        进入准备状态控制机器人进入"站着状态"，可接受速度指令控制行走
        :return: 响应结果，成功返回 "success"，失败返回 "fail_motor"
        """
        return self.send_command("request_prepare", {}, wait_for_response=True)

    def set_manip_mode(self, mode):
        """
        3-5. 移动操作模式 (request_set_ub_manip_mode)
        :param mode: 0-准备进入模式, 1-操作模式，开始跟踪末端位置, 2-准备退出模式
        :return: 响应结果，成功返回 "success"，失败返回 "fail_motor"
        """
        data = {"mode": mode}
        return self.send_command("request_set_ub_manip_mode", data, wait_for_response=True)

    def set_manip_ee_pose(self, 
                          head_quat=None, 
                          left_pos=None, left_quat=None,
                          right_pos=None, right_quat=None,
                          use_current_pose=True):
        """
        6. 末端控制 (request_set_ub_manip_ee_pose)
        
        参考坐标系定义：
        - 原点：base_link坐标系
        - 方向：x方向对齐机器人正前方，y方向朝向机器人左侧，z方向竖直向上
        
        :param head_quat: 头相对于参考坐标系的姿态，四元数[x,y,z,w]，None时使用当前位姿
        :param left_pos: 左手相对于参考坐标系的位置，单位为米，None时使用当前位姿
        :param left_quat: 左手相对于参考坐标系的姿态，四元数[x,y,z,w]，None时使用当前位姿
        :param right_pos: 右手相对于参考坐标系的位置，单位为米，None时使用当前位姿
        :param right_quat: 右手相对于参考坐标系的姿态，四元数[x,y,z,w]，None时使用当前位姿
        :param use_current_pose: 是否使用当前位姿作为默认值
        :return: 响应结果 - "success": 成功, "fail_motor": 电机错误, "fail_invalid_cmd": 非法指令
        """
        
        # 如果参数为None且use_current_pose为True，获取当前位姿作为默认值
        if use_current_pose and (head_quat is None or left_pos is None or left_quat is None or 
                                right_pos is None or right_quat is None):
            current_pose = self.get_manip_ee_pose()
            if current_pose and current_pose.get("result") == "success":
                if head_quat is None:
                    head_quat = current_pose.get("head_quat", [0.0, 0.0, 0.0, 1.0])
                if left_pos is None:
                    left_pos = current_pose.get("left_hand_pos", [0.0, 0.0, 0.0])
                if left_quat is None:
                    left_quat = current_pose.get("left_hand_quat", [0.0, 0.0, 0.0, 1.0])
                if right_pos is None:
                    right_pos = current_pose.get("right_hand_pos", [0.0, 0.0, 0.0])
                if right_quat is None:
                    right_quat = current_pose.get("right_hand_quat", [0.0, 0.0, 0.0, 1.0])
            else:
                print("Warning: 无法获取当前位姿，使用默认值")
        
        # 设置默认值（如果仍为None）
        if head_quat is None:
            head_quat = [0.0, 0.0, 0.0, 1.0]
        if left_pos is None:
            left_pos = [0.0, 0.0, 0.0]
        if left_quat is None:
            left_quat = [0.0, 0.0, 0.0, 1.0]
        if right_pos is None:
            right_pos = [0.0, 0.0, 0.0]
        if right_quat is None:
            right_quat = [0.0, 0.0, 0.0, 1.0]
            
        # 参数验证
        if len(head_quat) != 4 or len(left_quat) != 4 or len(right_quat) != 4:
            print("Error: 四元数必须包含4个元素 [x,y,z,w]")
            return None
        if len(left_pos) != 3 or len(right_pos) != 3:
            print("Error: 位置必须包含3个元素 [x,y,z]")
            return None
            
        data = {
            "head_quat": head_quat,
            "left_hand_pos": left_pos,
            "left_hand_quat": left_quat,
            "right_hand_pos": right_pos,
            "right_hand_quat": right_quat
        }
        # print(data)
        return self.send_command("request_set_ub_manip_ee_pose", data, wait_for_response=True)


    def get_manip_ee_pose(self):
        """
        7. 获取末端位姿 (request_get_ub_manip_ee_pose)
        
        参考坐标系定义：
        - 原点：base_link坐标系
        - 方向：x方向对齐机器人正前方，y方向朝向机器人左侧，z方向竖直向上
        
        :return: 包含末端位姿信息的字典，包括：
            - head_pos: 头相对于参考坐标系的位置，单位为米 [x,y,z]
            - head_quat: 头相对于参考坐标系的姿态，四元数[x,y,z,w]
            - left_hand_pos: 左手相对于参考坐标系的位置，单位为米 [x,y,z]
            - left_hand_quat: 左手相对于参考坐标系的姿态，四元数[x,y,z,w]
            - right_hand_pos: 右手相对于参考坐标系的位置，单位为米 [x,y,z]
            - right_hand_quat: 右手相对于参考坐标系的姿态，四元数[x,y,z,w]
            - result: "success": 成功, "fail_motor": 电机错误, "fail_invalid_cmd": 非法指令
        """
        return self.send_command("request_get_ub_manip_ee_pose", {}, wait_for_response=True)

    def set_walk_vel(self, x=0.0, y=0.0, yaw=0.0):
        """
        8. 控制行走 (request_set_walk_vel)
        
        在移动操作模式下，通过此协议控制机器人行走。
        注意：在全身操作模式下，此协议接口无效。
        
        :param x: 前进后退速度比值，取值范围[-1, 1]
        :param y: 横向行走速度比值，取值范围[-1, 1] 
        :param yaw: 旋转角速度比值，取值范围[-1, 1]
        :return: 成功执行则返回None，失败时返回错误信息
                "fail_imu": IMU错误, "fail_motor": 电机错误
        """
        # 参数验证
        if not (-1.0 <= x <= 1.0) or not (-1.0 <= y <= 1.0) or not (-1.0 <= yaw <= 1.0):
            print("Error: 速度比值必须在 [-1, 1] 范围内")
            return None
            
        data = {
            "x": x,
            "y": y,
            "yaw": yaw
        }
        
        # 成功执行时无返回，只有失败时才有响应
        result = self.send_command("request_set_walk_vel", data, wait_for_response=True, timeout=1.0)
        
        # 如果有返回值说明执行失败
        if result and "result" in result:
            return result["result"]
        
        # 无返回值表示成功
        return None

    def get_joint_state(self):
        """
        9. 获取关节状态 (request_get_joint_state)
        
        此请求用于获取机器人的各个关节状态。
        
        :return: 包含关节状态信息的字典，包括：
            - names: 各个关节的名称列表
            - q: 各个关节的位置列表
            - dq: 各个关节的速度列表  
            - tau: 各个关节的扭矩列表
            - result: "success": 成功, "fail_not_data": 无数据
        """
        return self.send_command("request_get_joint_state", {}, wait_for_response=True)

    def get_imu_data(self):
        """
        10. 获取IMU数据 (request_get_imu_data)
        
        获取机器人IMU传感器的姿态和运动数据。
        
        :return: 包含IMU数据的字典，包括：
            - result: "success": 成功, "fail_no_data": 无数据
            - euler: 欧拉角 [roll, pitch, yaw] 单位：度
            - acc: 加速度 [x, y, z] 单位：m/s²
            - gyro: 陀螺仪角速度 [x, y, z] 单位：rad/s
            - quat: 四元数 [w, x, y, z]
        """
        return self.send_command("request_get_imu_data", {}, wait_for_response=True)

    