#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import base64
import time
import threading
import subprocess
import rosnode

# 导入消息类型
from rtcm_msgs.msg import Message as RTCM_RTCM
from mavros_msgs.msg import RTCM as MAVROS_RTCM

# === 配置信息 ===
NTRIP_USER = 'qxvwbt0068xxx'
NTRIP_PASS = '8b8cxxx'
NTRIP_HOST = '203.107.45.154'
NTRIP_PORT = 8002
MOUNTPOINT = 'AUTO'

TCP_IP = '127.0.0.1'
TCP_PORT = 3503

LAT = 31.459567
LON = 120.435651

class RTKManagerPro:
    def __init__(self):
        rospy.init_node('rtk_manager_pro_node', anonymous=True)
        rospy.loginfo("Initializing RTK Manager Pro...")

        self.clients = []
        self.lock = threading.Lock()
        
        self.server_thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.server_thread.start()
        
        time.sleep(2)
        
        # 启动依赖 (Ublox)
        #self.ensure_dependencies()
        
        self.ublox_pub = rospy.Publisher('/ublox_gps/rtcm', RTCM_RTCM, queue_size=10)
        self.mavros_pub = rospy.Publisher('/mavros/gps_rtk/send_rtcm', MAVROS_RTCM, queue_size=10)
        
        rospy.loginfo("✅ All systems go!")

    # def ensure_dependencies(self):
    #     try:
    #         running_nodes = rosnode.get_node_names()
    #     except Exception:
    #         rospy.logerr("ROS Master not found!")
    #         return

    #     if not any('/ublox_gps' in node for node in running_nodes):
    #         rospy.loginfo("🚀 Launching Ublox Driver on /dev/ublox...")
    #         subprocess.Popen(
    #             ["roslaunch", "ublox_driver", "ublox_driver.launch", "device:=/dev/ublox"], 
    #             stdout=subprocess.DEVNULL, 
    #             stderr=subprocess.DEVNULL
    #         )
    #         time.sleep(2)
    #     else:
    #         rospy.loginfo("✅ Ublox Driver is already running.")

    def run_tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((TCP_IP, TCP_PORT))
            server.listen(5)
            rospy.loginfo(f"✅ TCP Server is listening on port {TCP_PORT}")
        except Exception as e:
            rospy.logerr(f"❌ TCP Server bind failed: {e}")
            return

        while not rospy.is_shutdown():
            try:
                server.settimeout(1.0)
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                with self.lock:
                    self.clients.append(conn)
                rospy.loginfo(f"📱 Driver connected via Socket: {addr}")
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Server error: {e}")
                break

    def get_gga(self, lat, lon):
        lat_deg = int(lat); lat_min = (lat-lat_deg)*60
        lon_deg = int(lon); lon_min = (lon-lon_deg)*60
        ns = 'N' if lat>0 else 'S'; ew = 'E' if lon>0 else 'W'
        raw = "GPGGA,%s,%02d%07.4f,%s,%03d%07.4f,%s,1,10,1.0,20.0,M,0.0,M,," % (
            time.strftime("%H%M%S", time.gmtime()), lat_deg, lat_min, ns, lon_deg, lon_min, ew)
        chk = 0
        for c in raw: chk ^= ord(c)
        return "$%s*%02X\r\n" % (raw, chk)

    def run_ntrip(self):
        while not rospy.is_shutdown():
            try:
                s_ntrip = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s_ntrip.settimeout(10)
                s_ntrip.connect((NTRIP_HOST, NTRIP_PORT))
                
                auth = base64.b64encode((NTRIP_USER+':'+NTRIP_PASS).encode()).decode()
                headers = "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic %s\r\n\r\n" % (MOUNTPOINT, auth)
                s_ntrip.sendall(headers.encode())
                s_ntrip.sendall(self.get_gga(LAT, LON).encode())
                
                rospy.loginfo("🛰️  NTRIP Connected! RTK data flowing...")
                
                while not rospy.is_shutdown():
                    # 这里依然接收大块数据，以保证网络效率
                    data = s_ntrip.recv(2048)
                    if not data: break
                    
                    now = rospy.Time.now()
                    
                    # 1. 发给 Ublox (Ublox 驱动通常能处理大包，直接发)
                    m1 = RTCM_RTCM(); m1.header.stamp = now; m1.message = data
                    self.ublox_pub.publish(m1)
                    
                    # === 关键修改开始 ===
                    # 2. 发给 MAVROS (必须切片！MAVLink最大只支持180字节)
                    # 我们这里设置 180 字节的切片上限
                    chunk_size = 180
                    if len(data) > chunk_size:
                        # 如果数据太长，循环切片发送
                        for i in range(0, len(data), chunk_size):
                            chunk = data[i : i + chunk_size]
                            m2 = MAVROS_RTCM()
                            m2.header.stamp = now
                            m2.data = chunk
                            self.mavros_pub.publish(m2)
                            # 稍微sleep极短时间防止瞬间堵塞ROS队列(可选)
                            # time.sleep(0.001) 
                    else:
                        # 如果数据很短，直接发送
                        m2 = MAVROS_RTCM()
                        m2.header.stamp = now
                        m2.data = data
                        self.mavros_pub.publish(m2)
                    # === 关键修改结束 ===

                    # 3. 发给 TCP 客户端
                    with self.lock:
                        for conn in self.clients[:]:
                            try: conn.sendall(data)
                            except: self.clients.remove(conn)
                                
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logwarn(f"NTRIP Reconnecting... ({e})")
                time.sleep(3)
            finally:
                s_ntrip.close()

if __name__ == '__main__':
    try:
        manager = RTKManagerPro()
        manager.run_ntrip()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import base64
import time
import threading
import subprocess
import rosnode

# 导入消息类型
from rtcm_msgs.msg import Message as RTCM_RTCM
from mavros_msgs.msg import RTCM as MAVROS_RTCM

# === 配置信息 ===
NTRIP_USER = 'qxvwbt0068276'
NTRIP_PASS = '8b8c5a5'
NTRIP_HOST = '203.107.45.154'
NTRIP_PORT = 8002
MOUNTPOINT = 'AUTO'

TCP_IP = '127.0.0.1'
TCP_PORT = 3503

LAT = 31.459567
LON = 120.435651

class RTKManagerPro:
    def __init__(self):
        rospy.init_node('rtk_manager_pro_node', anonymous=True)
        rospy.loginfo("Initializing RTK Manager Pro...")

        self.clients = []
        self.lock = threading.Lock()
        
        self.server_thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.server_thread.start()
        
        time.sleep(2)
        
        # 启动依赖 (Ublox)
        self.ensure_dependencies()
        
        self.ublox_pub = rospy.Publisher('/ublox_gps/rtcm', RTCM_RTCM, queue_size=10)
        self.mavros_pub = rospy.Publisher('/mavros/gps_rtk/send_rtcm', MAVROS_RTCM, queue_size=10)
        
        rospy.loginfo("✅ All systems go!")

    def ensure_dependencies(self):
        try:
            running_nodes = rosnode.get_node_names()
        except Exception:
            rospy.logerr("ROS Master not found!")
            return

        if not any('/ublox_gps' in node for node in running_nodes):
            rospy.loginfo("🚀 Launching Ublox Driver on /dev/ublox...")
            subprocess.Popen(
                ["roslaunch", "ublox_driver", "ublox_driver.launch", "device:=/dev/ublox"], 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            time.sleep(2)
        else:
            rospy.loginfo("✅ Ublox Driver is already running.")

    def run_tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((TCP_IP, TCP_PORT))
            server.listen(5)
            rospy.loginfo(f"✅ TCP Server is listening on port {TCP_PORT}")
        except Exception as e:
            rospy.logerr(f"❌ TCP Server bind failed: {e}")
            return

        while not rospy.is_shutdown():
            try:
                server.settimeout(1.0)
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                with self.lock:
                    self.clients.append(conn)
                rospy.loginfo(f"📱 Driver connected via Socket: {addr}")
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Server error: {e}")
                break

    def get_gga(self, lat, lon):
        lat_deg = int(lat); lat_min = (lat-lat_deg)*60
        lon_deg = int(lon); lon_min = (lon-lon_deg)*60
        ns = 'N' if lat>0 else 'S'; ew = 'E' if lon>0 else 'W'
        raw = "GPGGA,%s,%02d%07.4f,%s,%03d%07.4f,%s,1,10,1.0,20.0,M,0.0,M,," % (
            time.strftime("%H%M%S", time.gmtime()), lat_deg, lat_min, ns, lon_deg, lon_min, ew)
        chk = 0
        for c in raw: chk ^= ord(c)
        return "$%s*%02X\r\n" % (raw, chk)

    def run_ntrip(self):
        while not rospy.is_shutdown():
            try:
                s_ntrip = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s_ntrip.settimeout(10)
                s_ntrip.connect((NTRIP_HOST, NTRIP_PORT))
                
                auth = base64.b64encode((NTRIP_USER+':'+NTRIP_PASS).encode()).decode()
                headers = "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic %s\r\n\r\n" % (MOUNTPOINT, auth)
                s_ntrip.sendall(headers.encode())
                s_ntrip.sendall(self.get_gga(LAT, LON).encode())
                
                rospy.loginfo("🛰️  NTRIP Connected! RTK data flowing...")
                
                while not rospy.is_shutdown():
                    # 这里依然接收大块数据，以保证网络效率
                    data = s_ntrip.recv(2048)
                    if not data: break
                    
                    now = rospy.Time.now()
                    
                    # 1. 发给 Ublox (Ublox 驱动通常能处理大包，直接发)
                    m1 = RTCM_RTCM(); m1.header.stamp = now; m1.message = data
                    self.ublox_pub.publish(m1)
                    
                    # === 关键修改开始 ===
                    # 2. 发给 MAVROS (必须切片！MAVLink最大只支持180字节)
                    # 我们这里设置 180 字节的切片上限
                    chunk_size = 180
                    if len(data) > chunk_size:
                        # 如果数据太长，循环切片发送
                        for i in range(0, len(data), chunk_size):
                            chunk = data[i : i + chunk_size]
                            m2 = MAVROS_RTCM()
                            m2.header.stamp = now
                            m2.data = chunk
                            self.mavros_pub.publish(m2)
                            # 稍微sleep极短时间防止瞬间堵塞ROS队列(可选)
                            # time.sleep(0.001) 
                    else:
                        # 如果数据很短，直接发送
                        m2 = MAVROS_RTCM()
                        m2.header.stamp = now
                        m2.data = data
                        self.mavros_pub.publish(m2)
                    # === 关键修改结束 ===

                    # 3. 发给 TCP 客户端
                    with self.lock:
                        for conn in self.clients[:]:
                            try: conn.sendall(data)
                            except: self.clients.remove(conn)
                                
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logwarn(f"NTRIP Reconnecting... ({e})")
                time.sleep(3)
            finally:
                s_ntrip.close()

if __name__ == '__main__':
    try:
        manager = RTKManagerPro()
        manager.run_ntrip()
    except rospy.ROSInterruptException:
        pass
