#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import base64
import time
import threading
import subprocess
import rosnode
import rosgraph  # 用于检测 Master 状态
import os

# 导入消息类型
from rtcm_msgs.msg import Message as RTCM_RTCM
from mavros_msgs.msg import RTCM as MAVROS_RTCM

# === 配置信息 ===
NTRIP_USER = 'qxvwbt0068xxx'
NTRIP_PASS = '8b8c5xx'
NTRIP_HOST = '203.107.45.154'
NTRIP_PORT = 8002
MOUNTPOINT = 'RTCM32_GGB'

TCP_IP = '127.0.0.1'
TCP_PORT = 3503

LAT = 31.566689
LON = 120.3959739

class RTKManagerPro:
    def __init__(self):
        # 1. 首先检查并启动 roscore
        self.ensure_roscore()

        # 2. 初始化 ROS 节点
        rospy.init_node('rtk_manager_pro_node', anonymous=True)
        rospy.loginfo("RTK Manager Pro is starting up...")

        self.clients = []
        self.lock = threading.Lock()
        
        # 3. 启动 TCP 服务器线程 (3503 端口)
        self.server_thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.server_thread.start()
        
        # 4. 等待服务器就绪
        time.sleep(2)
        
        # 5. 启动 MAVROS 和 Ublox 驱动
        self.ensure_dependencies()
        
        # 6. 初始化发布者
        self.ublox_pub = rospy.Publisher('/ublox_gps/rtcm', RTCM_RTCM, queue_size=10)
        self.mavros_pub = rospy.Publisher('/mavros/gps_rtk/send_rtcm', MAVROS_RTCM, queue_size=10)
        
        rospy.loginfo("✅ All systems are running and synchronized.")

    def ensure_roscore(self):
        """ 检查 roscore 是否运行，如果没有则启动它 """
        print("🔍 Checking ROS Master...")
        if not rosgraph.is_master_online():
            print("⚠️  ROS Master (roscore) is not running. Launching it now...")
            # 使用 Popen 在后台启动 roscore
            subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # 给 roscore 一点启动时间，直到 master 在线
            timeout = 10
            start_time = time.time()
            while not rosgraph.is_master_online():
                time.sleep(1)
                if time.time() - start_time > timeout:
                    print("❌ Failed to start roscore within 10 seconds.")
                    exit(1)
            print("✅ roscore started successfully.")
        else:
            print("✅ ROS Master is already online.")

    def ensure_dependencies(self):
        """ 检查并启动 MAVROS 和 UBLOX 驱动 """
        try:
            running_nodes = rosnode.get_node_names()
        except Exception:
            return

        # 检查 MAVROS
        if not any('/mavros' in node for node in running_nodes):
            rospy.loginfo("🚀 Launching MAVROS...")
            subprocess.Popen(["roslaunch", "mavros", "px4.launch"], 
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(2)

        # 检查 Ublox Driver
        if not any('/ublox_gps' in node for node in running_nodes):
            rospy.loginfo("🚀 Launching Ublox Driver...")
            # 此时 TCP 3503 端口已经开启，驱动启动后能立即连接
            subprocess.Popen(["roslaunch", "ublox_driver", "ublox_driver.launch"], 
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run_tcp_server(self):
        """ TCP 服务逻辑 """
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((TCP_IP, TCP_PORT))
            server.listen(5)
            rospy.loginfo(f"📡 TCP Server listening on {TCP_IP}:{TCP_PORT}")
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
                rospy.loginfo(f"📱 Driver connected to 3503: {addr}")
            except:
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
        """ NTRIP 主循环 """
        while not rospy.is_shutdown():
            try:
                s_ntrip = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s_ntrip.settimeout(10)
                s_ntrip.connect((NTRIP_HOST, NTRIP_PORT))
                
                auth = base64.b64encode((NTRIP_USER+':'+NTRIP_PASS).encode()).decode()
                headers = "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP Client\r\nAuthorization: Basic %s\r\n\r\n" % (MOUNTPOINT, auth)
                s_ntrip.sendall(headers.encode())
                s_ntrip.sendall(self.get_gga(LAT, LON).encode())
                
                rospy.loginfo("🛰️  NTRIP Connected! Streaming RTK data...")
                
                while not rospy.is_shutdown():
                    data = s_ntrip.recv(2048)
                    if not data: break
                    
                    now = rospy.Time.now()
                    # 分发到 Topic
                    m1 = RTCM_RTCM(); m1.header.stamp = now; m1.message = data
                    self.ublox_pub.publish(m1)
                    
                    m2 = MAVROS_RTCM(); m2.header.stamp = now; m2.data = data
                    self.mavros_pub.publish(m2)
                    
                    # 分发到 TCP 客户端 (ublox_driver)
                    with self.lock:
                        for conn in self.clients[:]:
                            try: conn.sendall(data)
                            except: self.clients.remove(conn)
                                
            except Exception as e:
                if not rospy.is_shutdown():
                    rospy.logwarn(f"NTRIP Connection retry... ({e})")
                time.sleep(3)
            finally:
                s_ntrip.close()

if __name__ == '__main__':
    try:
        manager = RTKManagerPro()
        manager.run_ntrip()
    except rospy.ROSInterruptException:
        pass
