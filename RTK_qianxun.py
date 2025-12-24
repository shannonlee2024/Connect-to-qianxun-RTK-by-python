#!/usr/bin/env python3
# 文件名: run_rtk.py
import rospy
import socket
import base64
import time
from rtcm_msgs.msg import Message # 确保安装了 ros-noetic-rtcm-msgs

# === 配置 ===
NTRIP_USER = 'qxvwbt00682xx'
NTRIP_PASS = '8b8c5ax' # 你的密码
NTRIP_HOST = '203.107.45.154'
NTRIP_PORT = 8002
MOUNTPOINT = 'RTCM32_GGB'
# 你的欺骗坐标
LAT = 31.x
LON = 120.x

def get_gga(lat, lon):
    # 简单的GGA生成器
    lat_deg = int(lat); lat_min = (lat-lat_deg)*60
    lon_deg = int(lon); lon_min = (lon-lon_deg)*60
    ns = 'N' if lat>0 else 'S'; ew = 'E' if lon>0 else 'W'
    raw = "GPGGA,%s,%02d%07.4f,%s,%03d%07.4f,%s,1,10,1.0,20.0,M,0.0,M,," % (
        time.strftime("%H%M%S", time.gmtime()), lat_deg, lat_min, ns, lon_deg, lon_min, ew)
    chk = 0
    for c in raw: chk ^= ord(c)
    return "$%s*%02X\r\n" % (raw, chk)

def main():
    rospy.init_node('qianxun_client')
    pub = rospy.Publisher('/ublox_gps/rtcm', Message, queue_size=10)
    
    while not rospy.is_shutdown():
        try:
            s = socket.socket()
            s.connect((NTRIP_HOST, NTRIP_PORT))
            
            # 发送登录头 (模拟 User-Agent)
            auth = base64.b64encode((NTRIP_USER+':'+NTRIP_PASS).encode()).decode()
            headers = "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP NTRIPClient/1.0\r\nAuthorization: Basic %s\r\n\r\n" % (MOUNTPOINT, auth)
            s.sendall(headers.encode())
            
            # 发送位置 (欺骗)
            s.sendall(get_gga(LAT, LON).encode())
            
            rospy.loginfo("Connected to Qianxun! Streaming RTCM...")
            
            while not rospy.is_shutdown():
                data = s.recv(1024)
                if not data: break
                # 转发给 ROS
                msg = Message()
                msg.message = data
                msg.header.stamp = rospy.Time.now()
                pub.publish(msg)
        except Exception as e:
            rospy.logwarn(f"Connection lost: {e}, retrying...")
            time.sleep(3)

if __name__ == '__main__':
    main()
