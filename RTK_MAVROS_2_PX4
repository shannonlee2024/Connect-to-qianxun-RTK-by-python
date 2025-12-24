#!/usr/bin/env python3
import rospy
import socket
import base64
import time
from mavros_msgs.msg import RTCM  # 修改为 MAVROS 标准消息

# === 配置 (保持你之前的成功配置) ===
NTRIP_USER = 'qxvwbt00682xx'
NTRIP_PASS = '8b8c5xx'
NTRIP_HOST = '203.107.45.154'
NTRIP_PORT = 8002
MOUNTPOINT = 'RTCM32_GGB'

# 你的欺骗坐标 (千寻需要知道你在哪才能发差分数据)
LAT = 31.5xxxx
LON = 120.39xxxx

def get_gga(lat, lon):
    lat_deg = int(lat); lat_min = (lat-lat_deg)*60
    lon_deg = int(lon); lon_min = (lon-lon_deg)*60
    ns = 'N' if lat>0 else 'S'; ew = 'E' if lon>0 else 'W'
    raw = "GPGGA,%s,%02d%07.4f,%s,%03d%07.4f,%s,1,10,1.0,20.0,M,0.0,M,," % (
        time.strftime("%H%M%S", time.gmtime()), lat_deg, lat_min, ns, lon_deg, lon_min, ew)
    chk = 0
    for c in raw: chk ^= ord(c)
    return "$%s*%02X\r\n" % (raw, chk)

def main():
    rospy.init_node('ntrip_to_mavros_bridge')
    
    # 修改话题为 MAVROS 接收 RTCM 的标准话题
    pub = rospy.Publisher('/mavros/gps_rtk/send_rtcm', RTCM, queue_size=10)
    
    rospy.loginfo("Starting NTRIP to MAVROS Bridge...")
    
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((NTRIP_HOST, NTRIP_PORT))
            
            # 发送登录头
            auth = base64.b64encode((NTRIP_USER+':'+NTRIP_PASS).encode()).decode()
            headers = "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP NTRIPClient/1.0\r\nAuthorization: Basic %s\r\n\r\n" % (MOUNTPOINT, auth)
            s.sendall(headers.encode())
            
            # 发送位置 (必须发送，否则服务器会断开或不发数据)
            s.sendall(get_gga(LAT, LON).encode())
            
            rospy.loginfo("Connected to Qianxun! Sending RTCM to MAVROS...")
            
            while not rospy.is_shutdown():
                data = s.recv(2048)
                if not data:
                    rospy.logwarn("No data received, reconnecting...")
                    break
                
                # 构造 MAVROS 需要的 RTCM 消息
                rtcm_msg = RTCM()
                rtcm_msg.header.stamp = rospy.Time.now()
                rtcm_msg.header.frame_id = "base_link"
                rtcm_msg.data = data  # 直接放入原始二进制数据
                
                pub.publish(rtcm_msg)
                
        except Exception as e:
            rospy.logwarn(f"Connection error: {e}, retrying in 3s...")
            time.sleep(3)
        finally:
            s.close()

if __name__ == '__main__':
    main()
