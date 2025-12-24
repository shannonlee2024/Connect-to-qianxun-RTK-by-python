# Connect-to-qianxun-RTK-by-python
Connect to qianxun RTK-by python

RUN :
roslaunch ublox_driver.launch 
and then
python3 RTK_qianxun.py 

if succeed：
print：[INFO] [1766569156.009442]: Connected to Qianxun! Streaming RTCM...


rostopic hz /ublox_gps/rtcm

average rate: 0.995
	min: 0.781s max: 1.181s std dev: 0.10448s window: 37



ALL in one：

first install https://github.com/JIAHAO-FUHUA/gnss_driver#  Thanks jiahao

run ：
roscore
python3 RTK_bridge_MAVROS_TCP_ROSTOPIC.py 
