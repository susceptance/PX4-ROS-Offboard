#! /usr/bin/env python3

# 젯슨 나노(드론)에서 MAVROS를 통해 들어온 GPS 정보를 서버로 전송
import paho.mqtt.client as mqtt
import json
import rospy
from sensor_msgs.msg import NavSatFix
from mqtt_json2.msg import drone_gps
#drone_flag = False
lat = 0
lon = 0

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)


def on_disconnect(client, userdata, flags, rc=0):
    print(str(rc))


def on_publish(client, userdata, mid):
    print("In on_pub callback mid= ", mid)


def mav_gps_callback(data):
    global lat,lon
    lat=data.latitude
    lat=str(lat)
    lon=data.longitude
    lon=str(lon)
    drone_flag=data.drone_flag
    print(drone_flag)
    client = mqtt.Client()
    client.username_pw_set("a1","abcd")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    client.connect('27.96.135.70', 1883)
    client.loop_start()
    client.publish('Location/DRONE', json.dumps({"DroneFlag":drone_flag,"DroneLat":lat,"DroneLong":lon}), 1) # 사용자에게 전송
    #rospy.loginfo('I heard %f', data.latitude)
    #rospy.loginfo('I heard %f', data.longitude)


def main():
    rospy.init_node('mqtt_json2',anonymous=True)
    drone_gps_msg = drone_gps()
    client = mqtt.Client()
    client.username_pw_set("a1","abcd")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    client.connect('27.96.135.70', 1883)
    client.loop_start()
    mav_gps_sub = rospy.Subscriber("/drone_gps", drone_gps, mav_gps_callback) # APF노드로부터 받아오기
    rate = rospy.Rate(10) 
    #gps_msg = user_msg()
    # 새로운 클라이언트 생성
    '''client = mqtt.Client()
    # 콜백 함수 설정 on_connect(브로커에 접속), on_disconnect(브로커에 접속중료), on_publish(메세지 발행)
    client.username_pw_set("a1","abcd")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    # address : localhost, port: 1883 에 연결
    client.connect('27.96.135.70', 1883)
    # client.connect('localhost', 1883)'''
    #client.loop_start()
    # common topic 으로 메세지 발행
    #client.publish('Location/USER', json.dumps({"lat": "128.16.25"}), 1)
    #client.publish('Location/USER', json.dumps({"lat": lat, "long": lon}), 1)
    #client.loop_stop()
    # 연결 종료
    #client.disconnect()
    rospy.spin()
    client.loop_stop()
    client.disconnect()

    '''while not rospy.is_shutdown():
        #client.loop_start()
        rospy.spinOnce()
        client.publish('Location/USER', json.dumps({"lat": lat, "long": lon}), 1)
        #client.loop_stop()
        print("run")
        rate.sleep()

        client.loop_stop()'''

if __name__=='__main__':
    main()
