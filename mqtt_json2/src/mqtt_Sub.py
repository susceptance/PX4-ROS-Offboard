#! /usr/bin/env python3

#사용자로부터 드론으로 받아오기.
import paho.mqtt.client as mqtt
import rospy
from mqtt_json2.msg import user_gps

UserAlt,UserFlag,UserLat,UserLong=0,0,0,0

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)


def on_disconnect(client, userdata, flags, rc=0):
    print(str(rc))


def on_subscribe(client, userdata, mid, granted_qos):
    print("subscribed: " + str(mid) + " " + str(granted_qos))


def on_message(client, userdata, msg):
    global User, UserAlt,UserFlag,UserLat,UserLong
    User = msg.payload.decode("utf-8")
    print(userdata)
    print(str(msg.payload.decode("utf-8")))
    keys = []
    values = []
    data_list = msg.payload.decode("utf-8").split(",")

    for data in data_list:
        pair = data.split(":")
        keys.append(pair[0])
        values.append(pair[1])
    print(keys)
    print(values)
    value=[]
    aaaa=[]
    value=values[3]
    UserFlag=values[0]
    UserLat=value[1]
    UserLong=value[2]
    value=value.split("}")
    #print("aaaa")
    
    UserFlag=values[0]
    UserLat=values[1]
    UserLong=values[2]
    UserAlt=value[0]
    
    UserFlag=int(UserFlag)
    UserLat=float(UserLat)
    UserLong=float(UserLong)
    UserAlt=float(UserAlt)
    
    User_GPS=[]
    User_GPS.append(UserFlag)
    User_GPS.append(UserLat)
    User_GPS.append(UserLong)
    User_GPS.append(UserAlt)
    
    gps_msg.latitude = UserLat
    gps_msg.longitude = UserLong
    gps_msg.altitude = UserAlt
    gps_msg.user_flag = UserFlag
    gps_pub.publish(gps_msg)
    rospy.loginfo(gps_msg)


    print("=================================")
    # print(User_GPS)
    
    

def main():
    global gps_pub, gps_msg
    rospy.init_node('mqtt_json2', anonymous=True)
    gps_pub = rospy.Publisher('/user_gps_pos', user_gps, queue_size=10) # APF노드로 전송
    gps_msg = user_gps()
    rate = rospy.Rate(10) # 10hz
    # 새로운 클라이언트 생성
    client = mqtt.Client()
    # 콜백 함수 설정 on_connect(브로커에 접속), on_disconnect(브로커에 접속중료), on_subscribe(topic 구독),
    # on_message(발행된 메세지가 들어왔을 때)
    client.username_pw_set("a2","abcd")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    # address : localhost, port: 1883 에 연결
    client.connect('27.96.135.70', 1883)
    # client.connect('localhost', 1883)
    # common topic 으로 메세지 발행
    #client.subscribe('Location/USER', 1)
    #client.loop_start()
    client.subscribe('Location/USER', 1)
    '''while not rospy.is_shutdown():
        gps_msg.latitude = UserLat
        gps_msg.longitude = UserLong
        gps_msg.altitude = UserAlt
        gps_msg.flag = UserFlag
        gps_pub.publish(gps_msg)
        rospy.loginfo(gps_msg)
        rate.sleep()
    client.loop_forever()'''
    '''gps_msg.latitude = UserLat
    gps_msg.longitude = UserLong
    gps_msg.altitude = UserAlt
    gps_msg.flag = UserFlag
    gps_pub.publish(gps_msg)
    rospy.loginfo(gps_msg)
    rate.sleep()
    client.loop_forever()'''
    client.loop_forever()
    #client.loop_stop()
    #rospy.spin()

if __name__=='__main__':
    main()
