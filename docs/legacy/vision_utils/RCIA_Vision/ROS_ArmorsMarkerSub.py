# import rospy
# import math
# from visualization_msgs.msg import MarkerArray

# def msg_quaternion_to_euler(msg):
#     # print(msg.split("\n"))

#     T = "\n\n"
#     for line in str(msg).split("\n"):
#         if "ns:" in line:
#             ns = line.split(": ")[1].replace('"', '')
#         elif "id:" in line and "frame" not in line:
#             if ns == "armors":
#                 id = int(line.split(": ")[1]) + 1
#             elif ns == "chassisCentral":
#                 ns = ns[:-1]
#                 id = 0

#         if "x:" in line:
#             x = float(line.split(": ")[1])
#         elif "y:" in line:
#             y = float(line.split(": ")[1])
#         elif "z:" in line:
#             z = float(line.split(": ")[1])
#         elif "w:" in line:
#             w = float(line.split(": ")[1])

#         if "scale:" in line:
#             # print(x, y, z, w)
            
#             T += f"Type: {ns + str(id)}"
#             t0 = 2.0 * (w * x + y * z)
#             t1 = 1.0 - 2.0 * (x * x + y * y)
#             roll = math.atan2(t0, t1)

#             t2 = 2.0 * (w * y - z * x)
#             t2 = 1.0 if t2 > 1.0 else t2
#             t2 = -1.0 if t2 < -1.0 else t2
#             pitch = math.asin(t2)

#             t3 = 2.0 * (w * z + x * y)
#             t4 = 1.0 - 2.0 * (y * y + z * z)
#             yaw = math.atan2(t3, t4)

#             roll_deg = math.degrees(roll)
#             pitch_deg = math.degrees(pitch)
#             yaw_deg = math.degrees(yaw)

#             T += f" |Pitch: {round(pitch_deg, 3)} °".ljust(15, " ")
#             T += f" |Yaw: {round(yaw_deg, 3)} °".ljust(15, " ")
#             T += f" |Roll: {round(roll_deg, 3)} °".ljust(15, " ") + "\n"
        
#         print(T)
            

# # 回调函数，当接收到MarkerArray消息时调用
# def marker_array_callback(msg):
#     # 处理接收到的MarkerArray消息
#     # rospy.loginfo("Received MarkerArray message")

#     # rospy.loginfo(msg)
#     msg_quaternion_to_euler(msg)


# if __name__ == '__main__':
#     rospy.init_node('marker_array_subscriber')

#     # 订阅MarkerArray消息
#     rospy.Subscriber('/armor_markers', MarkerArray, marker_array_callback)

#     rospy.loginfo("Subscribed to /armor_markers")

#     # 保持节点运行
#     rospy.spin()
