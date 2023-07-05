import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
def callback(data):	
    speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
    position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
    a = min(data.ranges[0:30])
    b = min(data.ranges[330:359])
    dist = min(a, b)
    print(dist)
    if dist <= 1.5:
      print("stop")
      speed.publish(0)
    else:
      print("go")
      speed.publish(5000)
    
    

if __name__ == '__main__':
    try:
        rospy.init_node("ObjectAvoid")
        sub = rospy.Subscriber("/lidar2D", LaserScan, callback)
        rospy.spin()
    except:
        print("Error occured.")