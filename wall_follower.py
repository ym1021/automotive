# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

speed_pub = rospy.Publisher('/vesc/commands/motor/speed', Float64, queue_size=1)
position_pub = rospy.Publisher('/vesc/commands/servo/position', Float64, queue_size=1)

state_ = 0

state_dict_ = {
    0: 'away from the wall',
    1: 'going to wall',
    2: 'following the wall',
}

regions = None

def degTorad(deg):
	rad_diff = 0.5304
	rad = deg * (3.14/180)
	return rad + rad_diff

def callback(data):
    global state_, regions
    regions = {
        'right': min(data.ranges[60:89]),
        'fright': min(data.ranges[30:59]),
        'front': min(min(data.ranges[0:29]), min(data.ranges[330:359])),
        'fleft': min(data.ranges[300:329]),
        'left': min(data.ranges[270:299]),
    }
    
    #벽과의 최소 거리 설정
    distance = 2.0
    
    #차량의 상태 최신화
    #상태 값, 0: away from the wall , 1: going to wall, 2:following the wall
    take_action(distance)

    print("state_", state_)
    
    #차량은 왼쪽에 존재하는 벽을 추종함
    #차량의 현재 상태에 따라 행동 결정
    #차량이 벽에서 멀어지고 있을 때, 왼쪽으로 주행
    #차량이 벽으로 다가가고 있을 때, 오른쪽으로 주행
    #차량이 벽을 잘 따라가고 있을 때, 직진 주행
    if state_ == 0: #away from the wall
        turn_left()
    elif state_ == 1: #going to wall
        turn_right()
    elif state_ == 2: #following the wall
        follow_the_wall()
        pass
    else:
        rospy.logerr('Unknown state!')

def change_state(state):
    global state_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action(distance):
    global regions
    d = distance

    """
    전방 영역의 값, 좌전방 영역의 값, 우전방 영역의 값이 모두 distance보다 큰 경우, 
    벽에서 멀어지고 있는 상태이므로 상태를 0(away from the wall)으로 변경
    """
    """
    전방 영역의 값이 distance보다 작고, 좌전방 영역의 값이 distance보다 크고, 우전방 영역의 값이 distance보다 큰 경우,
    전방에 물체또는 벽이 있다고 판단되므로 going to wall로 변경
    """
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        # state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        # state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        # state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        # state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        # state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        # state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        # state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        # state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def turn_left():
    speed = 7000
    position = degTorad(-22)

    speed_pub.publish(speed)
    position_pub.publish(position)

def turn_right():
    position = degTorad(22)

    position_pub.publish(position)

def follow_the_wall():
    speed = 7000
    position = degTorad(22)

    speed_pub.publish(speed)
    position_pub.publish(position)

if __name__ == '__main__':
    rospy.init_node("WallFollwer")
    sub = rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
