#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose

from flask import Flask, render_template, request
app = Flask(__name__)

keyBindings = {
    0: (0, 0, 0),
    87: ((2**15)/4, 0, 0),
    65: (0, (2**15)/4, 0,),
    83: (-(2**15)/4, 0, 0),
    68: (0, -(2**15)/4, 0),
    81: (0, 0, -0.1),
    69: (0, 0, 0.1),
}

buttonBindings = {
    0: (0, 0, 0),
    "left": ((2**15)/4, 0, 0),
    "up": (0, (2**15)/4, 0,),
    "right": (-(2**15)/4, 0, 0),
    "down": (0, -(2**15)/4, 0),
    81: (0, 0, -0.1),
    69: (0, 0, 0.1),
}


@app.route('/')
def hello():
    return render_template('index.html')


@app.route('/mouse', methods=['POST'])
def mouse():
    if request.method == 'POST':
        print(j := request.get_json())
        pose = Pose()
        pose.position.x = j['x']
        pose.position.y = j['y']
        pub2.publish(pose)
        return 'Sucesss', 200


@app.route('/key', methods=['POST'])
def key():
    if request.method == 'POST':
        print(j := request.get_json())
        if j['key'] in keyBindings:
            twist.linear.x, twist.linear.y, twist.angular.z = keyBindings[j['key']]
            pub1.publish(twist)
        return 'Sucesss', 200


@app.route('/button', methods=['POST'])
def button():
    if request.method == 'POST':
        print(j := request.get_json())
        if j['button'] in buttonBindings:
            twist.linear.x, twist.linear.y, twist.angular.z = buttonBindings[j['button']]
            pub1.publish(twist)
        return 'Sucesss', 200


if __name__ == '__main__': 
    rospy.init_node('test_flask')
    rospy.loginfo('Node started')

    pub1 = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('position', Pose, queue_size=10)
    twist = Twist()

    app.run(debug=True, host='0.0.0.0')
