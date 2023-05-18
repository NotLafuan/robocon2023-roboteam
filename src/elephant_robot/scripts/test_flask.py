#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

from flask import Flask, render_template, request
app = Flask(__name__)

moveBindings = {
    0: (0, 0, 0),
    87: ((2**15), 0, 0),
    65: (0, (2**15), 0,),
    83: (-(2**15), 0, 0),
    68: (0, -(2**15), 0),
    81: (0, 0, 2500*2),
    69: (0, 0, -2500*2),
}


@app.route('/')
def hello():
    return render_template('index.html')


@app.route('/mouse', methods=['POST'])
def mouse():
    print('mouse')
    if request.method == 'POST':
        print(request.get_json())
        return 'Sucesss', 200


@app.route('/key', methods=['POST'])
def key():
    if request.method == 'POST':
        print(j := request.get_json())
        if j['key'] in moveBindings:
            twist.linear.x, twist.linear.y, twist.angular.z = moveBindings[j['key']]
            pub.publish(twist)
        return 'Sucesss', 200


if __name__ == '__main__':
    rospy.init_node('test_flask')
    rospy.loginfo('Node started')

    pub = rospy.Publisher('/elephant/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    app.run(debug=True, host='0.0.0.0')
