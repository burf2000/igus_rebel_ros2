#!/usr/bin/env python3
import rclpy
from threading import Thread
from flask import Flask, request, jsonify
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

app = Flask(__name__)
moveit2: MoveIt2 = None

@app.route('/move', methods=['POST'])
def move():
    data = request.get_json()
    pos = [data.get(k) for k in ('posX','posY','posZ')]
    quat = [data.get(k) for k in ('rotX','rotY','rotZ','rotW')]
    if None in pos or None in quat:
        return jsonify(error="Missing fields"), 400

    # Plan to the requested pose
    traj = moveit2.plan(position=pos, quat_xyzw=quat)
    if traj is None:
        return jsonify(status="planning_failed"), 500

    # Execute the planned trajectory
    exec_ok = moveit2.execute(traj)
    if not exec_ok:
        return jsonify(status="execution_failed"), 500

    # Compute FK to get current end-effector pose
    ee_pose_stamped = moveit2.compute_fk()
    if ee_pose_stamped is None:
        return jsonify(status="no_fk_response"), 500

    p = ee_pose_stamped.pose.position
    o = ee_pose_stamped.pose.orientation
    return jsonify({
        "status": "success",
        "ee_position": {"x": p.x, "y": p.y, "z": p.z},
        "ee_orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}
    }), 200

def main():
    global moveit2
    rclpy.init()
    node = rclpy.create_node('rebel_pymoveit_api')
    cbg = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=['joint1','joint2','joint3','joint4','joint5','joint6'],
        base_link_name='base_link',
        end_effector_name='link6',
        group_name='igus_rebel_arm',
        callback_group=cbg
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

    app.run(host='0.0.0.0', port=8080)

    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
