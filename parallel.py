import py_trees
import xml.etree.ElementTree as ET
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import threading

class CheckCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckCondition, self).__init__(name)
        self.t1 = time.time()
        self.t2 = 0.0
        self.done = False

    def update(self):
        self.t2 = time.time()
        if self.t2 - self.t1 > 5:
            self.done = True

        if self.done:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

class MovingToPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, pose, context=None):
        super(MovingToPose, self).__init__(name)
        # if context is None:
        #     rclpy.init()
        self.context = context
        self.node = Node('simple_bt_nav_client_' + name)
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.node.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.pose = pose
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose = self.pose
        self.node.get_logger().info('Sending goal...')
        self.send_goal_future = self.action_client.send_goal_async(self.goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_handle = None
        self.get_result_future = None
        self.done = False
        self.result = None

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.node.get_logger().info('FAILURE - goal rejected')
            self.done = True
            self.result = None
            return
        self.node.get_logger().info('Goal accepted')
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.done = True
        self.result = future.result()

    def update(self):
        if self.get_result_future is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future.done():
            if self.result is None:
                return py_trees.common.Status.FAILURE
            if self.result.status == 4:  # SUCCESS
                self.node.get_logger().info('Action Succeeded')
                return py_trees.common.Status.SUCCESS
            elif self.result.status == 6:  # ABORTED
                self.node.get_logger().info('Action Aborted')
                return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().info(f'Action status: {self.result.status}')
                return py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().info('Action Running')
            return py_trees.common.Status.RUNNING

def EulerToQuat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    q.w = cy * cp * cr + sy * sp * sr
    return q

def create_node_from_xml(node_element, executor=None):
    node_type = node_element.tag
    node_name = node_element.attrib['name']
    
    if node_type == 'CheckCondition':
        return CheckCondition(name=node_name)
    elif node_type == 'MovingToPose':
        node_x = node_element.attrib['x']
        node_y = node_element.attrib['y']
        node_yaw = node_element.attrib['yaw']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rclpy.clock.Clock().now().to_msg()
        pose.pose.position = Point(x=float(node_x), y=float(node_y), z=0.0)
        pose.pose.orientation = EulerToQuat(0, 0, float(node_yaw))
        moving_to_pose = MovingToPose(name=node_name, pose=pose)
        if executor is not None:
            executor.add_node(moving_to_pose.node)
        return moving_to_pose
    elif node_type == 'Sequence':
        return py_trees.composites.Sequence(name=node_name, memory=True)
    else:
        raise ValueError(f"Invalid node type: {node_type}")

def build_subtree(parent_node, xml_node_element, executor=None):
    """Recursively build the subtree from XML elements."""
    for child in xml_node_element:
        # Create a py_trees node from the XML child element
        py_tree_child = create_node_from_xml(child, executor)
        # Add the created node as a child to the parent node
        parent_node.add_child(py_tree_child)
        # Recursively build the subtree for this node
        build_subtree(py_tree_child, child, executor)

def build_tree_from_xml(xml_path, executor=None):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # Find the main behavior tree
    main_tree_id = root.get('main_tree_to_execute')
    behavior_tree = root.find(f".//BehaviorTree[@ID='{main_tree_id}']")
    
    if behavior_tree is None:
        raise ValueError("Main behavior tree not found in XML")
    
    # Get the first child (root node) of the behavior tree
    root_element = behavior_tree[0]
    root_node = create_node_from_xml(root_element, executor)
    
    # Build the complete tree
    build_subtree(root_node, root_element, executor)
    
    return py_trees.trees.BehaviourTree(root_node)

def print_tree_status(root):
    """Print the current status of the behavior tree."""
    print("\nBehavior Tree Status:")
    print("---------------------")
    for node in root.iterate():
        status = str(node.status) if node.status else 'UNKNOWN'
        print(f"{node.name}: {status}")
    print("---------------------")

def ros_spin(executor):
    """Function to spin the ROS executor."""
    executor.spin()

def main():
    rclpy.init()

    # Create a MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # Create main node if needed
    main_node = Node('main_node')
    executor.add_node(main_node)
    
    # Parse and build the tree from the XML file, passing the executor
    tree = build_tree_from_xml("mobile_robot.xml", executor)
    
    # Create and start a separate thread to spin ROS
    ros_thread = threading.Thread(target=ros_spin, args=(executor,), daemon=True)
    ros_thread.start()
    
    # Tick the Behavior Tree in the main thread
    try:
        for i in range(10):  # Run for 10 ticks
            print(f"\nTree tick {i+1}")
            tree.tick()
            print_tree_status(tree.root)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Behavior Tree stopped.")
    
    # Shutdown executor and ROS
    executor.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
