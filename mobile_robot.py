import py_trees
import xml.etree.ElementTree as ET
import time
import math
import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float32

class HumanDetection(py_trees.behaviour.Behaviour):
    # Simulated the human detection behavior, sub to ros topic type Float 32, if it have been published, return SUCCESS if value > 0, 
    #, value < 0 return FAILURE and otherwise return RUNNING
    def __init__(self, name):
        super(HumanDetection, self).__init__(name)
        self.node = Node('human_detecttion')
        self.sub = self.node.create_subscription(Float32, "human_detection", self.callback, 10)
        self.value = 0.0
        self.time1 = 0.0
        self.flag = False
    def callback(self, msg):
        self.value = msg.data
        print("value: ", self.value)
    def update(self):
        rclpy.spin_once(self.node, timeout_sec=1.0)
        if self.value > 0:
            if not self.flag:
                self.time1 = time.time()
                self.flag = True
            if time.time() - self.time1 >5:
                return py_trees.common.Status.SUCCESS
            else:
                print("Confirming.....")
                return py_trees.common.Status.RUNNING
        elif self.value < 0:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

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
    def __init__(self, name, pose, context = None):
        super(MovingToPose, self).__init__(name)
        # if context is None:
        #     rclpy.init()
        self.context = context
        self.node = Node('simple_bt_nav_client')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.node.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.pose = pose
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose = self.pose
        self.node.get_logger().info('Sending goal...')
        self.send_goal_future = self.action_client.send_goal_async(self.goal_msg)
        rclpy.spin_until_future_complete(self.node, self.send_goal_future)
        self.goal_handle = self.send_goal_future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.node.get_logger().info('FAILURE - goal rejected')
            if self.context is None:
                rclpy.shutdown()
            return
        self.get_result_future = self.goal_handle.get_result_async()
        self.done = False
        
        
    def update(self):
        if not self.get_result_future.done():
            self.node.get_logger().info('RUNNING')
            rclpy.spin_once(self.node, timeout_sec=1.0)
            return py_trees.common.Status.RUNNING
        else:
            result = self.get_result_future.result()
            if result.status == 4:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
 

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

def create_node_from_xml(node_element):
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
        return MovingToPose(name=node_name, pose = pose)
    elif node_type == 'HumanDetection':
        return HumanDetection(name=node_name)
    elif node_type == 'Sequence':
        return py_trees.composites.Sequence(name=node_name, memory=True)
    else:
        raise ValueError(f"Invalid node type: {node_type}")

def build_tree_from_xml(xml_path):
    tree = ET.parse(xml_path)
    # print(tree)
    root = tree.getroot()
    print(tree.getroot())
    
    # Find the main behavior tree
    main_tree_id = root.get('main_tree_to_execute')
    behavior_tree = root.find(f".//BehaviorTree[@ID='{main_tree_id}']")
    
    if behavior_tree is None:
        raise ValueError("Main behavior tree not found in XML")
    
    # Get the first child (root node) of the behavior tree
    # print("bt tree",behavior_tree)
    root_element = behavior_tree[0]
    root_node = create_node_from_xml(root_element)
    
    # Build the complete tree
    build_subtree(root_node, root_element)
    
    return py_trees.trees.BehaviourTree(root_node)

def build_subtree(parent_node, xml_node_element):
    """Recursively build the subtree from XML elements."""
    for child in xml_node_element:
        # Create a py_trees node from the XML child element
        py_tree_child = create_node_from_xml(child)
        # Add the created node as a child to the parent node
        parent_node.add_child(py_tree_child)
        # Recursively build the subtree for this node
        build_subtree(py_tree_child, child)

def print_tree_status(root):
    """Print the current status of the behavior tree."""
    print("\nBehavior Tree Status:")
    print("---------------------")
    for node in root.iterate():
        status = str(node.status) if node.status else 'UNKNOWN'
        print(f"{node.name}: {status}")
    print("---------------------")

def main():
    rclpy.init()
    tree = build_tree_from_xml("mobile_robot.xml")
    
    # Tick the tree
    try:
        for i in range(1000): 
            print(f"\nTree tick {i+1}")
            tree.tick()
            print_tree_status(tree.root)
            if tree.root.status == py_trees.common.Status.SUCCESS:
                print("Behavior Tree succeeded.")
                break
            if tree.root.status == py_trees.common.Status.FAILURE:
                print("Behavior Tree failed.")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        print("Behavior Tree stopped.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()

        