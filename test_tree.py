import py_trees
import xml.etree.ElementTree as ET
import time

# Define custom behavior nodes
class ApproachObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachObject, self).__init__(name)
        self.t1 = time.time()
        self.t2 = 0.0
        self.done = False

    def update(self):
        self.t2 = time.time()
        if self.t2 - self.t1 > 10:
            self.done = True
        if self.done:
            print("Reached")
            return py_trees.common.Status.SUCCESS
        else:
            print(f"Approaching: {self.name}")
            return py_trees.common.Status.RUNNING

class BatteryCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(BatteryCheck, self).__init__(name)
    
    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        battery_ok = blackboard.get('battery_ok')
        if battery_ok:
            print(f"{self.name}: Battery OK")
            return py_trees.common.Status.SUCCESS
        print(f"{self.name}: Battery Low")
        return py_trees.common.Status.FAILURE

class GripperInterface(py_trees.behaviour.Behaviour):
    def __init__(self, name, action="open"):
        super(GripperInterface, self).__init__(name)
        self.action = action
    
    def update(self):
        if self.action == "open":
            print(f"{self.name}: Gripper opening")
        elif self.action == "close":
            print(f"{self.name}: Gripper closing")
        return py_trees.common.Status.SUCCESS

def create_battery_check_node(name):
    """
    Create a battery check node
    """
    return BatteryCheck(name=name)

def create_node_from_xml(node_element):
    """
    Create a py_trees node from an XML node element.
    """
    node_type = node_element.tag
    node_name = node_element.attrib['name']
    
    if node_type == "Sequence":
        return py_trees.composites.Sequence(name=node_name, memory=True)
    elif node_type == "Selector":
        return py_trees.composites.Selector(name=node_name, memory=True)
    elif node_type == "CheckBattery":
        return create_battery_check_node(name=node_name)
    elif node_type == "ApproachObject":
        return ApproachObject(name=node_name)
    elif node_type == "OpenGripper":
        return GripperInterface(name=node_name, action="open")
    elif node_type == "CloseGripper":
        return GripperInterface(name=node_name, action="close")
    else:
        raise ValueError(f"Unknown node type: {node_type}")

def build_tree_from_xml(xml_path):
    """
    Build a behavior tree from an XML file.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # Locate the main behavior tree node
    behavior_tree_element = root.find("BehaviorTree")
    if behavior_tree_element is None:
        raise ValueError("No BehaviorTree node found in the XML file.")
    
    # Create the root of the behavior tree
    main_tree = None
    for child in behavior_tree_element:
        main_tree = create_node_from_xml(child)
        build_subtree(main_tree, child)
    
    return main_tree

def build_subtree(parent_node, xml_node_element):
    """
    Recursively build the subtree from XML elements.
    """
    for child in xml_node_element:
        # Create a py_trees node from the XML child element
        py_tree_child = create_node_from_xml(child)
        
        # Add the created node as a child to the parent node
        parent_node.add_child(py_tree_child)
        
        # Recursively build the subtree for this node
        build_subtree(py_tree_child, child)

def print_tree_status(root):
    """
    Print the current status of the behavior tree
    """
    print("\nBehavior Tree Status:")
    print("---------------------")
    for node in root.iterate():
        status = str(node.status) if node.status else 'UNKNOWN'
        print(f"{node.name}: {status}")
    print("---------------------")

def main():
    # Initialize the blackboard first
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("battery_ok", True)  # Set the battery status to OK (True)
    
    # Parse and build the tree from the XML file
    root = build_tree_from_xml("test_tree.xml")
    tree = py_trees.trees.BehaviourTree(root)
    
    # Tick the tree
    try:
        for i in range(10):  # Run for 5 ticks
            print(f"\nTree tick {i+1}")
            tree.tick()
            print_tree_status(root)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Behavior Tree stopped.")

if __name__ == "__main__":
    main()
