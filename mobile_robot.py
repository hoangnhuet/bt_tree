import py_trees
import xml.etree.ElementTree as ET
import time
import random

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
    def __init__(self, name):
        super(MovingToPose, self).__init__(name)  # Fixed the super() call
        self.probs = random.randint(0, 100)
        print(self.probs)
        
    def update(self):
        if self.probs < 30:
            return py_trees.common.Status.FAILURE
        elif self.probs < 80:  # Simplified condition
            self.probs = self.probs + 1
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

def create_gotopose_node(name):
    return MovingToPose(name=name)

def create_node_from_xml(node_element):
    node_type = node_element.tag
    node_name = node_element.attrib['name']
    
    if node_type == 'CheckCondition':
        return CheckCondition(name=node_name)
    elif node_type == 'MovingToPose':
        return MovingToPose(name=node_name)
    elif node_type == 'Sequence':
        return py_trees.composites.Sequence(name=node_name, memory=True)
    else:
        raise ValueError(f"Invalid node type: {node_type}")

def build_tree_from_xml(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # Find the main behavior tree
    main_tree_id = root.get('main_tree_to_execute')
    behavior_tree = root.find(f".//BehaviorTree[@ID='{main_tree_id}']")
    
    if behavior_tree is None:
        raise ValueError("Main behavior tree not found in XML")
    
    # Get the first child (root node) of the behavior tree
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
    
    # Parse and build the tree from the XML file
    tree = build_tree_from_xml("mobile_robot.xml")
    
    # Tick the tree
    try:
        for i in range(10):  # Run for 10 ticks
            print(f"\nTree tick {i+1}")
            tree.tick()
            print_tree_status(tree.root)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Behavior Tree stopped.")

if __name__ == "__main__":
    main()

        
        