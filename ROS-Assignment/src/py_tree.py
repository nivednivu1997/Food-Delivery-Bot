import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String 
import time
import py_trees
import threading
import queue
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

# Simulated Robot State
robot_state = {
    "order_received": False,
    "order_canceled": False,
    "confirmation_kitchen": False,
    "confirmation_table": {},
    "current_task": None,
    "tables": [],
}
msg_queue = queue.Queue()
locations = {
    "home": {"x": 3.9989, "y": -9.2448, "z": 0.4397},
    "table1": {"x": 2.4824, "y": -4.4583, "z": -0.0014},
    "table2": {"x": 5.6076, "y": -4.4618, "z": -0.0014},
    "table3": {"x": 8.6399, "y": -4.4647, "z": -0.0014},
    "kitchen": {"x": 11.0781, "y": -5.8501, "z": -0.0092}
}

class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__("robot_state_subscriber")
        self.subscription = self.create_subscription(
            String,
            "/robot_state",
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Robot state subscriber initialized.")
        self.msg_queue = queue.Queue() 

    def listener_callback(self, msg):
        try:
            new_state = json.loads(msg.data)  # ✅ Parse JSON properly
            print(f"📥 Received raw message: {msg.data}")  # Debug raw data
            print(f"✅ Parsed state: {new_state}")  # Debug parsed state
            self.msg_queue.put(new_state)
        except json.JSONDecodeError as e:
            print(f"❌ JSON decoding error: {e}")

class ReceiveOrder(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckOrderReceived"):
        super().__init__(name)

    def update(self):
        if robot_state.get("order_received", False):
            print("Order received!")
            return py_trees.common.Status.SUCCESS  # Proceed to next step
        
        print("No new order received. Waiting...")
        return py_trees.common.Status.FAILURE  # Prevents looping if no new order

class AllTablesDelivered(py_trees.behaviour.Behaviour):
    def __init__(self, name="AllTablesDelivered", timeout=10):
        super().__init__(name)
        self.timeout = timeout
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        # 🚨 Check if order is canceled -> IMMEDIATELY go home
        if robot_state.get("order_canceled", True): #false
            print("🚨 Order Canceled! Returning to home.")
            robot_state["go_home"] = True  # Make sure robot goes home
            return py_trees.common.Status.SUCCESS  # End tree execution

        pending_tables = [
            table for table in robot_state.get("tables", [])
            if not robot_state.get("confirmation_table", {}).get(table, False)
        ]

        if not pending_tables:
            print("✅ All tables have received their orders! Returning home.")
            robot_state["tables"] = []
            return py_trees.common.Status.SUCCESS

        # Check if we have already retried once
        if robot_state.get("retried_kitchen", False):
            print("⏳ Timeout again! Already retried kitchen. Going home instead.")
            robot_state["go_home"] = True
            return py_trees.common.Status.SUCCESS  # Ends loop

        # If timeout happens for the first time, return to kitchen
        if time.time() - self.start_time > self.timeout:
            print("⏳ Timeout! No confirmation received. Returning to kitchen.")
            robot_state["retried_kitchen"] = True  # Mark that retry has happened
            return py_trees.common.Status.FAILURE  

        print(f"🚧 Waiting for table {pending_tables} to confirm receipt...")
        return py_trees.common.Status.RUNNING



class CheckOrderCanceled(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckOrderCanceled"):
        super().__init__(name)
    
    def update(self):
        print("Checking if order is canceled...")
        if robot_state["order_canceled"]:
            print("Order was canceled!")
            return py_trees.common.Status.FAILURE
        print("Order is valid!")
        return py_trees.common.Status.SUCCESS

class MoveToKitchen(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="MoveToKitchen"):
        super().__init__(name)
        self.node = node
        self.arrived = False
        self.action_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")

    def initialise(self):
        self.arrived = False

    def send_navigation_goal(self, target_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)

        # ✅ Use executor instead of spin_until_future_complete
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        if not result.accepted:
            print("❌ Navigation goal rejected!")
            return False

        # ✅ Wait for the result using a separate future
        result_future = result.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()
        return result_future.result().status == 4  # SUCCESS status

    def update(self):
        if robot_state.get("go_home", False):
            print("🏠 Timeout again! Skipping kitchen confirmation and returning home.")
            return py_trees.common.Status.SUCCESS

        if not self.arrived:
            print("🔄 Going to the kitchen for food Pickup...")
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = locations["kitchen"]["x"]
            target_pose.pose.position.y = locations["kitchen"]["y"]
            target_pose.pose.position.z = locations["kitchen"]["z"]
            target_pose.pose.orientation.w = 1.0  # Default orientation

            if self.send_navigation_goal(target_pose):
                self.arrived = True
            else:
                return py_trees.common.Status.FAILURE

        print("🍽️ Arrived at the kitchen!")

        if robot_state.get("retried_kitchen", False):
            print("⏳ Timeout again! Already retried kitchen. Going home instead.")
            robot_state["go_home"] = True
            return py_trees.common.Status.SUCCESS  

        print("Kitchen confirmed order is ready!")
        return py_trees.common.Status.SUCCESS




class WaitForKitchenConfirmation(py_trees.behaviour.Behaviour):
    def __init__(self, name="WaitForKitchenConfirmation", timeout=15):
        super().__init__(name)
        self.timeout = timeout
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if robot_state["confirmation_kitchen"]:
            print("Kitchen confirmed order is ready!")
            robot_state["order_ready"] = True
            return py_trees.common.Status.SUCCESS

        if time.time() - self.start_time > self.timeout:
            print("Kitchen confirmation timed out! Returning home.")
            robot_state["kitchen_timeout"] = True  # Set timeout flag
            return py_trees.common.Status.SUCCESS  # Proceed to next step

        print("Waiting for kitchen confirmation...")
        return py_trees.common.Status.RUNNING

class MoveToTable(py_trees.behaviour.Behaviour):
    def __init__(self, node, table, name=None):
        super().__init__(name or f"MoveToTable_{table}")
        self.node = node
        self.table = table
        self.arrived = False
        self.action_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")

    def initialise(self):
        self.arrived = False  # Reset arrival status at start

    def send_navigation_goal(self, target_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)

        # ✅ Use executor for handling async operations
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        if not result.accepted:
            print(f"❌ Navigation goal to {self.table} rejected!")
            return False

        # ✅ Wait for navigation completion
        result_future = result.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()
        return result_future.result().status == 4  # SUCCESS status

    def update(self):
        if robot_state.get("go_home", False):
            print("🏠 Order canceled! Returning home immediately.")
            return py_trees.common.Status.SUCCESS

        # ✅ Check if table order was canceled before navigating
        if self.table in robot_state.get("canceled_tables", []):
            print(f"🚧 Table {self.table} order canceled. Returning to kitchen.")
            return py_trees.common.Status.FAILURE  # Signal failure to Selector

        if not self.arrived:
            print(f"🚗 Moving to {self.table}...")
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = locations[self.table]["x"]
            target_pose.pose.position.y = locations[self.table]["y"]
            target_pose.pose.position.z = locations[self.table]["z"]
            target_pose.pose.orientation.w = 1.0  # Default orientation

            if self.send_navigation_goal(target_pose):
                self.arrived = True
            else:
                print(f"❌ Failed to reach {self.table}! Returning to kitchen.")
                return py_trees.common.Status.FAILURE  # Failure, retry or return

        print(f"✅ Arrived at {self.table}!")
        return py_trees.common.Status.SUCCESS




class WaitForTableConfirmation(py_trees.behaviour.Behaviour):
    def __init__(self, table, name=None, timeout=5):
        super().__init__(name or f"WaitForTableConfirmation_{table}")
        self.table = table
        self.timeout = timeout
        self.start_time = None

    def update(self):
        if self.start_time is None:
            self.start_time = time.time()
        print(f"Waiting for table {self.table} confirmation...")
        if robot_state["confirmation_table"].get(self.table, False):
            print(f"Table {self.table} confirmed delivery!")
            return py_trees.common.Status.SUCCESS
        if time.time() - self.start_time > self.timeout:
            print(f"Table {self.table} confirmation timed out!")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

class MoveToHome(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="MoveToHome"):
        super().__init__(name)
        self.node = node
        self.arrived = False
        self.action_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")

    def initialise(self):
        self.arrived = False  # Reset state at start

    def send_navigation_goal(self, target_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)

        # ✅ Use executor to process async results
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)

        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        if not result.accepted:
            print("❌ Navigation goal to home rejected!")
            return False

        # ✅ Wait for navigation completion
        result_future = result.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()
        return result_future.result().status == 4  # SUCCESS status

    def update(self):
        if not self.arrived:
            print("🏠 Returning to home base...")
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = locations["home"]["x"]
            target_pose.pose.position.y = locations["home"]["y"]
            target_pose.pose.position.z = locations["home"]["z"]
            target_pose.pose.orientation.w = 1.0  # Default orientation

            if self.send_navigation_goal(target_pose):
                self.arrived = True
            else:
                print("❌ Failed to reach home! Retrying...")
                return py_trees.common.Status.FAILURE  # Retry until success

        print("✅ Arrived at home base!")
        return py_trees.common.Status.SUCCESS
class FinalState(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("FinalState")

    def update(self):
        print("🏁 All deliveries completed! Stopping behavior tree.")
        return py_trees.common.Status.SUCCESS



def create_behavior_tree():
    order_processing = py_trees.composites.Sequence("Order Processing", memory=True)
    order_processing.add_children([
        ReceiveOrder(),
        CheckOrderCanceled(),
    ])

    kitchen_sequence = py_trees.composites.Sequence("Kitchen Process", memory=True)
    kitchen_sequence.add_children([
        MoveToKitchen(node), #add node as param
        WaitForKitchenConfirmation()
    ])

    table_sequence = py_trees.composites.Selector("Table Deliveries", memory=True)
    for table in robot_state.get("tables", []):  # Use .get() to avoid errors
        table_task = py_trees.composites.Sequence(f"Deliver to {table}", memory=True)
        table_task.add_children([
            MoveToTable(node,table),
            WaitForTableConfirmation(table)
            
            
        ])
        table_sequence.add_child(table_task)
    table_sequence.add_child(AllTablesDelivered())
    
    root = py_trees.composites.Sequence("Restaurant Robot BT", memory=False)
    root.add_children([
        order_processing,
        kitchen_sequence,
        table_sequence,
        MoveToHome(node),
        FinalState()
    ])
    return root

def ros_spin_thread():
    rclpy.spin(node)

def main():
    global robot_state
    rclpy.init()
    global node
    node = RobotStateSubscriber()

    # Start ROS spin in a separate thread
    spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    spin_thread.start()


    print("Starting behavior tree...")
    behavior_tree = create_behavior_tree()
    behaviour_tree_root = py_trees.trees.BehaviourTree(behavior_tree)
    previous_tables = []

    while rclpy.ok():
        try:
            new_state = node.msg_queue.get(timeout=1)  # ✅ Wait for new message
            print(f"✅ Received new state: {new_state}")
            
            robot_state.update(new_state)
            print(f"✅ Updated robot state: {robot_state}")

            # ✅ Check if tables list has changed
            if robot_state["tables"] != previous_tables:
                print(f"🔄 Tables changed! Rebuilding behavior tree...")
                behavior_tree = create_behavior_tree()
                behaviour_tree_root = py_trees.trees.BehaviourTree(behavior_tree)
                previous_tables = robot_state["tables"].copy()  # ✅ Save state

        except queue.Empty:
            print("⏳ No new order received. Waiting...")

        behaviour_tree_root.tick()  # ✅ Process behavior tree



if __name__ == "__main__":
    main()
