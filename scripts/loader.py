#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import threading

class LoaderNode(Node):
    """
    Loader agent.

    ### Percepts:
        * Pool from loader manager
        * Task from loader manager
        * Courier arrival confirmation
    ### Actions:
        * Package loading on robot
    ### Sent messages:
        * Status update to loader manager
        * Response to pool from loader manager
        * Loaded package confirmation to robot
    ### Received messages:
        * Pool from loader manager
        * Courier arrival confirmation
    """
    def __init__(self):
        super().__init__('LoaderNode')

        self.declare_parameter(name='loader_id', value=0,
                        descriptor=ParameterDescriptor(
                            description="Loader ID",
                            type=Parameter.Type.INTEGER.value
                        ))
        
        self.id = self.get_parameter('loader_id').value
        self.status = 'idle'
        self.task = None
        self.courier = None
        
        
        # Subscribers for receiving request from loader manager
        self.sub_request = self.create_subscription(String, 'loader_pool', self.request_callback, 10)
    
        # Create publisher to send confirmation to loader manager
        self.pub_response = self.create_publisher(String, 'loader_pool_response', 10)

        # subscriber for receiving assigned courier
        self.sub_courier = self.create_subscription(String, 'loader_courier_assignment', self.courier_assignment, 10)

        # Create publisher to send status update to loader manager
        self.pub_status_to_loader_manager = self.create_publisher(String, 'loader_status', 10)
        self.create_timer(1.0, self.return_loader_status) # publish loader status

        # Create subscriber to receive courier arrival confirmation
        self.sub_courier = self.create_subscription(String, 'courier_arrival', self.courier_arrival_callback, 10)
        # Create publisher to send loaded package confirmation to robot
        self.pub_loaded_package = self.create_publisher(String, 'loaded_package', 10)

        self.get_logger().info(f"Loader with id {self.id} initialized successfully!")

    def request_callback(self, msg: String):
        """
        Handle request from loader manager.

        task structure:
        request = {
            id: requestId,
            name: packageName,
            type: packageType,
            weight: packageWeight,
            priority: packagePriority,
            loader: None,
            courier: None
        }
        """
        response = json.loads(msg.data)

        response_type = response['type'] # pool or task confirmation

        if response_type == 'pool':
            self.get_logger().info(f"Received pool from loader manager.")
            can_accept = self.status == 'idle'
            self.pub_response.publish(String(data=json.dumps({'id': self.id, 'is_willing': can_accept})))

        elif response_type == 'task':
            task = json.loads(response['task'])
            if self.id != task['loader']:
                return
            self.assign_task(task)
        else:
            self.get_logger().info(f"Invalid response type: {response_type}")

    def assign_task(self, task: dict):
        """
        Assign task to loader.
        """
        #request = json.loads(msg.data)
        self.task = task
        self.status = 'waiting'
        self.get_logger().info(f"Task assigned to loader {self.id}.")

    
    def return_loader_status(self):
        """
        Publish loader status.
        """
        self.pub_status_to_loader_manager.publish(String(data=json.dumps({
            'id': self.id,
            'task_id': self.task['id'] if self.task else None,
            'status': self.status})))

    def courier_assignment(self, msg: String):
        """
        Handle courier assignment.
        """
        response = json.loads(msg.data)
        
        if response['loader'] != self.id:
            return
        self.courier = response['courier']

    def courier_arrival_callback(self, msg: String):
        """
        Handle courier arrival confirmation.
        """
        response = json.loads(msg.data)
        
        if response['courier_id'] != self.courier:
            return
        
        self.get_logger().info(f"Courier {self.courier} arrived at loader {self.id}.")
        self.status = 'loading'

        # simulate loading time       
        def action():
            time.sleep(5)
            # send loaded package confirmation to robot
            self.pub_loaded_package.publish(String(data=json.dumps({
                'courier_id': self.courier,
                'task_id': self.task['id']
                })))
            self.status = 'idle'
            self.task = None
            self.courier = None
            self.get_logger().info(f"Loader {self.id} is idle.")
        threading.Thread(target=action).start()
        

def main(args=None):
    rclpy.init(args=args)

    loader_node = LoaderNode()

    try:
        rclpy.spin(loader_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print(f"Clean shutdown: Loader{loader_node.id} agent")
    else:
        rclpy.shutdown()

    loader_node.destroy_node()



if __name__ == "__main__":
    main()