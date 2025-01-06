#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class LoaderManagerNode(Node):
    """
    Loader manager agent.

    Percepts:
        * Request from system manager
    Actions:
        * Loader selection
        * Task assignment to loader
        * Courier ordering
    Sent messages:
        * Poll to loaders
        * Task confirmation to loader
        * Courier request to robot manager
        * Status update to system manager
        * Confirmation to system manager
    Received messages:
        * Status updates from loaders
        * Request from system manager
        * Response to poll from loaders
    """
    def __init__(self):
        super().__init__('LoaderManagerNode')

        # ==========WITH SYSTEM MANAGER==========
        self.create_timer(1.0, self.return_loader_status) # publish loader status

        # Subscribers for receiving request from system manager
        self.sub_request = self.create_subscription(String, 'request_to_loader_manager', self.request_callback, 10)

        # Publisher for status updates
        self.pub_status = self.create_publisher(String, 'status_update', 10)

        # ==========WITH ROBOT MANAGER==========

        # Create publisher to pass on request to robot manager
        self.pub_req_to_robot_manager = self.create_publisher(String, 'request_to_robot_manager', 10)
        
        # Subscribe to receive confirmation from robot manager
        self.sub_req_conf_from_robot_manager = self.create_subscription(String, 'confirmation_to_loader_manager', self.handle_request_confirmation, 10)

    
        # ==========WITH LOADERS==========

        # Publisher for loader pool
        self.pub_loader_pool = self.create_publisher(String, 'loader_pool', 10)
        # Subscriber for recieving loader pool results
        self.sub_loader_pool = self.create_subscription(String, 'loader_pool_response', self.handle_loader_pool, 10)

        # Listen for loader status updates
        self.sub_loader_status = self.create_subscription(String, 'loader_status', self.handle_loader_status, 10)

        # publish to loaders information about assigned courier
        self.pub_courier_assignment = self.create_publisher(String, 'loader_courier_assignment', 10)

        #==============================        


        self.queued_requests = []
        # a timer to assign requeststs when a loader frees up
        self.assign_requests_timer = self.create_timer(10.0, self.assign_request_queue)

        self.loaders = []        

        self.current_task_loader_assigned = {}

        self.get_logger().info("Loader manager initialized successfully!")
    
   
    def request_callback(self, request_msg: String):
        """
        Handle request from system manager. Assign task to loader.
        Request structure:
        request = {
            id: requestId,
            name: packageName,
            type: packageType,
            weight: packageWeight,
            priority: priority,
            status: 'Pending',
            loader: None,
            courier: None
        }
        """
        
        
        # convert string to json
        request = json.loads(request_msg.data)
        self.assign_request_queue(request)
        self.current_task_loader_assigned[request['id']] = False
        
    def handle_loader_pool(self, msg: String):
        """
        Handle response from loaders.
        """
        if len(self.queued_requests) == 0:
            return
        
        request = self.queued_requests[0]

        if request['loader'] or self.current_task_loader_assigned[request['id']]:
            return
        
        response = json.loads(msg.data)
        self.get_logger().info(f"Received response from loader {response['id']}: {response['is_willing']}")
        if response["is_willing"]:
            self.current_task_loader_assigned[request['id']] = True
            self.get_logger().info(f"Loader {response['id']} is willing to accept task.")
            
            
            # Assign loader to task
            request["loader"] = response["id"]
            request["status"] = "Assigned loader"
    
            # send confirmation to loader
            self.pub_loader_pool.publish(String(data=json.dumps({
                "type": "task",
                "task": json.dumps(request)})))
            
            # send status to system manager
            self.return_request_status(request)

            # Send task data to robot manager
            request_msg = String(data=json.dumps(request))
            self.pub_req_to_robot_manager.publish(request_msg) # TODO

            # publish loader status
            self.return_loader_status()

            self.queued_requests.pop(0)

        else:
            self.get_logger().info(f"Loader {response['id']} is not willing to accept task... retrying")
            #self.queued_requests.append(request)

    def assign_request_queue(self, request:dict=None):
        """
        Assign requests to loaders from request queue.
        """

        # can be called by timer or from request callback
        if request:
            self.get_logger().info(f"Received request with ID: {request['id']}")
            self.queued_requests.append(request)
        
        if len(self.queued_requests) == 0:
            return
        
        # pool loaders
        self.get_logger().info(f"Pooling loaders for task...")
        pool_msg = String(data=json.dumps({
            "type": "pool"}))
        self.pub_loader_pool.publish(pool_msg)
        
    def handle_loader_status(self, msg: String):
        """
        Handle status updates from loaders.
        """
        new_loader = json.loads(msg.data)
        loader = next((l for l in self.loaders if l['id'] == new_loader['id']), None)
        if not loader:
            self.loaders.append(new_loader)
        else:
            loader['status'] = new_loader['status']

        # update system manager with task status
        if new_loader['task_id']:
            self.return_request_status({
                "id": new_loader['task_id'],
                "status": new_loader['status']
            })

           
    def handle_request_confirmation(self, msg: String):
        """
        Handle confirmation from robot manager.
        """
        request = json.loads(msg.data)
        
        if request["loader"] and request["courier"]:
        #    self.get_logger().info("Task confirmed by robot manager")
            self.get_logger().info(f"Received confirmation from robot manager: ID: {request['id']} Loader:{request['loader']} Courier:{request['courier']}")
            
            # let loader know that courier is on the way
            self.pub_courier_assignment.publish(String(data=json.dumps({
                "loader": request['loader'],
                "courier": request['courier']
            })))

    def return_loader_status(self):
        """
        Publish loader status to system manager.
        """

        for loader in self.loaders:
            # publish loader status
            msg = String()
            msg.data = json.dumps({
                "type": "loader",
                "id": loader['id'],
                "status": loader['status']
            })
            self.pub_status.publish(msg)

    def return_request_status(self, request):
        """
        Publish request status to system manager.
        """
        msg = String()
        msg.data = json.dumps({
            "type": "request",
            "request": json.dumps(request)
        })
        #self.get_logger().info(f"request status from loader manager: {msg.data}")
        self.pub_status.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    loader_mng_node = LoaderManagerNode()


    try:
        rclpy.spin(loader_mng_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: Loader manager agent")
    else:
        rclpy.shutdown()

    loader_mng_node.destroy_node()



if __name__ == "__main__":
    main()