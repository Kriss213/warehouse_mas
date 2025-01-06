#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class RobotManagerNode(Node): #TODO
    """
    Robot manager agent.
        
    ### Percepts:
        * request from loader manager
    ### Actions:
        * robot selection (poll)
        * task assignment to robot
    ### Sent messages:
        * poll to robots
        * task confirmation to robot
        * status update to system manager
        * confirmation to loader manager        
    ### Received messages:
        * status updates from robots
        * request from loader manager
        * response to poll from robots
    """

    def __init__(self):
        super().__init__('RobotManagerNode')


        self.create_timer(1.0, self.return_courier_status) # publish courier status

        # Subscribers for receiving request from loader manager
        self.sub_request = self.create_subscription(String, 'request_to_robot_manager', self.request_callback, 10)
        
        # Create publisher to send confirmation to loader manager
        self.pub_conf_to_loader_manager = self.create_publisher(String, 'confirmation_to_loader_manager', 10)

        # Publisher for status updates
        self.pub_status = self.create_publisher(String, 'status_update', 10)

        # ==========WITH COURIERS==========

        # Publisher for courier pool
        self.pub_courier_pool = self.create_publisher(String, 'courier_pool', 10)
        # Subscriber for recieving courier pool results
        self.sub_courier_pool = self.create_subscription(String, 'courier_pool_response', self.handle_courier_pool, 10)

        # Listen for courier status updates
        self.sub_courier_status = self.create_subscription(String, 'courier_status', self.handle_courier_status, 10)

        #==============================        


        self.auctioned_request = None
        self.queued_requests = []
        # a timer to assign requeststs when a courier frees up
        self.assign_requests_timer = self.create_timer(10.0, self.assign_request_queue)

        self.couriers = [] # keep track of couriers (id, status and load (weight))

        self.current_task_courier_assigned = {}

        self.get_logger().info("Robot manager initialized successfully!")
    
   
    def request_callback(self, request_msg: String):
        """
        Handle request from system manager. Assign task to courier.
        Request structure:
        request = {
            id: requestId,
            name: packageName,
            type: packageType,
            weight: packageWeight,
            priority: priority,
            status: 'Pending',
            loader: some_loader_id,
            courier: None
        }
        """
        
        
        # convert string to json
        request = json.loads(request_msg.data)
        self.assign_request_queue(request)
        self.current_task_courier_assigned[request['id']] = False
        
    def handle_courier_pool(self, msg: String):
        """
        Handle response from couriers.
        """
        if len(self.queued_requests) == 0:
            return
        
        request = self.auctioned_request#self.queued_requests[0]

        if request == None or request['courier'] or self.current_task_courier_assigned[request['id']]:
            # courier already assigned to task
            return
        
        response = json.loads(msg.data)
        self.get_logger().info(f"Received response from courier {response['id']}: {response['is_willing']}")
        if response["is_willing"]:
            self.current_task_courier_assigned[request['id']] = True
            self.get_logger().info(f"Courier {response['id']} is willing to accept task.")
            
            
            # Assign courier to task
            request["courier"] = response["id"]
            request["status"] = "Assigned courier"
            # send confirmation to courier
            self.pub_courier_pool.publish(String(data=json.dumps({
                "type": "task",
                "task": json.dumps(request)})))
            
            # remove task from robot manager tasks as courier is taking care of it
            self.queued_requests = [qreq for qreq in self.queued_requests if qreq['id'] != request['id']]


            # confirm to loader manager
            self.pub_conf_to_loader_manager.publish(String(data=json.dumps(request)))

            # send status to system manager
            self.return_request_status(request)

            # publish courier status
            self.return_courier_status()           

        else:
            self.get_logger().info(f"Courier {response['id']} is not willing to accept task... retrying")
            #self.queued_requests.append(request)

    def assign_request_queue(self, request:dict=None):
        """
        Assign requests to couriers from request queue.
        """

        # can be called by timer or by request callback
        if request:
            self.get_logger().info(f"Received request with ID: {request['id']}")
            self.queued_requests.append(request)
        
        if len(self.queued_requests) == 0:
            return
        
        # pool couriers
        self.get_logger().info(f"Pooling couriers for task...")
        self.auctioned_request = self.queued_requests[0]
        pool_msg = String(data=json.dumps({"type": "pool", "package_weight": self.auctioned_request['weight']}))
        self.pub_courier_pool.publish(pool_msg)
        
    def handle_courier_status(self, msg: String):
        """
        Handle status updates from couriers.
        """
        new_courier = json.loads(msg.data)
        courier = next((c for c in self.couriers if c['id'] == new_courier['id']), None)
        
        if not courier:
            self.couriers.append(new_courier)
        else:            
            courier['status'] = new_courier['status']
            courier['load'] = new_courier['load']

        
        courier_task_statuses:dict = new_courier['task_statuses']
        # update system manager with task status
        for task_id, task_status in courier_task_statuses.items():
            self.return_request_status({
                "id": task_id,
                "status": task_status
            })
        

    def return_courier_status(self):
        """
        Publish courier status to system manager.
        """
        for courier in self.couriers:
            # publish courier status
            msg = String()
            msg.data = json.dumps({
                "type": "courier",
                "id": courier['id'],
                "status": courier['status'],
                "load": courier['load']
            })

            self.pub_status.publish(msg)

    def return_request_status(self, request):
        """
        Publish request status to system manager.
        """
        #self.get_logger().info(f"Returning request status: {request}")
        msg = String()
        msg.data = json.dumps({
            "type": "request",
            "request": json.dumps(request)
        })

        self.pub_status.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    courier_mng_node = RobotManagerNode()


    try:
        rclpy.spin(courier_mng_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: Robot manager agent")
    else:
        rclpy.shutdown()

    courier_mng_node.destroy_node()



if __name__ == "__main__":
    main()