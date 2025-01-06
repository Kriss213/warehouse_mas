#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import threading

class RobotNode(Node):
    """
    Courier agent.

    ### Percepts:
        * pool from robot manager
        * task from robot manager
        * invitation to unload from unloading point
    ### Actions:
        * package loading on robot
    ### Sent messages:
        * arrival confirmation to loader
        * response to pool from robot manager
        * line number request at unloading point
        * arrival confirmation at unloading point
    ### Received messages:
        * invitation to unload
        * poll and task from robot manager
        * loaded package confirmation from loader
        * unloaded package confirmation from unloading point
    """
    MAX_LOAD = 10.0
    COURIER_WAIT_TIME = 15.0
    def __init__(self):
        super().__init__('LoaderNode')

        self.declare_parameter(name='courier_id', value=0,
                        descriptor=ParameterDescriptor(
                            description="Courier ID",
                            type=Parameter.Type.INTEGER.value
                        ))
        
        self.id = self.get_parameter('courier_id').value
        self.status = 'idle'
        self.tasks = []
        self.loader = None

        self.recent_task_recieved = False

        self.current_load = 0.0 # current actual load on courier
           
        # Subscribers for receiving request from robot manager
        self.sub_request = self.create_subscription(String, 'courier_pool', self.request_callback, 10)

        # Create publisher to send confirmation to robot manager
        self.pub_response = self.create_publisher(String, 'courier_pool_response', 10)

        # Create publisher to send status update to robot manager
        self.pub_status_to_courier_manager = self.create_publisher(String, 'courier_status', 10)
        self.create_timer(1.0, self.return_courier_status) # publish courier status

        # Create publisher to send courier arrival confirmation
        self.pub_courier_arrival = self.create_publisher(String, 'courier_arrival', 10)
        # Create subscriber to receive loaded package confirmation
        self.sub_loaded_package = self.create_subscription(String, 'loaded_package', self.loaded_package_confirmation, 10)

        # Publisher to request line number at unloading point / robot has arrived at unloader
        self.pub_unloading_line = self.create_publisher(String, 'courier_unloading_line', 10)
        # Subscriber to recieve invitation to unload and confirmation that package is unloaded
        self.sub_unloading_confirmation = self.create_subscription(String, 'courier_unload', self.handle_unload, 10)

        self.get_logger().info(f"Courier with id {self.id} initialized successfully!")

    def request_callback(self, msg: String):
        """
        Handle request from courier manager.

        Request structure:
        request = {
            id: requestId,
            name: packageName,
            type: packageType,
            weight: packageWeight,
            priority: packagePriority,
            loader: some_loader_id,
            courier: None
        }
        """
        response = json.loads(msg.data)

        response_type = response['type'] # pool or task confirmation

        if response_type == 'pool':
            self.get_logger().info(f"Received pool from courier manager.")

            package_weight = float(response['package_weight'])

            cond1 = package_weight + self.current_load <= self.MAX_LOAD
            cond2 = self.status in ('idle', 'waiting')
            can_accept = cond1 and cond2
            self.pub_response.publish(String(data=json.dumps({'id': self.id, 'is_willing': can_accept})))

            
        elif response_type == 'task':
            task = json.loads(response['task'])
            if self.id != task['courier']:
                return
            self.assign_task(task)
        else:
            self.get_logger().info(f"Invalid response type: {response_type}")

    def assign_task(self, task: dict):
        """
        Assign task to courier.
        """
        # this shouldn't happen...
        if self.current_load > self.MAX_LOAD:
            raise Exception(f"Courier {self.id} exceeded max load ({self.current_load}/{self.MAX_LOAD} kg)")

        task['loaded_on_courier'] = False
        self.tasks.append(task)

        self.loader = task['loader']

        # restart waiting thread
        self.recent_task_recieved = True

        self.get_logger().info(f"Task assigned to courier {self.id}.")

        self.go_to_loader(task['id'])

    def go_to_loader(self, task_id):
        """
        Go to assigned loader.
        """
              
        # send arrival confirmation to loader 
        self.status = 'en_route'
        self.get_logger().info(f"Courier {self.id} is en route to loader {self.loader}.")
        def action():
            time.sleep(5)
            self.pub_courier_arrival.publish(String(data=json.dumps({
                'courier_id': self.id, 'request_id': task_id})))
            self.status = 'loading'
            self.update_task_status(task_id=task_id, new_status='loading')
            #self.get_logger().info(f"Courier {self.id} arrived at loader {self.loader}.")
        threading.Thread(target=action).start()
    
    def loaded_package_confirmation(self, msg: String):
        """
        Handle loaded package confirmation from loader.
        """
        response = json.loads(msg.data)
        
        if response['courier_id'] != self.id:
            return
        
        task_id = response['task_id']

        # make sure that task is in self.tasks if it isnt for whatever bullshit reason
        task_in_tasks_list = False
        for t in self.tasks:
            if t['id'] == task_id:
                task_in_tasks_list = True
                break
        if not task_in_tasks_list:
            # how tf
            self.get_logger().info(f"This shouldn't have happened, but task with {task_id} has been added to tasks list")
            self.tasks.append(response)
        
        self.update_task_status(task_id=task_id, new_status='loaded')
        self.get_logger().info(f"Courier {self.id} loaded package with id: {task_id}.")

        self.status = 'waiting'
        self.loader = None

        # find package weight with id
        pkg_weight = 0
        for t in self.tasks:
            if t['id'] == task_id:
                pkg_weight = float(t['weight'])
                break
        if pkg_weight == 0:
            raise ValueError(f'Package with id {task_id} not found in courier {self.id} tasks: {self.tasks}')
        self.current_load += pkg_weight
        
        self.get_logger().info(f"Courier {self.id} is waiting.")


        # wait for seconds for new task
        def action():
            start_time = time.time()
            while time.time() < start_time + self.COURIER_WAIT_TIME:
                if self.recent_task_recieved:
                    start_time = time.time()
                    self.recent_task_recieved = False
                    #break
            self.go_to_unloader()
        threading.Thread(target=action).start()

    def go_to_unloader(self):
        """
        Go to unloading point if no new request is received in 10 seconds.
        """
        
        def action():
            self.get_logger().info(f"Courier {self.id} is en route to unloading point.")
            self.status = 'en_route'
            self.update_task_status(new_status='en_route', all=True)
            time.sleep(5) # drive to unloading point
            self.status = 'unloading_line'

            self.update_task_status(new_status='unloading_line', all=True)

            self.get_logger().info(f"Courier {self.id} arrived at unloading area. Requesting line number...")
            
            # calculate package priorities by cumulative score
            priority_sum = 0
            priorities = {'high':3, 'normal':2, 'low':1}
            for t in self.tasks:
                priority_sum += priorities[t['priority']]

            # Request line number at unloading point
            self.pub_unloading_line.publish(String(data=json.dumps({
                'courier_id': self.id,
                'type': 'line_number_request',
                'priority': priority_sum
            })))
            

        threading.Thread(target=action).start()

    def handle_unload(self, msg:String):
        """
        Handle invitation to unload and unloading confirmation.
        """
        response = json.loads(msg.data)

        response_type = response['type']
        courier_id = response['courier_id']
        if self.id != courier_id:
            return
        
        if response_type == 'unload_invitation':
            def action():
                self.get_logger().info(f"Courier {self.id} arriving at unloading point")
                time.sleep(5)
                self.status = 'unloading'
                self.update_task_status(new_status='unloading', all=True)
                # send arrival confirmation
                self.pub_unloading_line.publish(String(data=json.dumps({
                    'courier_id': self.id,
                    'type': 'arrival_confirmation',
                    'package_ids': [task['id'] for task in self.tasks]
                })))
            threading.Thread(target=action).start()

        elif response_type == 'unload_confirmation':
            # prepare robot for next assignment
            unloaded_pkgs = response['unloaded_packages']
            self.status = 'idle'
            # remove completed tasks
            self.tasks = [t for t in self.tasks if t['id'] not in unloaded_pkgs]
            self.current_load = 0.0
        else:
            raise ValueError(f'Invalid response type: {response_type}')
    
    def return_courier_status(self):
        """
        Publish courier status.
        """
        self.pub_status_to_courier_manager.publish(String(data=json.dumps({
            'id': self.id,
            'status': self.status,
            'task_statuses': {self.tasks[i]['id'] : self.tasks[i]['status'] for i in range(len(self.tasks))},
            'load': self.current_load})))


    def update_task_status(self, new_status:str, task_id:str=None, all=False):
        status_updated = False or len(self.tasks) == 0
        for t in self.tasks:
            try:
                if all:
                    t['status'] = new_status
                    status_updated = True
                elif t['id'] == task_id: # BUG: sometimes key 'id' is not found. Why?
                    t['status'] = new_status
                    status_updated = True
                    break
            except Exception as e:
                self.get_logger().error(f"Problem with self.tasks: {self.tasks}")
                raise e
        if not status_updated:
            raise Exception(f"Task with id {task_id} not found in tasks: {self.tasks}")

def main(args=None):
    rclpy.init(args=args)

    courier_node = RobotNode()


    try:
        rclpy.spin(courier_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print(f"Clean shutdown: Loader{courier_node.id} agent")
    else:
        rclpy.shutdown()

    courier_node.destroy_node()



if __name__ == "__main__":
    main()