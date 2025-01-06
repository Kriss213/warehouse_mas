#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from flask import Flask
from flask_socketio import SocketIO, emit
from flask import render_template
import os
import json

BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)).replace('lib', 'share'), "system_manager")
app = Flask(
    __name__,
    template_folder=os.path.join(BASE_DIR, "templates"),
    static_folder=os.path.join(BASE_DIR, "static"))
socketio = SocketIO(app)

@app.route('/en') 
def index_en():
    tf = os.path.join(BASE_DIR, "templates")
    print(f"template folder: {tf}")
    return render_template('index_en.html')

@app.route('/') 
def index_lv():
    tf = os.path.join(BASE_DIR, "templates")
    print(f"template folder: {tf}")
    return render_template('index.html')


class SystemManagerNode(Node):
    """
    System manager agent.

    Percepts:
        * Request from GUI
        * Status updates from managers and drop-off point.
    Actions:
        * Request prioritization.
        * Request allocation to drop-off and robot managers.
        * Status information to GUI.
    Sent messages:
        * Request to drop-off manager.
        * Request to robot manager.
        * Status information to GUI.
    Received messages:
        * Request from GUI.
        * Status information from managers and drop-off point.
        * Confirmation from drop-off manager.
 
    """
    def __init__(self):
        super().__init__('SystemManagerNode')

        # request queues for different priorities
        self.request_queue_low = []
        self.request_queue_medium = []
        self.request_queue_high = []

        # Subscribers for receiving status updates
        self.sub_status_update = self.create_subscription(String, 'status_update', self.status_update_callback, 10)

        # Publisher to pass on requests to loader manager
        self.pub_request = self.create_publisher(String, 'request_to_loader_manager', 10)
        
        self.init_socketio_events()


        # create a timer to assign requests to loaders
        self.assign_requests_timer = self.create_timer(3.0, self.assign_requests)



        self.get_logger().info("System manager initialized successfully!")

    def status_update_callback(self, msg:String):
        """
        Handle status updates from managers and drop-off point. Emit status update to GUI.
        This handles the status updates of requests, robots, loaders and drop-off point.
        """

        # can recieve types: request, robot, loader, drop-off

        status_msg = json.loads(msg.data)
        status_type = status_msg['type']
        #self.get_logger().info(f"Received status update for type: {status_type}")

        if status_type == 'request':
            request = json.loads(status_msg['request'])
            socketio.emit('request_status', request)
        elif status_type == 'loader':
            socketio.emit('loader_status', {
                'id': status_msg['id'],
                'status': status_msg['status'],
                'name': f"Uzkrāvējs {status_msg['id']}"
            })
        elif status_type == 'courier':
            socketio.emit('courier_update', {
                'id': status_msg['id'],
                'status': status_msg['status'],
                'load': status_msg['load'],
                'name': f"Robots {status_msg['id']}"
            })
        elif status_type == 'unload_point':
            socketio.emit('unloading_point_status', status_msg['status'])
            
        else:
            raise ValueError(f'Invalid status type: {status_type}')
    
    def assign_requests(self):
        """
        Assign requests to loaders from request queue.
        """
        if len(self.request_queue_high) > 0:
            request = self.request_queue_high.pop(0)
        elif len(self.request_queue_medium) > 0:
            request = self.request_queue_medium.pop(0)
        elif len(self.request_queue_low) > 0:
            request = self.request_queue_low.pop(0)
        else:
            return
                
        self.get_logger().info(f"Request with id: {request['id']} assigned to loader manager.")
        self.pub_request.publish(String(data=json.dumps(request)))
        

    def init_socketio_events(self):
        """
        Initialize events for socketio. 
        """
        @socketio.on('connect')
        def handle_connect():
            self.get_logger().info("client connected")
            emit('connected', {'data': 'Connected'})
                
        @socketio.on('new_request')
        def handle_new_request(request_data):
            """
            Pass request to loader manager.
            
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
        
            # add request to request queue
            priority = request_data["priority"]
            if priority == "low":
                self.request_queue_low.append(request_data)
            elif priority == "normal":
                self.request_queue_medium.append(request_data)
            elif priority == "high":
                self.request_queue_high.append(request_data)
            else:
                raise ValueError(f"Invalid priority value: {priority}")
            
            self.get_logger().info(f"Request with id: {request_data['id']} with priority {priority} added to queue.")

            # request = String()
            # request.data = json.dumps(data)
            # self.pub_request.publish(request)
            # self.get_logger().info(f"request passed to loader manager")


    


def main(args=None):
    rclpy.init(args=args)

    sys_mng_node = SystemManagerNode()

    flask_thread = threading.Thread(target=lambda: app.run(debug=True, use_reloader=False))
    flask_thread.daemon = True # kill thread when main program exits
    flask_thread.start()
    
    try:
        rclpy.spin(sys_mng_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: system manager agent")
    else:
        rclpy.shutdown()

    sys_mng_node.destroy_node()



if __name__ == "__main__":
    main()