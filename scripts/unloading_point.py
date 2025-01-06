#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import threading

class UnloadingPoint(Node):
    """
    Unloading point agent.

    ### Percepts:
       * Courier arrival confirmation
       * Courier line number request
    ### Actions:
        * package unloading
        * line number assignment to courier
    ### Sent messages:
        * invitation to unload to courier
        * confirmation about package unloading
        * status update to system manager
    ### Received messages:
        * line number request from courier
    """
    def __init__(self):
        super().__init__('UnloadingPointNode')

        self.status = 'idle'
        self.courier_line = []

        # =============== FROM / TO ROBOT ==============
        # Line number request and arrival confirmation
        self.sub_robot_to_unload = self.create_subscription(String, 'courier_unloading_line', self.courier_request_clb, 10)

        # Invitation to unload and package unloading confirmation publisher
        self.pub_unload_to_robot = self.create_publisher(String, 'courier_unload', 10)

        # a timer to perform unloading task
        self.unload_timer = self.create_timer(5.0, self.unload_packages_invitation)

        # ============ TO SYSTEM MANAGER ===============
        # publisher to system manager
        self.pub_status_update = self.create_publisher(String, 'status_update', 10)
        
        
        self.get_logger().info(f"Unloading point initialized successfully!")

    def courier_request_clb(self, msg:String):
        """
        Handle courier's line number request and arrival confirmation.
        """

        response = json.loads(msg.data)
        response_type = response['type']
        courier_id = response['courier_id']

        if response_type == 'line_number_request':
            # insert in queue and sort by priority
            self.courier_line.append({
                'courier_id': courier_id,
                'priority': response['priority']
                })

            self.courier_line.sort(key= lambda x: x['priority'], reverse=True)


        elif response_type == 'arrival_confirmation':
            # unload package and send unload confirmation to courier. Update package status to system manager
            def action():
                self.status = 'unloading'
                # send status to sys mng
                self.send_status_update('unload_point', self.status)

                package_ids = response['package_ids']
                self.get_logger().info(f"Unloading packages with ids: {package_ids}")
                
                # send status update to system manager
                self.send_status_update('request', 'unloading', package_ids)

                time.sleep(5)
                #send confirmation
                self.pub_unload_to_robot.publish(String(data=json.dumps({
                    'courier_id': courier_id,
                    'type': 'unload_confirmation',
                    'unloaded_packages': package_ids
                })))
                self.get_logger().info(f'Packages from courier {courier_id} unloaded.')
                # send status update to system manager
                self.send_status_update('request', 'delivered', package_ids)

                # remove courier from line by constructing a new list and skipping existing value
                self.courier_line = [c for c in self.courier_line if c['courier_id'] != courier_id]
                self.status = 'idle'
                # send status to sys mng
                self.send_status_update('unload_point', self.status)

            threading.Thread(target=action).start()

                
        else:
            raise ValueError(f'Invalid response type: {response_type}')

    def unload_packages_invitation(self):
        """
        Send invitation to robot to arrive at unloading point. Called by timer.
        """
        if self.status != 'idle' or len(self.courier_line) == 0:
            return
        courier_id = self.courier_line[0]['courier_id']
        self.get_logger().info(f"Inviting courier {courier_id} to unload...")
        self.pub_unload_to_robot.publish(String(data=json.dumps({
            'courier_id': courier_id,
            'type': 'unload_invitation'
        })))

    def send_status_update(self, update_type, status, request_ids=[]):
        """
        Send status updates to system manager.
        """
        if update_type == 'request':
            for req_id in request_ids:
                self.pub_status_update.publish(String(data=json.dumps({
                    'type': 'request',
                    'request': json.dumps({
                        'id': req_id,
                        'status': status
                    })
                })))
        elif update_type == 'unload_point':
            self.pub_status_update.publish(String(data=json.dumps({
                'type': 'unload_point',
                'status': status
            })))
        else:
            raise ValueError(f'Invalid update type: {update_type}')


def main(args=None):
    rclpy.init(args=args)

    unloading_point_node = UnloadingPoint()

    try:
        rclpy.spin(unloading_point_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print(f"Clean shutdown: Unloadin point agent")
    else:
        rclpy.shutdown()

    unloading_point_node.destroy_node()



if __name__ == "__main__":
    main()