'''How It Works (The "Velocity Trick")Even though you type a distance (Position), this node actually sends Speed (Velocity). 
It uses the physics formula: Time = Distance / Speed.Input: You type y 20 (Move 20 cm in Y).

Calculation:The node has a fixed speed of 10 cm/s ($0.1$ m/s).It calculates the duration: $20 \text{ cm} / 10 \text{ cm/s} = \mathbf{2.0 \text{ seconds}}$
.
Execution Loop:Instead of saying "Go to Y=20", it says "Move at speed 0.1 m/s".

It keeps repeating this command 10 times a second (10 Hz).It stops exactly after 2.0 seconds.'''

'''
Technical Details
Node Name: precision_mover.

Publisher Topic: /delta_twist_cmds.

This is the standard input topic for MoveIt Servo.

Message Type: geometry_msgs/msg/TwistStamped.

Twist contains Linear Velocity (x, y, z) and Angular Velocity (pitch, roll, yaw).

Stamped means it includes a Header with a Frame ID and Timestamp.

Frame ID: base_link.

This tells the robot that "X" means "Forward relative to the base", not relative to the gripper.'''




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class PrecisionMover(Node):
    def __init__(self):
        super().__init__('precision_mover')
        self.publisher_ = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)
        
        # SPEED SETTINGS
        self.linear_speed = 0.1  # 0.1 m/s = 10 cm/s
        self.target_frame = 'base_link' 
        
    def move_cm(self, distance_cm, axis='x'):
        """
        Moves the robot a specific distance in cm along an axis.
        distance_cm: float (e.g., 5.0, -10.0)
        axis: 'x', 'y', or 'z'
        """
        
        # 1. Calculate Duration needed
        # Distance (m) / Speed (m/s) = Time (s)
        distance_meters = distance_cm / 100.0
        duration_sec = abs(distance_meters / self.linear_speed)
        
        # Determine Direction (+1 or -1)
        direction = 1.0 if distance_meters > 0 else -1.0
        velocity = self.linear_speed * direction

        print(f"--- MOVING {distance_cm} cm in {axis.upper()} ---")
        print(f"Speed: {self.linear_speed*100} cm/s | Time: {duration_sec:.2f} sec")

        # 2. Prepare Message
        msg = TwistStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if axis == 'x': msg.twist.linear.x = velocity
        elif axis == 'y': msg.twist.linear.y = velocity
        elif axis == 'z': msg.twist.linear.z = velocity

        # 3. Execution Loop
        # We use a rate to publish continuously for the calculated duration
        rate = self.create_rate(100) # 10 Hz
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9
            
            if elapsed > duration_sec:
                break
                
            # Update timestamp and publish
            msg.header.stamp.sec = 0       # Force 0 again inside the loop
            msg.header.stamp.nanosec = 0
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 4. STOP (Crucial!)
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = self.target_frame
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(stop_msg)
        print("--- MOVEMENT COMPLETE ---")

def main():
    rclpy.init()
    mover = PrecisionMover()

    try:
        # EXAMPLE: Ask user for input
        while True:
            user_input = input("Enter command (e.g., 'x 5' or 'z -10'): ")
            parts = user_input.split()
            
            if len(parts) == 2:
                axis = parts[0]
                dist = float(parts[1])
                mover.move_cm(dist, axis)
            else:
                print("Invalid format. Try: x 10")
                
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()