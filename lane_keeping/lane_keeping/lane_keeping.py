import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [mask, contour, crosshair])

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):
        msg = Twist()
        msg.linear.x = 1
        msg.angular.z = 0.0

        #img=img[330:480,:]

        rows, cols = img.shape[:2]

        # Convert to HSV and threshold white
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        stackedMask = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Split left and right line based on image center
        left_centroids = []
        right_centroids = []
        for c in contours:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            if cx < cols // 2:
                left_centroids.append((cx, cy))
            else:
                right_centroids.append((cx, cy))

        def average_centroid(points):
            if not points:
                return None
            x = int(np.mean([pt[0] for pt in points]))
            y = int(np.mean([pt[1] for pt in points]))
            return (x, y)

        left_center = average_centroid(left_centroids)
        right_center = average_centroid(right_centroids)

        # Draw centroids
        if left_center:
            cv2.circle(contourMask, left_center, 10, (0, 255, 0), -1)
        if right_center:
            cv2.circle(contourMask, right_center, 10, (255, 0, 0), -1)

        # Compute target point
        if left_center and right_center:
            target_x = (left_center[0] + right_center[0]) // 2
        elif left_center:
            target_x = left_center[0] + 300  # bias right
        elif right_center:
            target_x = right_center[0] - 300  # bias left
        else:
            target_x = cols // 2  # fallback to straight

        # Draw navigation lines
        cv2.line(crosshairMask, (target_x, 0), (target_x, rows), (0, 0, 255), 2)
        cv2.line(crosshairMask, (cols//2, 0), (cols//2, rows), (255, 0, 0), 2)

        # Compute angular velocity
        offset = cols//2 - target_x
        msg.angular.z = float(offset)*float(0.02)
        msg.linear.x = 1
        msg.linear.x = float(msg.linear.x)
        msg.angular.z = float(msg.angular.z)

        self.publisher.publish(msg)

        return white_mask, contourMask, crosshairMask

    # Convert to RGB channels
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B

    # Apply threshold and result a binary image
    def threshold_binary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255

    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, size=(160, 120)):

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()