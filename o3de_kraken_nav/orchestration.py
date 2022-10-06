from importlib.resources import path
from rclpy.node import Node
import rclpy
import re
import pathlib
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
import time
import threading

class WaypointSender(Node):
  def __init__(self):
    super().__init__('waypoints_sender')
    self.goal_succeeded = True
    self.wps = self.parse(pathlib.Path(__file__).parent.joinpath("line0.txt"))
    self.log_sub = self.create_subscription(Log, "/rosout", self.log_cb, 10)
    self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    self.thread = threading.Thread(target=self.coroutine)
    self.thread.start()

  def coroutine(self):
    for wp in self.wps:
      pos_msg = PoseStamped()
      pos_msg.header.frame_id = "map"
      pos_msg.pose.position.x = float(wp[0])
      pos_msg.pose.position.y = float(wp[1])

      pos_msg.pose.orientation.x = float(wp[3])
      pos_msg.pose.orientation.y = float(wp[4])
      pos_msg.pose.orientation.z = float(wp[5])
      pos_msg.pose.orientation.w = float(wp[6])

      self.goal_succeeded = False
      self.goal_pub.publish(pos_msg)
      print(f">>> Going to a tree: ({float(wp[0])}, {float(wp[2])}) ({float(wp[3])}{float(wp[4])}{float(wp[5])}{float(wp[6])}) \n")

      while not self.goal_succeeded:
        time.sleep(0.1)
      
      print(">>> Collecting apples... \n")
      time.sleep(3)
      print(">>> Apples collected. \n")
      time.sleep(1)

  def log_cb(self, msg):
      if msg.msg == "Goal succeeded":
        self.goal_succeeded = True

  def parse(self, file):
    print(">>> Loading path file...")
    r = re.compile(r'position:\s+x:(\s-*[0-9]+\.[0-9]+)\s+y:(\s-*[0-9]+\.[0-9]+)\s+z:(\s-*[0-9]+\.[0-9]+)\s+orientation:\s+x:(\s-*[0-9]+\.[0-9]+)\s+y:(\s-*[0-9]+\.[0-9]+)\s+z:(\s-*[0-9]+\.[0-9]+)\s+w:(\s-*[0-9]+\.[0-9]+)')
    with open(file, 'r') as f:
        wps = f.read()
        # print(wps)
        res = r.findall(wps)
        # print(res)
        return res

def main(args=None): 
  rclpy.init(args=args)
  ws = WaypointSender()
  rclpy.spin(ws)
  
  ws.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()