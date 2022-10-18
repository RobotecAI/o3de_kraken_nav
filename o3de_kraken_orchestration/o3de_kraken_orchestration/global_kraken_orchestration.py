import rclpy
import time
from threading import Thread

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from .state_machine import GlobalOrchestrationSM
from .orchestration_node import KrakenOrchestrationNode

class GlobalOrchestration(GlobalOrchestrationSM, KrakenOrchestrationNode):
    def __init__(self):
        rclpy.init()

        GlobalOrchestrationSM.__init__(self)
        KrakenOrchestrationNode.__init__(self)

        spin_thread = Thread(target=rclpy.spin, args=(self,))
        spin_thread.start()

        self.poses = []
        self.current_index = 0

    def get_next_pose(self):
        if self.current_index >= len(self.poses):
            print('Invalid index!')
            return PoseStamped()
        
        pose = self.poses[self.current_index]
        self.current_index += 1
        return pose

    def __del__(self):
        self.destroy_node()
        rclpy.shutdown()

    def on_enter_waiting(self):
        print('Has the simulation started?')
        self.send_state("Wait for connection")
        # Perform a check whether the simulation has started or not.
        if (self.sim_started):
            self.started()
        else:
            self.sim_started = True
            self.await_start()

    def on_enter_spawning(self):
        print('Spawning the robot.')
        self.send_state("Spawning the robot")
        if self.spawn_kraken():
            self.spawned()
        else:
            print('Robot spawning failed.')

    def on_spawned(self):
        print('Robot spawned successfully. Fetching info.')
        pose = self.get_spawn_point_pose()
        self.poses = self.get_plan_poses(pose)
        # Give nav stack a few seconds before we start first goal point
        print('Waiting for robot sensors to latch...')
        time.sleep(3)
        
    def on_enter_gathering(self):
        print('Is the robot in a gathering position?')
        # Perform a check whether the robot is in a gathering position or not.
        if (self.can_gather):
            self.gather()
        else:
            self.navigate()

    def on_gather(self):
        print('Gathering the apples.')
        # Instruct the robot to start gathering apples.
        self.has_finished = not self.has_finished
        pass

    def on_enter_finishing(self):
        print('Has the robot completed its job?')
        # Perform a check whether the robot has finished its job or not.
        if (self.current_index >= len(self.poses)):
            self.finished()
        else:
            self.finish()

    def on_navigate(self):
        self.send_state("Navigating")
        print('Navigating the robot to a closest gathering position?')
        self.can_gather = False
        # Instruct the robot to navigate to the closest gathering position.
        self.navigate_to_pose(self.get_next_pose())

        while not self.goal_pose_reached:
            time.sleep(1)
            
        self.goal_pose_reached = False
        self.can_gather = True

    def on_gather(self):
        print("Requesting gathering...")
        self.send_state("Gather %.1f "% self.apple_progress)
        trigger_res = self.trigger_apple_gathering()
        print(trigger_res.message)

        if trigger_res.success:
            while not self.gathering_done:
                self.send_state("Gather %.0f %%"% self.apple_progress)
                time.sleep(0.2)
            
            print("Done gathering apples.")
        else:
            print("Freeing up the AppleKraken...")
            cancel_res = self.cancel_apple_gathering()
            print(cancel_res.message)
            time.sleep(2)

    def on_finish(self):
        # Same action is performed.
        self.on_navigate()

    def on_finished(self):
        self.send_state("Completed")
        print('The robot has completed its job.')
        # The robot has finished its job.
        pass

def main():
    global_orch = GlobalOrchestration()
    global_orch.run()

if __name__ == '__main__':
    main()
