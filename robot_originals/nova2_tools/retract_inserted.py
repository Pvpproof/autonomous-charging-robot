#!/usr/bin/env python3
"""Home withdraws to the pre-insertion pose before normal homing."""
import sys,rclpy
from insert_back import Retreat,run

def main():
 rclpy.init();n=Retreat()
 try:run(n,confirmed=True);return 0
 except (Exception,KeyboardInterrupt) as error:print('INSERT_RETRACT_REFUSED:',error);return 2
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':sys.exit(main())
