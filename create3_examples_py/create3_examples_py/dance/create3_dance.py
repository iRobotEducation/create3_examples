# Copyright 2021 iRobot Corporation. All Rights Reserved.

import rclpy
import sys
from create3_examples_py.dance import dance_choreograph as dc

def main(args=None):
    rclpy.init(args=args)
    cp = dc.ColorPalette()
    '''
    DANCE SEQUENCE is defined as a list of pairs with 
    (time to start action in seconds, action to take)
    '''
    DANCE_SEQUENCE = [
        (0.0, dc.Lights([cp.green, cp.green, cp.green, cp.green, cp.green, cp.green])),
        (0.0, dc.Move(0.0, 0.0)),
        (0.5, dc.Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
        (0.5, dc.Move(0.15, 70)),
        (1.0, dc.Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
        (1.0, dc.Move(0.15, -70)),
        (1.5, dc.Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
        (1.5, dc.Move(0.15, 70)),
        (2.0, dc.Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
        (2.0, dc.Move(0.15, -70)),
        (2.5, dc.Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
        (2.5, dc.Move(-0.15, 70)),
        (3.0, dc.Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
        (3.0, dc.Move(-0.15, -70)),
        (3.5, dc.Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
        (3.5, dc.Move(-0.15, 70)),
        (4.0, dc.Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
        (4.0, dc.Move(-0.15, -70)),
        (4.5, dc.Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
        (4.5, dc.Move(0.0, 60)),
        (5.0, dc.Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
        (5.5, dc.Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
        (6.0, dc.Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
        (6.5, dc.Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
        (6.5, dc.Move(0.0, -75)),
        (7.0, dc.Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
        (7.5, dc.Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
        (8.0, dc.Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
        (8.5, dc.Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
        (9.0, dc.Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
        (9.5, dc.Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
        (9.5, dc.Move(-0.15, 50)),
        (10.0, dc.Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
        (10.0, dc.Move(-0.15, -50)),
        (10.5, dc.Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
        (10.5, dc.Move(-0.15, 50)),
        (11.0, dc.Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
        (11.0, dc.Move(-0.15, -50)),
        (11.5, dc.Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
        (11.5, dc.Move(0.0, 75)),
        (13.0, dc.Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
        (13.0, dc.Move(0.0, 75)),
        (14.0, dc.Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
        (14.0, dc.Move(0.0, -100)),
        (15.5, dc.Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
        (15.5, dc.Move(0.0, 100)),
        (20.5, dc.Lights([cp.red, cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow])),
        (20.5, dc.Move(-0.15, -70)),
        (21.0, dc.Lights([cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow, cp.red])),
        (21.0, dc.Move(-0.15, 70)),
        (21.5, dc.Lights([cp.red, cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow])),
        (21.5, dc.Move(0.15, -70)),
        (22.0, dc.Lights([cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow, cp.red])),
        (22.0, dc.Move(0.15, 70)),
        (22.5, dc.FinishedDance())
        ]

    dance_publisher = None
    try: 
        dance_choreographer = dc.DanceChoreographer(DANCE_SEQUENCE)
        dance_publisher = dc.DanceCommandPublisher(dance_choreographer)
        rclpy.spin(dance_publisher)
    except dc.FinishedDance:
        print('Finished Dance')
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception in dance:', file=sys.stderr)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if dance_publisher:
            dance_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
