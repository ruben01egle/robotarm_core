import rclpy
from rclpy.executors import MultiThreadedExecutor

from .generic_planner_node import GenericPlannerNode
from .p2p_jointspace_planner import JointSpacePlanner
from interface.action import PlanJointSpace

import numpy as np

def main(args=None):
    rclpy.init(args=args)

    # 1. Instanziiere die spezifische Planer-Logik
    # Diese Klasse sollte die .calculate(request, progress_cb, is_canceled_cb) Methode haben
    # TODO: read limits for all joints form urdf
    num_joints = 6

    # 2. Initialisiere die Arrays für die Limits
    # (Werte sind hier beispielhaft gewählt)
    min_pos = np.array([ np.deg2rad(-180.0), np.deg2rad(-160.0), np.deg2rad(-120.0), np.deg2rad(-180.0), np.deg2rad(-180.0), np.deg2rad(-180.0)]) # in Grad oder Rad
    max_pos = np.array([ np.deg2rad(+180.0), np.deg2rad(+20.00), np.deg2rad(+135.0), np.deg2rad(+180.0), np.deg2rad(+180.0), np.deg2rad(+180.0)])
    max_v   = np.array([ np.deg2rad(+180.0), np.deg2rad(+180.0), np.deg2rad(+180.0), np.deg2rad(+180.0), np.deg2rad(+180.0), np.deg2rad(+180.0)])  # Einheiten/s
    max_tau = np.array([ 6.0,   30.0,   12.0,   4.0,   4.0,   4.0])  # Hier als max_acc genutzt

    # 3. Instanziiere den Planer mit diesen Arrays
    planner = JointSpacePlanner(
        min_pos=min_pos,
        max_pos=max_pos,
        max_v=max_v,
        max_tau=max_tau,
        dt=0.001            # TODO: magic numbers
    )

    # 2. Erstelle die generische Node
    # Wir übergeben: Node-Name, Action-Name, Action-Typ, Logik-Instanz
    node = GenericPlannerNode(
        'joint_planner_node',
        'plan_trajectory/p2p_jointspace',
        PlanJointSpace,
        planner
    )

    # 3. Da du eine ReentrantCallbackGroup nutzt, ist ein MultiThreadedExecutor ratsam
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()