import rclpy
from rclpy.executors import MultiThreadedExecutor
from .generic_planner_node import GenericPlannerNode
from .p2p_jointspace_planner import JointSpacePlanner
from interface.action import PlanJointSpace

def main(args=None):
    rclpy.init(args=args)

    # 1. Instanziiere die spezifische Planer-Logik
    # Diese Klasse sollte die .calculate(request, progress_cb, is_canceled_cb) Methode haben
    planner = JointSpacePlanner(50, 100)

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