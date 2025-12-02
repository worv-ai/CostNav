"""
This is the implementation of the OGN node defined in OgnRmpController.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import math
import numpy as np
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.rmp_robot.ogn.OgnRmpControllerDatabase import OgnRmpControllerDatabase

class OgnRmpControllerInternalState(BaseResetNode):
    def __init__(self):
        self.linear_velocity = 0.0
        self.front_wheel_angle = 0.0
        self.min_turning_radius = None
        self.node = None
        self.graph_id = None
        super().__init__(initialize=False)

    def custom_reset(self):
        if self.initialized:
            self.node.get_attribute("outputs:frontWheelAngle").set(0.0)
            self.node.get_attribute("outputs:velocityCommand").set([0.0, 0.0, 0.0, 0.0])


class OgnRmpController:    
    """
         Rmp Controller
    """
    @staticmethod
    def init_instance(node, graph_instance_id):
        state = OgnRmpControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnRmpControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
            state.initialized = False

    @staticmethod
    def internal_state():
        return OgnRmpControllerInternalState()
        
    @staticmethod
    def compute(db:OgnRmpControllerDatabase) -> bool:
        """Compute the outputs from the current input"""
        state:OgnRmpControllerInternalState = db.per_instance_state
        try:
            # Initialize the minimum turning radius
            if not state.initialized:
                state.min_turning_radius = db.inputs.wheelBase * (0.25 + 1 / math.tan(db.inputs.maxWheelRotation) ** 2) ** 0.5
                state.initialized = True
            
            # Compute the linear velocity
            target_linear_velocity = np.clip(
                db.inputs.linearVelocity, -db.inputs.maxLinearSpeed, db.inputs.maxLinearSpeed)
            if math.fabs(target_linear_velocity) > math.fabs(state.linear_velocity) and target_linear_velocity * state.linear_velocity >= 0:
                state.linear_velocity = max(
                    state.linear_velocity - db.inputs.maxAcceleration * db.inputs.dt,
                    min(
                        target_linear_velocity,
                        state.linear_velocity + db.inputs.maxAcceleration * db.inputs.dt
                    )
                )
            else:
                state.linear_velocity = max(
                    state.linear_velocity - db.inputs.maxDeceleration * db.inputs.dt,
                    min(
                        target_linear_velocity,
                        state.linear_velocity + db.inputs.maxDeceleration * db.inputs.dt
                    )
                )
            
            # Compute the front wheel angle
            if db.inputs.angularVelocity != 0:
                target_angular_velocity = np.clip(
                    db.inputs.angularVelocity, -db.inputs.maxAngularSpeed, db.inputs.maxAngularSpeed)
                target_turning_radius = target_linear_velocity / target_angular_velocity
                if math.fabs(target_turning_radius) < state.min_turning_radius:
                    if target_turning_radius < 0:
                        target_turning_radius = -state.min_turning_radius
                    else:
                        target_turning_radius = state.min_turning_radius
                cot_square = (target_turning_radius ** 2) / (db.inputs.wheelBase ** 2) - 0.25
                target_front_wheel_angle = math.atan(
                    1 / (cot_square ** 0.5)
                )
                if db.inputs.angularVelocity < 0:
                    target_front_wheel_angle *= -1
            else:
                target_front_wheel_angle = 0

            state.front_wheel_angle = max(
                state.front_wheel_angle - db.inputs.maxWheelRotationVelocity * db.inputs.dt,
                min(
                    target_front_wheel_angle,
                    state.front_wheel_angle + db.inputs.maxWheelRotationVelocity * db.inputs.dt
                )
            )
            state.front_wheel_angle = np.clip(
                state.front_wheel_angle, -db.inputs.maxWheelRotation, db.inputs.maxWheelRotation)
            db.outputs.frontWheelAngle = state.front_wheel_angle
            
            # Compute the velocity command
            if state.front_wheel_angle == 0.0:
                db.outputs.velocityCommand = 4 * [state.linear_velocity / db.inputs.wheelRadius]
            else:
                if state.linear_velocity == 0.0:
                    if math.fabs(state.front_wheel_angle) == db.inputs.maxWheelRotation:
                        wheel_angular_velocity = db.inputs.trackWidth / 2 * db.inputs.angularVelocity / db.inputs.wheelRadius
                        cos = math.cos(state.front_wheel_angle)
                        db.outputs.velocityCommand = [
                            -wheel_angular_velocity * cos,
                            wheel_angular_velocity * cos,
                            -wheel_angular_velocity,
                            wheel_angular_velocity
                        ]
                    else:
                        wheel_angular_velocity = db.inputs.trackWidth / 2 * db.inputs.maxWheelRotationVelocity / db.inputs.wheelRadius
                        if state.front_wheel_angle < 0:
                            wheel_angular_velocity *= -1
                        if target_front_wheel_angle == 0:
                            wheel_angular_velocity *= -1
                        db.outputs.velocityCommand = [
                            -wheel_angular_velocity,
                            +wheel_angular_velocity,
                            0,
                            0
                        ]
                else:
                    turning_radius_front = db.inputs.wheelBase / math.sin(state.front_wheel_angle)
                    turning_radius_rear = db.inputs.wheelBase / math.tan(state.front_wheel_angle)
                    
                    turning_radius = db.inputs.wheelBase * (0.25 + 1 / math.tan(state.front_wheel_angle) ** 2) ** 0.5
                    if state.front_wheel_angle < 0:
                        turning_radius *= -1
                    
                    angular_velocity = state.linear_velocity / turning_radius
                    
                    db.outputs.velocityCommand = [
                        angular_velocity * (turning_radius_front - db.inputs.trackWidth/2) / db.inputs.wheelRadius,
                        angular_velocity * (turning_radius_front + db.inputs.trackWidth/2) / db.inputs.wheelRadius,
                        angular_velocity * (turning_radius_rear - db.inputs.trackWidth/2) / db.inputs.wheelRadius,
                        angular_velocity * (turning_radius_rear + db.inputs.trackWidth/2) / db.inputs.wheelRadius
                    ]
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True
