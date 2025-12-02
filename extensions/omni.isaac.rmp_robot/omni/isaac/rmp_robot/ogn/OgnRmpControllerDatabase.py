"""Support for simplified access to data on nodes of type OmniIsaacRmp_robotExtension.RmpController

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Rmp Controller
"""

import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnRmpControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type OmniIsaacRmp_robotExtension.RmpController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.angularVelocity
            inputs.dt
            inputs.execIn
            inputs.linearVelocity
            inputs.maxAcceleration
            inputs.maxAngularSpeed
            inputs.maxDeceleration
            inputs.maxLinearSpeed
            inputs.maxWheelRotation
            inputs.maxWheelRotationVelocity
            inputs.trackWidth
            inputs.wheelBase
            inputs.wheelRadius
        Outputs:
            outputs.frontWheelAngle
            outputs.velocityCommand
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 77, 0)
    TARGET_VERSION = (2, 170, 0)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:angularVelocity', 'double', 0, None, 'desired target rotation velocity', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:dt', 'double', 0, None, 'delta time', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:linearVelocity', 'double', 0, None, 'desired target linear velocity', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:maxAcceleration', 'double', 0, None, 'maximum linear acceleration of the robot for forward and reverse, 0.0 means not set', {ogn.MetadataKeys.DEFAULT: '2.8'}, True, 2.8, False, ''),
        ('inputs:maxAngularSpeed', 'double', 0, None, 'max angular speed allowed for vehicle, 0.0 means not set', {ogn.MetadataKeys.DEFAULT: '1.8'}, True, 1.8, False, ''),
        ('inputs:maxDeceleration', 'double', 0, None, 'maximum linear braking of the robot, 0.0 means not set.', {ogn.MetadataKeys.DEFAULT: '2.8'}, True, 2.8, False, ''),
        ('inputs:maxLinearSpeed', 'double', 0, None, 'max linear speed allowed for vehicle, 0.0 means not set', {ogn.MetadataKeys.DEFAULT: '2.5'}, True, 2.5, False, ''),
        ('inputs:maxWheelRotation', 'double', 0, None, 'Maximum angle of rotation for the front wheels in radians', {ogn.MetadataKeys.DEFAULT: '0.349'}, True, 0.349, False, ''),
        ('inputs:maxWheelRotationVelocity', 'double', 0, None, 'Maximum velocity of rotation for the front wheels in radians', {ogn.MetadataKeys.DEFAULT: '1.257'}, True, 1.257, False, ''),
        ('inputs:trackWidth', 'double', 0, None, 'Distance between the left and right rear wheels of the robot in meters', {ogn.MetadataKeys.DEFAULT: '0.545'}, True, 0.545, False, ''),
        ('inputs:wheelBase', 'double', 0, None, 'Distance between the front and rear axles of the robot in meters', {ogn.MetadataKeys.DEFAULT: '0.456'}, True, 0.456, False, ''),
        ('inputs:wheelRadius', 'double', 0, None, 'radius of the wheels', {ogn.MetadataKeys.DEFAULT: '0.11'}, True, 0.11, False, ''),
        ('outputs:frontWheelAngle', 'double', 0, None, 'Angle for the front turning wheel in radians', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('outputs:velocityCommand', 'double[]', 0, None, 'velocity commands', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0, 0.0], False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"angularVelocity", "dt", "execIn", "linearVelocity", "maxAcceleration", "maxAngularSpeed", "maxDeceleration", "maxLinearSpeed", "maxWheelRotation", "maxWheelRotationVelocity", "trackWidth", "wheelBase", "wheelRadius", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.angularVelocity, self._attributes.dt, self._attributes.execIn, self._attributes.linearVelocity, self._attributes.maxAcceleration, self._attributes.maxAngularSpeed, self._attributes.maxDeceleration, self._attributes.maxLinearSpeed, self._attributes.maxWheelRotation, self._attributes.maxWheelRotationVelocity, self._attributes.trackWidth, self._attributes.wheelBase, self._attributes.wheelRadius]
            self._batchedReadValues = [0.0, 0.0, 0, 0.0, 2.8, 1.8, 2.8, 2.5, 0.349, 1.257, 0.545, 0.456, 0.11]

        @property
        def angularVelocity(self):
            return self._batchedReadValues[0]

        @angularVelocity.setter
        def angularVelocity(self, value):
            self._batchedReadValues[0] = value

        @property
        def dt(self):
            return self._batchedReadValues[1]

        @dt.setter
        def dt(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def linearVelocity(self):
            return self._batchedReadValues[3]

        @linearVelocity.setter
        def linearVelocity(self, value):
            self._batchedReadValues[3] = value

        @property
        def maxAcceleration(self):
            return self._batchedReadValues[4]

        @maxAcceleration.setter
        def maxAcceleration(self, value):
            self._batchedReadValues[4] = value

        @property
        def maxAngularSpeed(self):
            return self._batchedReadValues[5]

        @maxAngularSpeed.setter
        def maxAngularSpeed(self, value):
            self._batchedReadValues[5] = value

        @property
        def maxDeceleration(self):
            return self._batchedReadValues[6]

        @maxDeceleration.setter
        def maxDeceleration(self, value):
            self._batchedReadValues[6] = value

        @property
        def maxLinearSpeed(self):
            return self._batchedReadValues[7]

        @maxLinearSpeed.setter
        def maxLinearSpeed(self, value):
            self._batchedReadValues[7] = value

        @property
        def maxWheelRotation(self):
            return self._batchedReadValues[8]

        @maxWheelRotation.setter
        def maxWheelRotation(self, value):
            self._batchedReadValues[8] = value

        @property
        def maxWheelRotationVelocity(self):
            return self._batchedReadValues[9]

        @maxWheelRotationVelocity.setter
        def maxWheelRotationVelocity(self, value):
            self._batchedReadValues[9] = value

        @property
        def trackWidth(self):
            return self._batchedReadValues[10]

        @trackWidth.setter
        def trackWidth(self, value):
            self._batchedReadValues[10] = value

        @property
        def wheelBase(self):
            return self._batchedReadValues[11]

        @wheelBase.setter
        def wheelBase(self, value):
            self._batchedReadValues[11] = value

        @property
        def wheelRadius(self):
            return self._batchedReadValues[12]

        @wheelRadius.setter
        def wheelRadius(self, value):
            self._batchedReadValues[12] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"frontWheelAngle", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.velocityCommand_size = 4
            self._batchedWriteValues = { }

        @property
        def velocityCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            return data_view.get(reserved_element_count=self.velocityCommand_size)

        @velocityCommand.setter
        def velocityCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            data_view.set(value)
            self.velocityCommand_size = data_view.get_array_size()

        @property
        def frontWheelAngle(self):
            value = self._batchedWriteValues.get(self._attributes.frontWheelAngle)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.frontWheelAngle)
                return data_view.get()

        @frontWheelAngle.setter
        def frontWheelAngle(self, value):
            self._batchedWriteValues[self._attributes.frontWheelAngle] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnRmpControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnRmpControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnRmpControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'OmniIsaacRmp_robotExtension.RmpController'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnRmpControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnRmpControllerDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except Exception:
                db = OgnRmpControllerDatabase(node)

            try:
                compute_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnRmpControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnRmpControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnRmpControllerDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnRmpControllerDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnRmpControllerDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.rmp_robot")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Rmp Controller")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Rmp Controller")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnRmpControllerDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnRmpControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnRmpControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnRmpControllerDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("OmniIsaacRmp_robotExtension.RmpController")
