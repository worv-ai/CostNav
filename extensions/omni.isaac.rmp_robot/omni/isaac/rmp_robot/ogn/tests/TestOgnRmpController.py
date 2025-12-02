import os
import omni.graph.core as og
import omni.graph.core.tests as ogts


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.rmp_robot.ogn.OgnRmpControllerDatabase import OgnRmpControllerDatabase
        test_file_name = "OgnRmpControllerTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):  # pragma: no cover
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_OmniIsaacRmp_robotExtension_RmpController")
        database = OgnRmpControllerDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:  # pragma no cover
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"

        self.assertTrue(test_node.get_attribute_exists("inputs:angularVelocity"))
        attribute = test_node.get_attribute("inputs:angularVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.angularVelocity
        database.inputs.angularVelocity = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:dt"))
        attribute = test_node.get_attribute("inputs:dt")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.dt
        database.inputs.dt = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.execIn
        database.inputs.execIn = db_value
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:linearVelocity"))
        attribute = test_node.get_attribute("inputs:linearVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.linearVelocity
        database.inputs.linearVelocity = db_value
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxAcceleration"))
        attribute = test_node.get_attribute("inputs:maxAcceleration")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxAcceleration
        database.inputs.maxAcceleration = db_value
        expected_value = 2.8
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxAngularSpeed"))
        attribute = test_node.get_attribute("inputs:maxAngularSpeed")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxAngularSpeed
        database.inputs.maxAngularSpeed = db_value
        expected_value = 1.8
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxDeceleration"))
        attribute = test_node.get_attribute("inputs:maxDeceleration")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxDeceleration
        database.inputs.maxDeceleration = db_value
        expected_value = 2.8
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxLinearSpeed"))
        attribute = test_node.get_attribute("inputs:maxLinearSpeed")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxLinearSpeed
        database.inputs.maxLinearSpeed = db_value
        expected_value = 2.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxWheelRotation"))
        attribute = test_node.get_attribute("inputs:maxWheelRotation")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxWheelRotation
        database.inputs.maxWheelRotation = db_value
        expected_value = 0.349
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxWheelRotationVelocity"))
        attribute = test_node.get_attribute("inputs:maxWheelRotationVelocity")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.maxWheelRotationVelocity
        database.inputs.maxWheelRotationVelocity = db_value
        expected_value = 1.257
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:trackWidth"))
        attribute = test_node.get_attribute("inputs:trackWidth")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.trackWidth
        database.inputs.trackWidth = db_value
        expected_value = 0.545
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelBase"))
        attribute = test_node.get_attribute("inputs:wheelBase")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelBase
        database.inputs.wheelBase = db_value
        expected_value = 0.456
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:wheelRadius"))
        attribute = test_node.get_attribute("inputs:wheelRadius")
        self.assertTrue(attribute.is_valid())
        db_value = database.inputs.wheelRadius
        database.inputs.wheelRadius = db_value
        expected_value = 0.11
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:frontWheelAngle"))
        attribute = test_node.get_attribute("outputs:frontWheelAngle")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.frontWheelAngle
        database.outputs.frontWheelAngle = db_value

        self.assertTrue(test_node.get_attribute_exists("outputs:velocityCommand"))
        attribute = test_node.get_attribute("outputs:velocityCommand")
        self.assertTrue(attribute.is_valid())
        db_value = database.outputs.velocityCommand
        temp_setting = database.inputs._setting_locked
        database.inputs._testing_sample_value = True
        database.outputs._testing_sample_value = True
        database.inputs._setting_locked = temp_setting
        self.assertTrue(database.inputs._testing_sample_value)
        self.assertTrue(database.outputs._testing_sample_value)
