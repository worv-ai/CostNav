.. _OmniIsaacRmp_robotExtension_RmpController_1:

.. _OmniIsaacRmp_robotExtension_RmpController:

.. ================================================================================
.. THIS PAGE IS AUTO-GENERATED. DO NOT MANUALLY EDIT.
.. ================================================================================

:orphan:

.. meta::
    :title: Rmp Controller
    :keywords: lang-en omnigraph node omniisaacrmp_robotextension rmp-controller


Rmp Controller
==============

.. <description>

Rmp Controller

.. </description>


Installation
------------

To use this node enable :ref:`omni.isaac.rmp_robot<ext_omni_isaac_rmp_robot>` in the Extension Manager.


Inputs
------
.. csv-table::
    :header: "Name", "Type", "Descripton", "Default"
    :widths: 20, 20, 50, 10

    "Angular Velocity (*inputs:angularVelocity*)", "``double``", "desired target rotation velocity", "0.0"
    "Dt (*inputs:dt*)", "``double``", "delta time", "0.0"
    "Exec In (*inputs:execIn*)", "``execution``", "The input execution", "0"
    "Linear Velocity (*inputs:linearVelocity*)", "``double``", "desired target linear velocity", "0.0"
    "Max Acceleration (*inputs:maxAcceleration*)", "``double``", "maximum linear acceleration of the robot for forward and reverse, 0.0 means not set", "2.8"
    "Max Angular Speed (*inputs:maxAngularSpeed*)", "``double``", "max angular speed allowed for vehicle, 0.0 means not set", "1.8"
    "Max Deceleration (*inputs:maxDeceleration*)", "``double``", "maximum linear braking of the robot, 0.0 means not set.", "2.8"
    "Max Linear Speed (*inputs:maxLinearSpeed*)", "``double``", "max linear speed allowed for vehicle, 0.0 means not set", "2.5"
    "Max Wheel Rotation (*inputs:maxWheelRotation*)", "``double``", "Maximum angle of rotation for the front wheels in radians", "0.349"
    "Max Wheel Rotation Velocity (*inputs:maxWheelRotationVelocity*)", "``double``", "Maximum velocity of rotation for the front wheels in radians", "1.257"
    "Track Width (*inputs:trackWidth*)", "``double``", "Distance between the left and right rear wheels of the robot in meters", "0.545"
    "Wheel Base (*inputs:wheelBase*)", "``double``", "Distance between the front and rear axles of the robot in meters", "0.456"
    "Wheel Radius (*inputs:wheelRadius*)", "``double``", "radius of the wheels", "0.11"


Outputs
-------
.. csv-table::
    :header: "Name", "Type", "Descripton", "Default"
    :widths: 20, 20, 50, 10

    "Front Wheel Angle (*outputs:frontWheelAngle*)", "``double``", "Angle for the front turning wheel in radians", "0.0"
    "Velocity Command (*outputs:velocityCommand*)", "``double[]``", "velocity commands", "[0.0, 0.0, 0.0, 0.0]"


Metadata
--------
.. csv-table::
    :header: "Name", "Value"
    :widths: 30,70

    "Unique ID", "OmniIsaacRmp_robotExtension.RmpController"
    "Version", "1"
    "Extension", "omni.isaac.rmp_robot"
    "Has State?", "False"
    "Implementation Language", "Python"
    "Default Memory Type", "cpu"
    "Generated Code Exclusions", "None"
    "uiName", "Rmp Controller"
    "Generated Class Name", "OgnRmpControllerDatabase"
    "Python Module", "omni.isaac.rmp_robot"

