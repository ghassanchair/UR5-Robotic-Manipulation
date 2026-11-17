### Error: Variable `Robot` not recognized

**Error message:**

```text
Error evaluating parameter 'RigidBodyTree' in
'manipulator_ur5/Computed Torque Controller/Gravity Torque'

Caused by:
Unrecognized function or variable 'Robot'.
Variable 'Robot' has been deleted from base workspace.
```

**Cause:**

The variable `Robot`, which contains the UR5 robot model, is missing from the MATLAB Base Workspace.

**Solution:**

Create the variable by executing the following command in MATLAB:

```matlab
Robot = loadrobot("universalUR5", ...
    DataFormat="column", ...
    Gravity=[0 0 -9.81]);
```

**Note:** Execute this command before running the Simulink model. Make sure that the variable name `Robot` matches the name specified in the `RigidBodyTree` parameter of the `Gravity Torque` block.
