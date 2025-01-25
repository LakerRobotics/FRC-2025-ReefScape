/*Here's a basic outline of how you could structure a vision subsystem using LimeLight in Java:

1.  Setup LimeLight NetworkTables: LimeLight uses NetworkTables to communicate with the robot code. You'll need to access these tables to get data from the LimeLight.

2.  Create a Vision Subsystem Class: This class will handle the logic for processing data from the LimeLight.

3.  Implement Methods to Retrieve Data: These methods will pull data from the LimeLight, such as target offsets, target presence, and more.

4.  Add Control Methods: Implement methods to use the vision data for robot control, like aligning to a target.

Here's a simple example code:

*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    // NetworkTable for LimeLight
    private final NetworkTable limeLightTable;

    public VisionSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTargetOffsetX() {
        // "tx" is the horizontal offset from crosshair to target
        NetworkTableEntry tx = limeLightTable.getEntry("tx");
        return tx.getDouble(0.0);
    }

    public double getTargetOffsetY() {
        // "ty" is the vertical offset from crosshair to target
        NetworkTableEntry ty = limeLightTable.getEntry("ty");
        return ty.getDouble(0.0);
    }

    public boolean isTargetVisible() {
        // "tv" indicates whether a target is visible
        NetworkTableEntry tv = limeLightTable.getEntry("tv");
        return tv.getDouble(0) == 1;
    }

    // Add more methods as needed for robot control
}

