<<<<<<< Updated upstream
package first.robot;

import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Autonomous;



public final class HomingAuto implements OpMode{
  public HomingAuto(Robot robot) {
 //   robot.arm.homingElevator();

    RobotModeTriggers.autonomous().onTrue(robot.arm.homingElevator());
  }

}
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.SubsystemBase;

public class HomingAuto extends SubsystemBase {
  /** Creates a new HomingAuto. */
  public HomingAuto() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
>>>>>>> Stashed changes
