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