// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Degrees;

import first.robot.constants.ArmConstants;
import first.robot.constants.DriverConstants;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.units.measure.Angle;

/*  This is a Teleop Mode.  Multiple Teleop Modes can be defined and selected from the Drive
* Station.  When this one is seleceted it will just do the arm.
 */



@Teleop
public class ArmTeleop extends PeriodicOpMode {
  /** How often the stick and arm angles are echoed to the console, in seconds. */
  private static final double LOG_PERIOD = 1.0;

  public ArmTeleop(Robot robot) {
    // faceDown on the driver controller is A. Hold it to move the arm to 15 degrees. Releas and
    // will stay there
    robot.driverController.faceDown().whileTrue(robot.arm.holdAtAngle(Degrees.of(15)));

    // Bumpers adjust kP live and echo the value read back off the Talon. 
    // These are independent of any mechanism, so they can be pressed while moving the arm
    robot.driverController.rightBumper().onTrue(robot.arm.increaseKp());
    robot.driverController.leftBumper().onTrue(robot.arm.decreaseKp());

    // Arm angle tracks the right stick angle:  full right is 0 degrees, straight up is 90.
    // Invalid stick angle is ignored, and arm just holds until there is valid input.
    var stickTarget = new Trigger(() -> armAngleFromStick(robot.driverController).isPresent());
    stickTarget.whileTrue(robot.arm.followAngle(() -> armAngleFromStick(robot.driverController)));

    // Raw stick direction beside the angle it resolves to, so a direction the arm refuses is
    // visibly distinct from one it never read.
    // Note the use of "OptionalDouble and Angle".  You can check .isPresent.   Provides a way to
    // know if the value is valid or not before trying to act on it.
    addPeriodic(
        () -> {
          OptionalDouble direction = stickDirection(robot.driverController);
          Optional<Angle> target = armAngleFromStick(robot.driverController);
          System.out.printf(
              "Stick %s -> arm %s%n",
              direction.isPresent()
                  ? String.format("%6.1f deg", direction.getAsDouble())
                  : "  centered",
              target.map(angle -> String.format("%6.1f deg", angle.in(Degrees))).orElse("    hold"));
        },
        LOG_PERIOD);
  }

  /**
   * The direction the right stick points, in degrees counterclockwise from full right, or empty if
   * the stick is in the deadband
   *
   * Only Y is negated. The gamepad reports right as positive X, but up as negative Y, so
   * flipping Y  puts the stick into a conventional frame where atan2 measures counterclockwise
   * from "full right" which is the arms 0 angle
   * 
   * @return the angle at which the stick is pointing.  Empty if within Deadband
   */
  private static OptionalDouble stickDirection(CommandGamepad controller) {
    double right = controller.getRightX();
    double up = -controller.getRightY();

    if (Math.hypot(right, up) < DriverConstants.STICK_ANGLE_DEADBAND) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(Math.toDegrees(Math.atan2(up, right)));
  }

  /**
   * Maps the right stick onto the arm's travel, mirroring the arm in space: full right is 0 degrees
   * (arm horizontal), straight up is 90, and swinging further left reaches the 120-degree limit at
   * the upper left.
   *
   * @return the angle to command the arm to move to:
   *         empty if the stick within deadband, 
   *         empty if the angle is outside the soft stops of the arm travel. 
   * 
   *    */
  private static Optional<Angle> armAngleFromStick(CommandGamepad controller) {
    OptionalDouble direction = stickDirection(controller);
    if (direction.isEmpty()) {
      return Optional.empty();
    }

    double degrees = direction.getAsDouble();
    if (degrees < ArmConstants.MIN_ANGLE.in(Degrees)
        || degrees > ArmConstants.MAX_ANGLE.in(Degrees)) {
      return Optional.empty();
    }
    return Optional.of(Degrees.of(degrees));
  }
}
