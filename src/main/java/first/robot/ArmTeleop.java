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

@Teleop
public class ArmTeleop extends PeriodicOpMode {
  /** How often the stick and arm angles are echoed to the console, in seconds. */
  private static final double LOG_PERIOD = 1.0;

  public ArmTeleop(Robot robot) {
    // Hold A to run the arm up to 15 degrees. Releasing hands it back to the default hold, which
    // keeps it wherever the move left it.
    robot.driverController.faceDown().whileTrue(robot.arm.goToAngle(Degrees.of(15)));

    // Bumpers step kP live and echo the value read back off the Talon. These claim no mechanism,
    // so they can be pressed mid-move -- the travel keeps running and picks up each new gain.
    robot.driverController.rightBumper().onTrue(robot.arm.increaseKp());
    robot.driverController.leftBumper().onTrue(robot.arm.decreaseKp());

    // Point the right stick where you want the arm: full right is 0 degrees, straight up is 90.
    // The trigger is true only while the stick names an angle the arm can actually reach, so
    // centering it or swinging outside the travel range simply drops back to the default hold.
    var stickTarget = new Trigger(() -> armAngleFromStick(robot.driverController).isPresent());
    stickTarget.whileTrue(robot.arm.followAngle(() -> armAngleFromStick(robot.driverController)));

    // Raw stick direction beside the angle it resolves to, so a direction the arm refuses is
    // visibly distinct from one it never read.
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
   * the stick is close enough to center that its direction is only noise.
   *
   * <p>Only Y is negated. The gamepad already reports right as positive X, but up as negative Y, so
   * flipping Y alone puts the stick into a conventional frame where atan2 measures counterclockwise
   * from "full right" -- which is exactly the arm's own zero.
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
   * @return the angle to command, or empty if the stick is centered or points outside the travel
   *     range, both of which mean "no command".
   */
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
