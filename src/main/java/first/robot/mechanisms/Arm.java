// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.robot.constants.ArmConstants;
import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;

/**
 * A single-motor arm driven by a Kraken on Motion Magic position control, using the motor's onboard
 * rotor encoder for feedback.
 *
 * {@code kG} can be nudged live with {@link #increaseKg()} / {@link #decreaseKg()} rather than
 * redeploying for each value. Those change only the running config on the Talon -- once a value
 * works, copy it back into {@link ArmConstants} or the next boot reverts it.
 * 
 * Notice the use of Javadoc tags.  this will make the tooltips in your code work when you hover 
 * over somethnig.
 */
public class Arm implements Mechanism {
  private final TalonFX motor =
      new TalonFX(ArmConstants.MOTOR_CAN_ID, new CANBus(CANPort.CAN_S0));

  /** Reused rather than reallocated each loop, per Phoenix 6's control-request convention. */
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

  /** The live slot-0 gains, held as a field so kG can be re-applied while tuning. */
  private final Slot0Configs slot0 = new Slot0Configs();

  public Arm() {

    //Arm constructor.   In this part of the code we are writing the motor configuration to the motor
    //motors have multiple configuration slots, and we are using slot 0.
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = ArmConstants.kS;
    slot0.kV = ArmConstants.kV;
    slot0.kA = ArmConstants.kA;
    slot0.kG = ArmConstants.kG;
    slot0.kP = ArmConstants.kP;
    slot0.kI = ArmConstants.kI;
    slot0.kD = ArmConstants.kD;

    var config = new TalonFXConfiguration();
    config.Slot0 = slot0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // With this set, every position and velocity on the Talon is in arm rotations rather than
    // motor rotations, so the gains and setpoints below are all in mechanism units.
    config.Feedback.SensorToMechanismRatio = ArmConstants.GEAR_RATIO;

    config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY_RPS;
    config.MotionMagic.MotionMagicAcceleration = ArmConstants.ACCELERATION_RPS2;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.MAX_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.MIN_ANGLE.in(Rotations);

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // apply() reports failure by return value rather than throwing, so an unreachable or
    // misidentified device otherwise fails silently and leaves the Talon on its old gains.
    report("apply config", motor.getConfigurator().apply(config));

    // The rotor encoder is relative, so seed it with where the arm is sitting at boot.
    report("seed position", motor.getConfigurator().setPosition(ArmConstants.STARTING_ANGLE));

    System.out.printf(
        "Arm: TalonFX %d ready, kG = %.3f V, kP = %.1f (read back from device)%n",
        ArmConstants.MOTOR_CAN_ID, readBackKg(), slot0.kP);
  }

  /**
   * Re-reads slot 0 off the Talon into {@link #slot0} and returns the kG the device actually holds.
   * Printing this rather than the value we sent means the console cannot claim a gain that never
   * landed -- the exact failure mode where Phoenix Tuner shows 0 while the code believes otherwise.
   */
  private double readBackKg() {
    report("read back slot 0", motor.getConfigurator().refresh(slot0));
    return slot0.kG;
  }

  private static void report(String what, StatusCode status) {
    if (status.isOK()) {
      return;
    }
    DriverStationErrors.reportError(
        "Arm: " + what + " failed on TalonFX " + ArmConstants.MOTOR_CAN_ID + " -- " + status,
        false);
  }

  /** The arm's current angle, with 0 horizontal and positive up. */
  public Angle getAngle() {
    return motor.getPosition().getValue();
  }

  /** The gain currently loaded on the Talon, which diverges from constants while live-tuning. */
  public double getKg() {
    return slot0.kG;
  }

  /** Whether the arm has arrived at {@code target}, within {@link ArmConstants#TOLERANCE}. */
  public boolean isAtAngle(Angle target) {
    return getAngle().minus(target).abs(Rotations) <= ArmConstants.TOLERANCE.in(Rotations);
  }

  /**
   * Drives the arm to {@code target} and holds it there indefinitely, going neutral once
   * interrupted. Never finishes on its own, so bind it to a trigger rather than placing it in a
   * sequence -- a sequence waiting on this command would never advance.
   *
   * @param target the angle to hold, 0 horizontal and positive up.
   * @return the command.
   */
  public Command holdAtAngle(Angle target) {
    return run(coroutine -> {
          motor.setControl(positionRequest.withPosition(target));
          coroutine.park();
        })
        .whenCanceled(motor::stopMotor)
        .named("Arm hold at " + target.in(Degrees) + " deg");
  }

  /**
   * Drives the arm to {@code target} and finishes once it arrives, within
   * {@link ArmConstants#TOLERANCE}. Use this inside a sequence that should wait on the arm before
   * moving on; use {@link #holdAtAngle(Angle)} to keep holding instead.
   *
   * @param target the angle to reach, 0 horizontal and positive up.
   * @return the command.
   */
  public Command goToAngle(Angle target) {
    return run(coroutine -> {
          motor.setControl(positionRequest.withPosition(target));
          coroutine.waitUntil(() -> isAtAngle(target));
        })
        .whenCanceled(motor::stopMotor)
        .named("Arm to " + target.in(Degrees) + " deg");
  }

  /** Holds the arm wherever it currently sits. Suitable as the default command. */
  public Command hold() {
    return run(coroutine -> {
          motor.setControl(positionRequest.withPosition(getAngle()));
          coroutine.park();
        })
        .whenCanceled(motor::stopMotor)
        .named("Arm Hold");
  }

  /**
   * Continuously drives the arm to whatever angle {@code targetSupplier} reports, re-commanding
   * every loop so the setpoint can move under the arm. An empty result leaves the previous setpoint
   * standing rather than going neutral, so a momentary dropout never drops the arm.
   *
   * @param targetSupplier the angle to chase, 0 horizontal and positive up.
   * @return the command.
   */
  public Command followAngle(Supplier<Optional<Angle>> targetSupplier) {
    return run(coroutine -> {
          while (true) {
            targetSupplier
                .get()
                .ifPresent(target -> motor.setControl(positionRequest.withPosition(target)));
            coroutine.yield();
          }
        })
        .whenCanceled(motor::stopMotor)
        .named("Arm Follow Angle");
  }

  /** Cuts output and lets the arm fall to its brake-mode rest. */
  public Command stop() {
    return run(coroutine -> {
          motor.stopMotor();
          coroutine.park();
        })
        .named("Arm Stop");
  }

  /** Raises the live kG by one step. Claims no mechanism, so it will not interrupt a hold. */
  public Command increaseKg() {
    return adjustGain("kG", () -> slot0.kG, value -> slot0.kG = value, ArmConstants.KG_STEP);
  }

  /** Lowers the live kG by one step, stopping at zero. */
  public Command decreaseKg() {
    return adjustGain("kG", () -> slot0.kG, value -> slot0.kG = value, -ArmConstants.KG_STEP);
  }

  /** Raises the live kP by one step. Claims no mechanism, so it will not interrupt a hold. */
  public Command increaseKp() {
    return adjustGain("kP", () -> slot0.kP, value -> slot0.kP = value, ArmConstants.KP_STEP);
  }

  /** Lowers the live kP by one step, stopping at zero. */
  public Command decreaseKp() {
    return adjustGain("kP", () -> slot0.kP, value -> slot0.kP = value, -ArmConstants.KP_STEP);
  }

  /**
   * Steps one slot-0 gain and re-applies the slot, echoing the value the Talon reports afterwards
   * rather than the one we sent. A rejected apply is rolled back so the field never disagrees with
   * the device.
   */
  private Command adjustGain(String name, DoubleSupplier get, DoubleConsumer set, double delta) {
    return Command.noRequirements(
            coroutine -> {
              double previous = get.getAsDouble();
              set.accept(Math.max(0, previous + delta));

              var status = motor.getConfigurator().apply(slot0);
              if (!status.isOK()) {
                set.accept(previous);
                DriverStationErrors.reportError(
                    "Arm: " + name + " change rejected -- " + status, false);
                return;
              }

              // refresh() overwrites slot0 with what the device actually holds.
              report("read back slot 0", motor.getConfigurator().refresh(slot0));
              System.out.printf("Arm %s: %.3f -> %.3f%n", name, previous, get.getAsDouble());
            })
        .named("Arm " + name + (delta > 0 ? " up" : " down"));
  }
}
