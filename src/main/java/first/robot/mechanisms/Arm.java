// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

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
import org.wpilib.system.SystemServer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Time;

/**
 * A single-motor arm driven by a Kraken on Motion Magic position control, using
 * the motor's onboard
 * rotor encoder for feedback.
 *
 * {@code kG} can be nudged live with {@link #increaseKg()} /
 * {@link #decreaseKg()} rather than
 * redeploying for each value. Those change only the running config on the Talon
 * -- once a value
 * works, copy it back into {@link ArmConstants} or the next boot reverts it.
 * 
 * Notice the use of Javadoc tags. this will make the tooltips in your code work
 * when you hover
 * over somethnig.
 */
public class Arm implements Mechanism {
  private final TalonFX shoulder = new TalonFX(ArmConstants.SHOULDER_MOTOR_CAN_ID, new CANBus(CANPort.CAN_S0));
  private final TalonFX elbow = new TalonFX(ArmConstants.ELBOW_MOTOR_CAN_ID, new CANBus(CANPort.CAN_S1)); // change later

  /**
   * Reused rather than reallocated each loop, per Phoenix 6's control-request
   * convention.
   */
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

  /**
   * The live slot-0 gains, held as a field so kG can be re-applied while tuning.
   */
  private final Slot0Configs shoulderSlot0 = new Slot0Configs();
  private final Slot0Configs elbowSlot0 = new Slot0Configs();

  public Arm() {

    // Arm constructor. In this part of the code we are writing the motor
    // configuration to the motor
    // motors have multiple configuration slots, and we are using slot 0.
    shoulderSlot0.GravityType = GravityTypeValue.Arm_Cosine;
    shoulderSlot0.kS = ArmConstants.kS;
    shoulderSlot0.kV = ArmConstants.kV;
    shoulderSlot0.kA = ArmConstants.kA;
    shoulderSlot0.kG = ArmConstants.kG;
    shoulderSlot0.kP = ArmConstants.kP;
    shoulderSlot0.kI = ArmConstants.kI;
    shoulderSlot0.kD = ArmConstants.kD;

    // change later
    elbowSlot0.GravityType = GravityTypeValue.Arm_Cosine;
    elbowSlot0.kS = ArmConstants.kS;
    elbowSlot0.kV = ArmConstants.kV;
    elbowSlot0.kA = ArmConstants.kA;
    elbowSlot0.kG = ArmConstants.kG;
    elbowSlot0.kP = ArmConstants.kP;
    elbowSlot0.kI = ArmConstants.kI;
    elbowSlot0.kD = ArmConstants.kD;

    var shoulderConfig = new TalonFXConfiguration();
    shoulderConfig.Slot0 = shoulderSlot0;

    var elbowConfig = new TalonFXConfiguration();
    elbowConfig.Slot0 = elbowSlot0;

    // shoulder config

    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // With this set, every position and velocity on the Talon is in arm rotations
    // rather than
    // motor rotations, so the gains and setpoints below are all in mechanism units.
    shoulderConfig.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_GEAR_RATIO;

    shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY_RPS;
    shoulderConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.ACCELERATION_RPS2;

    shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.SHOULDER_MAX_ANGLE.in(Rotations);
    shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.SHOULDER_MIN_ANGLE.in(Rotations);

    shoulderConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT_AMPS;
    shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // apply() reports failure by return value rather than throwing, so an
    // unreachable or
    // misidentified device otherwise fails silently and leaves the Talon on its old
    // gains.
    report("apply config", shoulder.getConfigurator().apply(shoulderConfig), ArmConstants.SHOULDER_MOTOR_CAN_ID);

    // The rotor encoder is relative, so seed it with where the arm is sitting at
    // boot.
    report("seed position", shoulder.getConfigurator().setPosition(ArmConstants.SHOULDER_STARTING_ANGLE), ArmConstants.SHOULDER_MOTOR_CAN_ID);

    System.out.printf(
        "Arm: TalonFX %d ready, kG = %.3f V, kP = %.1f (read back from device)%n",
        ArmConstants.SHOULDER_MOTOR_CAN_ID, readBackKg(), shoulderSlot0.kP);

    // elbow config




    elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // With this set, every position and velocity on the Talon is in arm rotations
    // rather than
    // motor rotations, so the gains and setpoints below are all in mechanism units.
    elbowConfig.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_GEAR_RATIO;

    elbowConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY_RPS;
    elbowConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.ACCELERATION_RPS2;

    elbowConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elbowConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.ELBOW_MAX_ANGLE.in(Rotations);
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ELBOW_MIN_ANGLE.in(Rotations);

    elbowConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT_AMPS;
    elbowConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // apply() reports failure by return value rather than throwing, so an
    // unreachable or
    // misidentified device otherwise fails silently and leaves the Talon on its old
    // gains.
    report("apply config", elbow.getConfigurator().apply(elbowConfig), ArmConstants.ELBOW_MOTOR_CAN_ID);

    // The rotor encoder is relative, so seed it with where the arm is sitting at
    // boot.
    report("seed position", elbow.getConfigurator().setPosition(ArmConstants.ELBOW_STARTING_ANGLE), ArmConstants.ELBOW_MOTOR_CAN_ID);

    System.out.printf(
        "Arm: TalonFX %d ready, kG = %.3f V, kP = %.1f (read back from device)%n",
        ArmConstants.ELBOW_MOTOR_CAN_ID, readBackKg(), elbowSlot0.kP);

  }

  /**
   * Re-reads slot 0 off the Talon into {@link #shoulderSlot0} and returns the kG
   * the device actually holds.
   * Printing this rather than the value we sent means the console cannot claim a
   * gain that never
   * landed -- the exact failure mode where Phoenix Tuner shows 0 while the code
   * believes otherwise.
   */
  // come back to this
  private double readBackKg() {
    report("read back slot 0", shoulder.getConfigurator().refresh(shoulderSlot0), ArmConstants.SHOULDER_MOTOR_CAN_ID);
    report("read back slot 0", elbow.getConfigurator().refresh(elbowSlot0), ArmConstants.ELBOW_MOTOR_CAN_ID);
    return shoulderSlot0.kG;
  }

  private static void report(String what, StatusCode status, int canID) {
    if (status.isOK()) {
      return;
    }
    DriverStationErrors.reportError(
        "Arm: " + what + " failed on TalonFX " + canID + " -- " + status,
        false);
  }

  /** The arm's current angle, with 0 horizontal and positive up. */
  public Angle getShoulderAngle() {
    return shoulder.getPosition().getValue();
  }
  public Angle getElbowAngle() {
    return elbow.getPosition().getValue();
  }
  /**
   * The gain currently loaded on the Talon, which diverges from constants while
   * live-tuning.
   */
  public double getShoulderKg() {
    return shoulderSlot0.kG;
  }

  public double getElbowKg() {
    return elbowSlot0.kG;
  }

  /**
   * Whether the arm has arrived at {@code target}, within
   * {@link ArmConstants#TOLERANCE}.
   */
  public boolean shoulderIsAtAngle(Angle target) {
    return getShoulderAngle().minus(target).abs(Rotations) <= ArmConstants.TOLERANCE.in(Rotations);
  }

  public boolean elbowIsAtAngle(Angle target) {
    return getElbowAngle().minus(target).abs(Rotations) <= ArmConstants.TOLERANCE.in(Rotations);
  }

  /**
   * Drives the arm to {@code target} and holds it there indefinitely, going
   * neutral once
   * interrupted. Never finishes on its own, so bind it to a trigger rather than
   * placing it in a
   * sequence -- a sequence waiting on this command would never advance.
   *
   * @param target the angle to hold, 0 horizontal and positive up.
   * @return the command.
   */
  public Command shoulderHoldAtAngle(Angle target) {
    return run(coroutine -> {
      shoulder.setControl(positionRequest.withPosition(target));
      coroutine.park();
    })
        .whenCanceled(shoulder::stopMotor)
        .named("Shoulder hold at " + target.in(Degrees) + " deg");
  }

   public Command elbowHoldAtAngle(Angle target) {
    return run(coroutine -> {
      elbow.setControl(positionRequest.withPosition(target));
      coroutine.park();
    })
        .whenCanceled(elbow::stopMotor)
        .named("Elbow hold at " + target.in(Degrees) + " deg");
  }

  /**
   * Drives the arm to {@code target} and finishes once it arrives, within
   * {@link ArmConstants#TOLERANCE}. Use this inside a sequence that should wait
   * on the arm before
   * moving on; use {@link #shoulderHoldAtAngle(Angle)} to keep holding instead.
   *
   * @param target the angle to reach, 0 horizontal and positive up.
   * @return the command.
   */
  public Command shoulderGoToAngle(Angle target) {
    return run(coroutine -> {
      shoulder.setControl(positionRequest.withPosition(target));
      coroutine.waitUntil(() -> shoulderIsAtAngle(target));
    })
        .whenCanceled(shoulder::stopMotor)
        .named("Shoulder to " + target.in(Degrees) + " deg");
  }

  public Command elbowGoToAngle(Angle target) {
    return run(coroutine -> {
      elbow.setControl(positionRequest.withPosition(target));
      coroutine.waitUntil(() -> elbowIsAtAngle(target));
    })
        .whenCanceled(elbow::stopMotor)
        .named("Elbow to " + target.in(Degrees) + " deg");
  }
  /**
   * Holds the arm wherever it currently sits. Suitable as the default command.
   */
  public Command hold() {
    return run(coroutine -> {
      shoulder.setControl(positionRequest.withPosition(getShoulderAngle()));
      elbow.setControl(positionRequest.withPosition(getElbowAngle()));
      coroutine.park();
    })
        .whenCanceled(() -> {
          shoulder.stopMotor();
          elbow.stopMotor();
        })
        .named("Arm Hold");
  }

  /**
   * Continuously drives the arm to whatever angle {@code targetSupplier} reports,
   * re-commanding
   * every loop so the setpoint can move under the arm. An empty result leaves the
   * previous setpoint
   * standing rather than going neutral, so a momentary dropout never drops the
   * arm.
   *
   * @param targetSupplier the angle to chase, 0 horizontal and positive up.
   * @return the command.
   */
  public Command shoulderFollowAngle(Supplier<Optional<Angle>> targetSupplier) {
    return run(coroutine -> {
      while (true) {
        targetSupplier
            .get()
            .ifPresent(target -> shoulder.setControl(positionRequest.withPosition(target)));
        coroutine.yield();
      }
    })
        .whenCanceled(shoulder::stopMotor)
        .named("Shoulder Follow Angle");
  }

   public Command elbowFollowAngle(Supplier<Optional<Angle>> targetSupplier) {
    return run(coroutine -> {
      while (true) {
        targetSupplier
            .get()
            .ifPresent(target -> elbow.setControl(positionRequest.withPosition(target)));
        coroutine.yield();
      }
    })
        .whenCanceled(elbow::stopMotor)
        .named("Elbow Follow Angle");
  }

  public Command homingArm() {
        double initialCurrent = shoulder.getStatorCurrent().getValueAsDouble();
        double spikeChange = 1.0;
        System.out.println("method ran");
        return run(coroutine -> {
          System.out.println("method ran except other");
            shoulder.setVoltage(ArmConstants.HOMING_VOLTAGE);
            
            coroutine.waitUntil(() -> isStalled(initialCurrent, spikeChange), Time.ofBaseUnits(5, Seconds));
            System.out.println("STALLED");
            shoulder.setPosition(0.0);
          
            shoulder.setVoltage(0.0);
            coroutine.park();
        })
        .named("Zero Arm");
    }

    public boolean isStalled(double initialCurrent, double spikeChange) {
      double error = 0.3;
      boolean isStalled1 = shoulder.getStatorCurrent().getValueAsDouble() - initialCurrent > spikeChange
          && Math.abs(shoulder.getVelocity().getValueAsDouble()) < error;
      System.out.println("Stalled: " + (shoulder.getStatorCurrent().getValueAsDouble() - initialCurrent > spikeChange));
      System.out.println("Stopped: " + (Math.abs(shoulder.getVelocity().getValueAsDouble()) < error));
      System.out.println("Stator:" + shoulder.getStatorCurrent().getValueAsDouble());
      System.out.println("Velocity:" + shoulder.getVelocity().getValueAsDouble());
      System.out.println("initialCurrent: " + initialCurrent);
      System.out.println("spike: " + (shoulder.getStatorCurrent().getValueAsDouble() - initialCurrent));

      return isStalled1;
    }

  /** Cuts output and lets the arm fall to its brake-mode rest. */
  public Command stop() {
    return run(coroutine -> {
      shoulder.stopMotor();
      elbow.stopMotor();
      coroutine.park();
    })
        .named("Arm Stop");
  }

  /**
   * Raises the live kG by one step. Claims no mechanism, so it will not interrupt
   * a hold.
   */
  // make methods for elbowSlot0 as well if deemed necessary during testing
  public Command increaseKg() {
    return adjustGain("kG", () -> shoulderSlot0.kG, value -> shoulderSlot0.kG = value, ArmConstants.KG_STEP);
  }

  /** Lowers the live kG by one step, stopping at zero. */
  public Command decreaseKg() {
    return adjustGain("kG", () -> shoulderSlot0.kG, value -> shoulderSlot0.kG = value, -ArmConstants.KG_STEP);
  }

  /**
   * Raises the live kP by one step. Claims no mechanism, so it will not interrupt
   * a hold.
   */
  // make methods for elbowSlot0 as well if deemed necessary during testing
  public Command increaseKp() {
    return adjustGain("kP", () -> shoulderSlot0.kP, value -> shoulderSlot0.kP = value, ArmConstants.KP_STEP);
  }

  /** Lowers the live kP by one step, stopping at zero. */
  public Command decreaseKp() {
    return adjustGain("kP", () -> shoulderSlot0.kP, value -> shoulderSlot0.kP = value, -ArmConstants.KP_STEP);
  }

  /**
   * Steps one slot-0 gain and re-applies the slot, echoing the value the Talon
   * reports afterwards
   * rather than the one we sent. A rejected apply is rolled back so the field
   * never disagrees with
   * the device.
   */
  // write one for elbow when deemed necessary
  private Command adjustGain(String name, DoubleSupplier get, DoubleConsumer set, double delta) {
    return Command.noRequirements(
        coroutine -> {
          double previous = get.getAsDouble();
          set.accept(Math.max(0, previous + delta));

          var status = shoulder.getConfigurator().apply(shoulderSlot0);
          if (!status.isOK()) {
            set.accept(previous);
            DriverStationErrors.reportError(
                "Arm: " + name + " change rejected -- " + status, false);
            return;
          }

          // refresh() overwrites slot0 with what the device actually holds.
          report("read back slot 0", shoulder.getConfigurator().refresh(shoulderSlot0), ArmConstants.SHOULDER_MOTOR_CAN_ID);
          System.out.printf("Arm %s: %.3f -> %.3f%n", name, previous, get.getAsDouble());
        })
        .named("Arm " + name + (delta > 0 ? " up" : " down"));
  }
}
