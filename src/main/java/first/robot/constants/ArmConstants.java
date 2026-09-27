// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.constants;

import static org.wpilib.units.Units.Degrees;

import first.robot.mechanisms.Arm;
import org.wpilib.units.measure.Angle;

/**
 * Constants for the {@link Arm}.
 *
 * <p>Angle convention: 0 degrees is the arm held horizontal, positive is up. Motion Magic's
 * {@code Arm_Cosine} gravity feedforward assumes this convention -- it applies the full {@code kG}
 * at 0 degrees and none at +/-90, so the zero must be horizontal for gravity compensation to track
 * the real load.
 */
public final class ArmConstants {
  private ArmConstants() {}

  public static final int MOTOR_CAN_ID = 32;

  /**
   * Motor rotations per arm rotation, including the gearbox and any chain or belt stage after it.
   * Every angle below is interpreted through this number, so a wrong value moves the arm to the
   * wrong place.
   */
  public static final double GEAR_RATIO = 50.0;

  /** Travel limits, enforced on the Talon as soft limits. */
  public static final Angle MIN_ANGLE = Degrees.of(0);

  public static final Angle MAX_ANGLE = Degrees.of(120);

  /**
   * Where the arm physically rests when the robot boots -- horizontal, which is this mechanism's
   * zero. The Kraken's onboard rotor encoder is relative, so it reads zero at power-on wherever the
   * arm happens to be sitting; the mechanism seeds the encoder with this value at construction. The
   * arm must actually be at rest at boot or every setpoint is offset by the difference.
   */
  public static final Angle STARTING_ANGLE = Degrees.of(0);

  /**
   * Motion Magic profile limits, in arm rotations per second and per second squared. Set slow for
   * bring-up: 0.1 rps is 36 deg/s, so full travel takes a little over three seconds. Walk these up
   * once the gains hold position cleanly.
   */
  public static final double CRUISE_VELOCITY_RPS = 0.1;

  public static final double ACCELERATION_RPS2 = 0.2;

  // Slot 0 gains. All zero until tuned on the real arm -- see the tuning note in Arm.
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.33;
  public static final double kP = 50.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** How much a bumper press moves kG while live-tuning. */
  public static final double KG_STEP = 0.01;

  /** How much a bumper press moves kP while live-tuning. */
  public static final double KP_STEP = 10.0;

  public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;

  /** How close to the setpoint counts as arrived. */
  public static final Angle TOLERANCE = Degrees.of(2);
}
