// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final CANBus rioCANBus = CANBus.roboRIO();
  public static final CANBus shooterCANBus = new CANBus("shooter");

  public static final int driverControllerID = 0, operatorControllerID = 1;

  public static class SpindexerConstants {
    // Independant Constants

    /** Default omega velocity is kicker omega / 3 as spindexer has 6" diameter. */
    // It seems like the spindexer's maximum speed, at least in sim, is close to 1380 RPM.
    public static final AngularVelocity defaultOmega = KickerConstants.defaultOmega.div(3);

    public static final int spindexerMotorCANID = 50;
    public static final double spindexerGearing = 5d;
    public static final double spindexerConversionFactor = 1 / spindexerGearing;

    // Real Constants
    public static final double kPReal = 0.002, kIReal = 0.0, kDReal = 0.1, kVReal = 0.009;

    // Simulation Specific Constants
    public static final double kPSim = 0.004, kISim = 0.0, kDSim = 0.0, kVSim = 0.009;

    /** Roughly taken using onshape's "mass tool" */
    public static final double spindexerMOI = 0.009602574;
  }

  public static class KickerConstants {
    // Independant Constants
    public static final AngularVelocity defaultOmega = RPM.of(5500d); // 3 in wheels
    public static final int kickerMotorCANID = 55;

    public static final double kickerGearing = 2d;
    public static final double encoderConversionFactor = 1.0d / kickerGearing;

    // Real Constants
    public static final double kPReal = 0.2d, kIReal = 0d, kDReal = 0d, kVReal = 0.1224d;

    // Simulation Specific Constants
    public static final double kPSim = 0.2d, kISim = 0d, kDSim = 0d, kVSim = 0.1224d;

    /** Roughly taken using onshape's "mass tool" */
    public static final double kickerMOI = 0.0089466111;
  }

  public static class IntakeConstants {
    // Independant Constants
    public static final AngularVelocity spinOmega = RPM.of(2000);

    public static final Angle pivotMaxTheta = Degrees.of(131.76767);
    public static final Angle pivotMinTheta = Degrees.of(0);
    public static final Angle deployedTheta = pivotMinTheta;
    public static final Angle retractedTheta = Degrees.of(130);
    public static final Angle startingTheta = pivotMaxTheta;

    public static final int intakePivotMotorCANID = 52;
    public static final int intakeSpinMotorCANID = 51;

    public static final double intakePivotGearing = 26.84933149230769;
    public static final double intakePivotEncoderConversionFactor = 1 / intakePivotGearing;

    public static final double intakeSpinGearing = 5d;
    public static final double intakeSpinEncoderConversionFactor = 1 / intakeSpinGearing;

    public static final Distance armLength = Inches.of(11.25);

    // Real Constants
    public static final double kPRealPivot = 1.0,
        kIRealPivot = 0d,
        kDRealPivot = 0d,
        kVRealPivot = 0d,
        kCosRealPivot = 0.4;
    public static final double kPRealSpin = 0d, kIRealSpin = 0d, kDRealSpin = 0d, kVRealSpin = 3.2d;

    // Simulation Specific Constants
    public static final double kPSimPivot = 1.0,
        kISimPivot = 0d,
        kDSimPivot = 0d,
        kVSimPivot = 0d,
        kCosSimPivot = 0.4;
    public static final double kPSimSpin = 0d, kISimSpin = 0d, kDSimSpin = 0d, kVSimSpin = 3.2d;

    /** Roughly taken using onshape's "mass tool" */
    public static final double intakeSpinnerMOI = 0.112309246196442;

    /** Roughly taken using onshape's "mass tool" */
    public static final double intakePivotMOI = 0.0518;
  }
}
