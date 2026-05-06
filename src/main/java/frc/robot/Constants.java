// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.AngularVelocity;
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
    public static final AngularVelocity defaultOmega = KickerConstants.defaultOmega.div(3);

    public static final int spindexerMotorCANID = 50;
    public static final double spindexerGearing = 5d;
    public static final double spindexerConversionFactor = 1 / spindexerGearing;

    // Real Constants
    public static final double kPReal = 0.002, kIReal = 0.0, kDReal = 0.1, kVReal = 0.009;

    // Simulation Specific Constants
    public static final double kPSim = 0.002, kISim = 0.0, kDSim = 0.1, kVSim = 0.009;

    /** Roughly taken using onshape's "mass tool" */
    public static final double spindexerMOI = 0.009602574;
  }

  public static class KickerConstants {
    // Independant Constants
    public static final AngularVelocity defaultOmega = RPM.of(5500d); // 3 in wheels

    public static final double kickerGearing = 2d;
    public static final double encoderConversionFactor = 1.0d / kickerGearing;

    // Real Constants
    public static final double kPReal = 0.2d,
        kIReal = 0d,
        kDReal = 0d,
        kSReal = 0d,
        kVReal = 0.1224d;

    // Simulation Specific Constants
    public static final double kPSim = 0.2d, kISim = 0d, kDSim = 0d, kSSim = 0d, kVSim = 0.1224d;

    /** Roughly taken using onshape's "mass tool" */
    public static final double kickerMOI = 0.0089466111;
  }
}
