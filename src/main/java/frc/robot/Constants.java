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
    public static final int spindexerMotorCANID = 50;

    public static final double spindexerGearing = 5d;
    public static final double spindexerConversionFactor = 1 / spindexerGearing;

    public static final double kP = 0.002, kI = 0.0, kD = 0.1, kV = 0.009;

    public static final double spindexerMOI = 0.0012; // TBD

    public static final AngularVelocity defaultOmega = KickerConstants.velocitySetpoint.div(3);
  }

  public static class KickerConstants {

    public static final AngularVelocity velocitySetpoint = RPM.of(5500d); // 3 in wheels
  }
}
