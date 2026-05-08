package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // Pivot
    public AngularVelocity pivotOmega = RPM.of(0);
    public Angle pivotTheta = Radians.of(0);
    public Angle pivotThetaSetpoint = Radians.of(0);
    public Current pivotCurrent = Amps.of(0);
    public Voltage pivotVoltage = Volts.of(0);

    // Spin
    public AngularVelocity spinOmega = RPM.of(0);
    public AngularVelocity spinOmegaSetpoint = RPM.of(0);
    public Current spinCurrent = Amps.of(0);
    public Voltage spinVoltage = Volts.of(0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setSpinSpeed(AngularVelocity omega) {}

  public default void setPivotAngle(Angle theta) {}
}
