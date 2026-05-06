package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public AngularVelocity omega = RPM.of(0);
    public AngularVelocity omegaSetpoint = RPM.of(0);

    public Current current = Amps.of(0);
    public Voltage voltage = Volts.of(0);
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setSpeed(AngularVelocity omega) {}
}
