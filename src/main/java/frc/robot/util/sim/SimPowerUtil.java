package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;

public class SimPowerUtil {
  private static Current currentSum = Amps.of(0);

  public static Voltage getBatteryLoadedVoltage() {
    return Volts.of(
        BatterySim.calculateDefaultBatteryLoadedVoltage(getSubsystemCurrentSum().in(Amps)));
  }

  public static Current getSubsystemCurrentSum() {
    return currentSum;
  }

  public static void addSubsystemCurrentToCurrentSum(Current current) {
    currentSum = currentSum.plus(current);
  }

  public static void resetSubsystemCurrentSum() {
    currentSum = Amps.of(0);
  }
}
