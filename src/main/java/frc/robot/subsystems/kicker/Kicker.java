package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void runForwards() {
    io.setSpeed(Constants.KickerConstants.defaultOmega);
  }

  public void runReverse() {
    io.setSpeed(Constants.KickerConstants.defaultOmega.unaryMinus());
  }

  public void stop() {
    io.setSpeed(RPM.of(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
