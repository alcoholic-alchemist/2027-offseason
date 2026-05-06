package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void runForwads() {
    io.setSpeed(Constants.KickerConstants.defaultOmega);
  }

  public void runReverse() {
    io.setSpeed(Constants.KickerConstants.defaultOmega.unaryMinus());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
