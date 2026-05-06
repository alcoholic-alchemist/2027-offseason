package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public void runForwards() {
    io.setSpeed(Constants.SpindexerConstants.defaultOmega);
  }

  public void runReverse() {
    io.setSpeed(Constants.SpindexerConstants.defaultOmega.unaryMinus());
  }

  public void stop() {
    io.setSpeed(RPM.of(0));
  }
}
