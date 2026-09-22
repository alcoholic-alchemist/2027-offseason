package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LoggedMechanism2d pivotMechanism = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d pivotMechanismRoot =
      pivotMechanism.getRoot("intakePivotArm", 1.0, 0.1);
  private final LoggedMechanismLigament2d pivotArmVisualization =
      pivotMechanismRoot.append(
          new LoggedMechanismLigament2d(
              "intakePivotArm",
              Constants.IntakeConstants.armLength,
              Constants.IntakeConstants.startingTheta));

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void raiseIntake() {
    io.setPivotAngle(Constants.IntakeConstants.retractedTheta);
  }

  public void lowerIntake() {
    io.setPivotAngle(Constants.IntakeConstants.deployedTheta);
  }

  public void spinIntakeForwards() {
    io.setSpinSpeed(Constants.IntakeConstants.spinOmega);
  }

  public void spinIntakeReverse() {
    io.setSpinSpeed(Constants.IntakeConstants.spinOmega.unaryMinus());
  }

  public void stopIntakeSpinners() {
    io.setSpinSpeed(RPM.of(0));
  }

  public void zeroIntakeToDeployedPosition() {
    io.overrideIntakeEncoderPosition(Constants.IntakeConstants.deployedTheta);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    pivotArmVisualization.setAngle(inputs.pivotTheta);

    Logger.processInputs(getName(), inputs);
  }
}
