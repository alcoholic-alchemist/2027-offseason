package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex pivot;
  private final SparkFlex spin;

  private Angle pivotThetaSetpoint = Radians.of(0);
  private AngularVelocity spinOmegaSetpoint = RPM.of(0);

  public IntakeIOReal() {
    pivot = new SparkFlex(Constants.IntakeConstants.intakePivotMotorCANID, MotorType.kBrushless);
    pivot.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(60)
            .inverted(false)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(
                        Constants.IntakeConstants.intakePivotEncoderConversionFactor)
                    .velocityConversionFactor(
                        Constants.IntakeConstants.intakePivotEncoderConversionFactor))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.kPRealPivot,
                        Constants.IntakeConstants.kIRealPivot,
                        Constants.IntakeConstants.kDRealPivot)
                    .apply(
                        new FeedForwardConfig()
                            .kV(Constants.IntakeConstants.kVRealPivot)
                            .kCos(Constants.IntakeConstants.kCosRealPivot))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    spin = new SparkFlex(Constants.IntakeConstants.intakeSpinMotorCANID, MotorType.kBrushless);
    spin.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(60)
            .inverted(true)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(
                        Constants.IntakeConstants.intakeSpinEncoderConversionFactor)
                    .velocityConversionFactor(
                        Constants.IntakeConstants.intakeSpinEncoderConversionFactor))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.kPRealSpin,
                        Constants.IntakeConstants.kIRealSpin,
                        Constants.IntakeConstants.kDRealSpin)
                    .apply(new FeedForwardConfig().kV(Constants.IntakeConstants.kVRealSpin))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setSpinSpeed(AngularVelocity omega) {
    spin.getClosedLoopController().setSetpoint(omega.in(RPM), ControlType.kVelocity);
    spinOmegaSetpoint = omega;
  }

  @Override
  public void setPivotAngle(Angle theta) {
    pivot.getClosedLoopController().setSetpoint(theta.in(Rotations), ControlType.kPosition);
    pivotThetaSetpoint = theta;
  }

  @Override
  public void overrideIntakeEncoderPosition(Angle theta) {
    pivot.getEncoder().setPosition(theta.in(Rotations));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotTheta = Rotations.of(pivot.getEncoder().getPosition());
    inputs.pivotThetaSetpoint = pivotThetaSetpoint;
    inputs.pivotOmega = RPM.of(pivot.getEncoder().getVelocity());
    inputs.pivotCurrent = Amps.of(pivot.getOutputCurrent());
    inputs.pivotVoltage = Volts.of(pivot.getBusVoltage() * pivot.getAppliedOutput());

    inputs.spinOmega = RPM.of(spin.getEncoder().getVelocity());
    inputs.spinOmegaSetpoint = spinOmegaSetpoint;
    inputs.spinCurrent = Amps.of(spin.getOutputCurrent());
    inputs.spinVoltage = Volts.of(pivot.getBusVoltage() * pivot.getAppliedOutput());
  }
}
