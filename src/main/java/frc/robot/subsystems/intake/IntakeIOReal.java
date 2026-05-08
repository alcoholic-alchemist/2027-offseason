package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex pivot;
  private final SparkFlex spin;

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
}
