package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
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
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class SpindexerIOReal implements SpindexerIO {

  private final SparkFlex motor;

  private AngularVelocity omegaSetpoint = RPM.of(0);

  public SpindexerIOReal() {
    motor = new SparkFlex(Constants.SpindexerConstants.spindexerMotorCANID, MotorType.kBrushless);
    motor.configure(
        new SparkFlexConfig()
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(
                        Constants.SpindexerConstants.spindexerConversionFactor)
                    .velocityConversionFactor(
                        Constants.SpindexerConstants.spindexerConversionFactor)
                    .inverted(false))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.SpindexerConstants.kPReal,
                        Constants.SpindexerConstants.kIReal,
                        Constants.SpindexerConstants.kDReal)
                    .apply(new FeedForwardConfig().kV(Constants.SpindexerConstants.kVReal))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setSpeed(AngularVelocity omega) {
    motor.getClosedLoopController().setSetpoint(omega.in(RPM), ControlType.kVelocity);
    omegaSetpoint = omega;
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.omega = RPM.of(motor.getEncoder().getVelocity());
    inputs.omegaSetpoint = omegaSetpoint;
    inputs.voltage = Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    inputs.current = Amps.of(motor.getOutputCurrent());
  }
}
