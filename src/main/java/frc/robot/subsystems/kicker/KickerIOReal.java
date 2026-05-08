package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class KickerIOReal implements KickerIO {
  private final TalonFX motor;
  private AngularVelocity omegaSetpoint = RPM.of(0);

  public KickerIOReal() {
    motor = new TalonFX(Constants.KickerConstants.kickerMotorCANID, Constants.shooterCANBus);
    motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.KickerConstants.kPReal)
                .withKI(Constants.KickerConstants.kIReal)
                .withKD(Constants.KickerConstants.kDReal)
                .withKV(Constants.KickerConstants.kVReal));
    motor
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.KickerConstants.encoderConversionFactor));
  }

  @Override
  public void setSpeed(AngularVelocity omega) {
    motor.setControl(new VelocityVoltage(omega).withSlot(0));
    omegaSetpoint = omega;
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.omega = motor.getVelocity().getValue();
    inputs.omegaSetpoint = omegaSetpoint;
    inputs.current = motor.getSupplyCurrent().getValue();
    inputs.voltage = motor.getMotorVoltage().getValue();
  }
}
