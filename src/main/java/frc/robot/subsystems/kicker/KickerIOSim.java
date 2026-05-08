package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.util.sim.SimPowerUtil;

public class KickerIOSim implements KickerIO {
  private final TalonFX motor;
  private final TalonFXSimState motorSimState;
  private final DCMotor krakenX60;
  private final FlywheelSim kickerFlywheel;

  private AngularVelocity omegaSetpoint = RPM.of(0);

  public KickerIOSim() {
    krakenX60 = DCMotor.getKrakenX60(1);

    motor = new TalonFX(Constants.KickerConstants.kickerMotorCANID, Constants.shooterCANBus);
    motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.KickerConstants.kPSim)
                .withKI(Constants.KickerConstants.kISim)
                .withKD(Constants.KickerConstants.kDSim)
                .withKV(Constants.KickerConstants.kVSim));
    motor
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.KickerConstants.encoderConversionFactor));

    motorSimState = new TalonFXSimState(motor);

    kickerFlywheel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                krakenX60,
                Constants.KickerConstants.kickerMOI,
                Constants.KickerConstants.kickerGearing),
            krakenX60);
  }

  @Override
  public void setSpeed(AngularVelocity omega) {
    motor.setControl(new VelocityVoltage(omega).withSlot(0));
    omegaSetpoint = omega;
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    motorSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    kickerFlywheel.setInputVoltage(motor.getMotorVoltage().getValue().in(Volts));
    kickerFlywheel.update(0.02);

    motorSimState.setRotorVelocity(kickerFlywheel.getAngularVelocity());

    inputs.omega = kickerFlywheel.getAngularVelocity();
    inputs.omegaSetpoint = omegaSetpoint;
    inputs.current = motor.getSupplyCurrent().getValue();
    inputs.voltage = motor.getMotorVoltage().getValue();

    SimPowerUtil.addSubsystemCurrentToCurrentSum(inputs.current);
  }
}
