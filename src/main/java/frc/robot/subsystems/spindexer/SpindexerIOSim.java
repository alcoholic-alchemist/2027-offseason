package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

/** Simulation implementation of {@link}SpindexerIO */
public class SpindexerIOSim implements SpindexerIO {
  private final DCMotor dcMotor;
  private final SparkFlex motor;
  private final SparkFlexSim motorSim;
  private final FlywheelSim spindexerFlywheel;

  private AngularVelocity omegaSetpoint = RPM.of(0);

  public SpindexerIOSim() {
    dcMotor = DCMotor.getNeoVortex(1);

    motor = new SparkFlex(Constants.SpindexerConstants.spindexerMotorCANID, MotorType.kBrushless);
    motor.configure(
        new SparkFlexConfig()
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(
                        Constants.SpindexerConstants.spindexerConversionFactor)
                    .velocityConversionFactor(
                        Constants.SpindexerConstants.spindexerConversionFactor))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.SpindexerConstants.kPSim,
                        Constants.SpindexerConstants.kISim,
                        Constants.SpindexerConstants.kDSim)
                    .apply(new FeedForwardConfig().kV(Constants.SpindexerConstants.kVSim))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    motorSim = new SparkFlexSim(motor, dcMotor);

    spindexerFlywheel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                dcMotor,
                Constants.SpindexerConstants.spindexerMOI,
                Constants.SpindexerConstants.spindexerGearing),
            dcMotor);
  }

  @Override
  public void setSpeed(AngularVelocity omega) {
    motor.getClosedLoopController().setSetpoint(omega.in(RPM), ControlType.kVelocity);
    omegaSetpoint = omega;
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    spindexerFlywheel.setInput(motor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    spindexerFlywheel.update(0.02);
    motorSim.iterate(spindexerFlywheel.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
    BatterySim.calculateDefaultBatteryLoadedVoltage(spindexerFlywheel.getCurrentDrawAmps());

    inputs.omega = RPM.of(motor.getEncoder().getVelocity());
    inputs.omegaSetpoint = omegaSetpoint;
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.voltage = Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
  }
}
