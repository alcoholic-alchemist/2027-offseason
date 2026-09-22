package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.util.sim.SimPowerUtil;

public class IntakeIOSim implements IntakeIO {
  private final SparkFlex pivot;
  private final DCMotor pivotDCMotor;
  private final SparkFlexSim pivotSim;
  private final SingleJointedArmSim pivotArm;

  private final SparkFlex spin;
  private final DCMotor spinDCMotor;
  private final SparkFlexSim spinSim;
  private final FlywheelSim spinFlywheel;

  private Angle pivotThetaSetpoint = Radians.of(0);
  private AngularVelocity spinOmegaSetpoint = RPM.of(0);

  public IntakeIOSim() {
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
                        Constants.IntakeConstants.kPSimPivot,
                        Constants.IntakeConstants.kISimPivot,
                        Constants.IntakeConstants.kDSimPivot)
                    .apply(
                        new FeedForwardConfig()
                            .kV(Constants.IntakeConstants.kVSimPivot)
                            .kCos(Constants.IntakeConstants.kCosSimPivot))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotDCMotor = DCMotor.getNeoVortex(1);
    pivotSim = new SparkFlexSim(pivot, pivotDCMotor);
    pivotArm =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                pivotDCMotor,
                Constants.IntakeConstants.intakePivotMOI,
                Constants.IntakeConstants.intakePivotGearing),
            pivotDCMotor,
            Constants.IntakeConstants.intakePivotGearing,
            Constants.IntakeConstants.armLength.in(Meters),
            Constants.IntakeConstants.pivotMinTheta.in(Radians),
            Constants.IntakeConstants.pivotMaxTheta.in(Radians),
            true,
            Constants.IntakeConstants.startingTheta.in(Radians));

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
                        Constants.IntakeConstants.kPSimSpin,
                        Constants.IntakeConstants.kISimSpin,
                        Constants.IntakeConstants.kDSimSpin)
                    .apply(new FeedForwardConfig().kV(Constants.IntakeConstants.kVSimSpin))),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    spinDCMotor = DCMotor.getNeoVortex(1);
    spinSim = new SparkFlexSim(spin, spinDCMotor);
    spinFlywheel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                spinDCMotor,
                Constants.IntakeConstants.intakeSpinnerMOI,
                Constants.IntakeConstants.intakeSpinGearing),
            spinDCMotor);
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
    pivotArm.setState(theta.in(Radians), pivotArm.getVelocityRadPerSec());
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    spinFlywheel.setInputVoltage(spin.getAppliedOutput() * RoboRioSim.getVInVoltage());
    spinFlywheel.update(0.02);
    spinSim.iterate(spinFlywheel.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    inputs.spinOmega = spinFlywheel.getAngularVelocity();
    inputs.spinOmegaSetpoint = spinOmegaSetpoint;
    inputs.spinCurrent = Amps.of(spin.getOutputCurrent());
    inputs.spinVoltage = Volts.of(spin.getAppliedOutput() * spin.getBusVoltage());

    SimPowerUtil.addSubsystemCurrentToCurrentSum(inputs.spinCurrent);

    pivotArm.setInputVoltage(pivot.getAppliedOutput() * RoboRioSim.getVInVoltage());
    pivotArm.update(0.02);
    pivotSim.iterate(
        RadiansPerSecond.of(pivotArm.getVelocityRadPerSec()).in(RPM),
        RoboRioSim.getVInVoltage(),
        0.02);

    inputs.pivotTheta = Radians.of(pivotArm.getAngleRads());
    inputs.pivotThetaSetpoint = pivotThetaSetpoint;
    inputs.pivotOmega = RadiansPerSecond.of(pivotArm.getVelocityRadPerSec());
    inputs.pivotCurrent = Amps.of(spin.getOutputCurrent());
    inputs.pivotVoltage = Volts.of(spin.getAppliedOutput() * spin.getBusVoltage());

    SimPowerUtil.addSubsystemCurrentToCurrentSum(inputs.pivotCurrent);
  }
}
