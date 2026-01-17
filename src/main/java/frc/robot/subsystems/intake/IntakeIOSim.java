package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO{
    private DCMotorSim rollerSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, IntakeConstants.ROLLER_MOTOR_REDUCTION),
                    DCMotor.getKrakenX44Foc(1));

    private DCMotorSim deploySim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, IntakeConstants.DEPLOYMENT_MOTOR_REDUCTION),
                    DCMotor.getKrakenX44Foc(1));

    private double rollerAppliedVolts = 0.0;
    private double deployAppliedVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerSim.setInputVoltage(rollerAppliedVolts);
        rollerSim.update(0.02);

        deploySim.setInputVoltage(deployAppliedVolts);
        deploySim.update(0.02);

        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.deployAppliedVolts = deployAppliedVolts;
    }

    @Override
    public void setRollerMotorVoltage(double volts) {
        rollerAppliedVolts = MathUtil.clamp(volts, -12.0, -12.0);
    }

    @Override
    public void setDeploymentMotorVoltage(double volts) {
        deployAppliedVolts = MathUtil.clamp(volts, -12.0, -12.0);
    }
}
