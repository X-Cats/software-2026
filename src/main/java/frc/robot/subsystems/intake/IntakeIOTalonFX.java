package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO{
    private final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    private final StatusSignal<Voltage> rollerAppliedVolts = roller.getMotorVoltage();

    private final TalonFX deploy = new TalonFX(IntakeConstants.DEPLOYMENT_MOTOR_ID);
    private final StatusSignal<Voltage> deployAppliedVolts = deploy.getMotorVoltage();

    public IntakeIOTalonFX() {
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_MOTOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        roller.getConfigurator().apply(rollerConfig, 0.25);
//        tryUntilOk(5, ()->  roller.getConfigurator().apply(rollerConfig, 0.25));

        var deployConfig = new TalonFXConfiguration();
        deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOYMENT_MOTOR_CURRENT_LIMIT;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deploy.getConfigurator().apply(deployConfig, 0.25);
//        tryUntilOk(5, ()->  deploy.getConfigurator().apply(deployConfig, 0.25));
    }
}
