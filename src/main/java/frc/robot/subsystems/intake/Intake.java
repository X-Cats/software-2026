package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Command runRollerMotor() {
        return runEnd(
                ()-> {
                    io.setRollerMotorVoltage(IntakeConstants.ROLLER_MOTOR_VOLTAGE);
                },
                ()-> {
                    io.setRollerMotorVoltage(0.0);
                });
    }

    public Command stow() {
        return runEnd(
                ()-> {
                    io.setDeploymentMotorVoltage(IntakeConstants.DEPLOYMENT_MOTOR_VOLTAGE);
                },
                ()-> {
                    io.setDeploymentMotorVoltage(0.0);
                });
    }

    public Command deploy() {
        return runEnd(
                ()-> {
                    io.setDeploymentMotorVoltage(-IntakeConstants.DEPLOYMENT_MOTOR_VOLTAGE);
                },
                ()-> {
                    io.setDeploymentMotorVoltage(0.0);
                });
    }
}
