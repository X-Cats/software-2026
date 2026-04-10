package frc.robot.subsystems.HoodKicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.util.LaunchCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class HoodKicker extends SubsystemBase {
  private final HoodKickerIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodKickerIO.HoodIOOutputs outputs = new HoodKickerIO.HoodIOOutputs();
  private final RobotStateMachine robotState;

  private static final LoggedTunableNumber goalPosition =
      new LoggedTunableNumber("Hood/Position", 500);
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Hood/kP", HoodKickerConstants.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Hood/kD", HoodKickerConstants.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Hood/kS", HoodKickerConstants.kS);

  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Hood/Homing/Volts", -2);
  private static final LoggedTunableNumber homingVelocityThreshold =
      new LoggedTunableNumber("Hood/Homing/VelocityThreshold", 0.05);

  private boolean hasBeenZeroed = false;

  public HoodKicker(HoodKickerIO io, RobotStateMachine rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    hasBeenZeroed = hasBeenZeroed || -0.5 < inputs.hoodPosition && inputs.hoodPosition < 0.5;
    Logger.processInputs("Hood", inputs);

    LaunchCalculator.getInstance().getParameters().hoodAngle();
    outputs.kP = kP.getAsDouble();
    outputs.kD = kD.getAsDouble();
    outputs.kS = kS.getAsDouble();

    if (this.hasBeenZeroed) {
      switch (robotState.getDesiredHoodState().getHoodState()) {
        case AIMING -> outputs.positionRad =
            LaunchCalculator.getInstance().getParameters().hoodAngle();
        case STOWED -> outputs.positionRad = 0;
        default -> {
          System.out.println(
              "Illegal Hood State : " + robotState.getDesiredHoodState().getHoodState());
          io.setHoodMotorVoltage(0);
        }
      }
      io.applyOutputs(outputs);
    } else {
      io.zero();
    }

    switch (robotState.getDesiredHoodState().getKickerState()) {
      case FEEDING -> {
        if (robotState.getShooterAssyReady())
          io.setKickerMotorVoltage(HoodKickerConstants.KICKER_MOTOR_VOLTAGE);
        else io.setKickerMotorVoltage(0);
      }
      case INDEXING -> io.setKickerMotorVoltage(-HoodKickerConstants.KICKER_MOTOR_VOLTAGE);
      case OFF -> io.setKickerMotorVoltage(0);
    }
  }

  public Command runStateful() {
    return Commands.none();
  }

  public Command runHoodMotor() {
    return runEnd(
        () -> {
          io.setHoodMotorVoltage(HoodKickerConstants.HOOD_MOTOR_VOLTAGE);
        },
        () -> {
          io.setHoodMotorVoltage(0.0);
        });
  }

  public Command runKickerMotor() {
    return runEnd(
        () -> {
          io.setKickerMotorVoltage(HoodKickerConstants.KICKER_MOTOR_VOLTAGE);
        },
        () -> {
          io.setKickerMotorVoltage(0.0);
        });
  }
}
