import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import org.junit.jupiter.api.BeforeEach;

public class IntakeTest {
	static final double DELTA = 1e-2;
	
	IntakeIOTalonFX intake;
	TalonFXSimState rollerState;
	TalonFXSimState deployState;
	DCMotorSim rollerSim;
	DCMotorSim deploySim;
	
	
	@BeforeEach
	void setup() {
		assert HAL.initialize(500, 0);
		
		intake = new IntakeIOTalonFX();
		
		
		var gearbox = DCMotor.getKrakenX60Foc(1);
		rollerSim = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(gearbox, 0.001,
						Constants.IntakeConstants.ROLLER_GEAR_RATIO),
				gearbox);
		deploySim = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(gearbox, 0.001,
						Constants.IntakeConstants.DEPLOY_GEAR_RATIO),
				gearbox);
	}

}
