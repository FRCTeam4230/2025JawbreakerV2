package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulation implementation of the elevator subsystem. This class extends ElevatorIOCTRE to provide
 * a physics-based simulation of the elevator mechanism using WPILib's simulation classes.
 *
 * <p>The simulation models: - Dual Kraken X60 FOC motors - Realistic elevator physics including
 * gravity - Position and velocity feedback through simulated encoders - Battery voltage effects
 */
public class ElevatorIOSIMREV extends ElevatorIOREV {

  /** Physics simulation model for the elevator mechanism */
  private final ElevatorSim motorSimModel;

  /** Simulation state for the leader motor */
  private final SparkFlexSim leaderSim;

  /** Simulation state for the throughbore */
  private final SparkRelativeEncoderSim encoderSim;

  /**
   * Constructs a new ElevatorIOSIM instance. Initializes the physics simulation with realistic
   * parameters including: - Dual Neo 500 motors -10 pound carriage mass -8 foot maximum height -
   * Gravity simulation enabled
   */
  public ElevatorIOSIMREV() {
    super(); // Initialize hardware interface components

    DCMotor motor = DCMotor.getNeoVortex(2);
    // Get simulation states for all hardware
    leaderSim = new SparkFlexSim(leader, motor);
    encoderSim = new SparkRelativeEncoderSim(leader);

    // Create elevator physics model
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createElevatorSystem(
            motor, // 1 Neo  motor
            Pounds.of(10).in(Kilograms), // Carriage mass (10 lbs -> kg)
            elevatorRadius.in(Meters), // Drum radius in meters
            GEAR_RATIO); // Motor to mechanism gear ratio

    // Initialize elevator simulation
    motorSimModel =
        new ElevatorSim(
            linearSystem,
            motor,
            0, // Initial position
            Feet.of(8).in(Meters), // Maximum height (8 feet -> meters)
            true, // Enable gravity simulation
            0); // Start at bottom position
  }

  /**
   * Updates the simulation model and all simulated sensor inputs.
   *
   * @param inputs The ElevatorIOInputs object to update with simulated values
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update base class inputs first
    super.updateInputs(inputs);

    motorSimModel.setInputVoltage(
        leaderSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    motorSimModel.update(0.02);

    double simulatedMotorRPM =
        motorSimModel.getVelocityMetersPerSecond() / elevatorRadius.in(Meters);

    leaderSim.iterate(simulatedMotorRPM, RobotController.getBatteryVoltage(), 0.02);

    //    leaderSim.setBusVoltage(RobotController.getBatteryVoltage());
    //
    //    // Update physics simulation
    //    motorSimModel.setInput(leaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    //    motorSimModel.update(0.02); // Simulate 20ms timestep (50Hz)
    //
    //    leaderSim.iterate(
    //        motorSimModel.getVelocityMetersPerSecond(),
    //        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
    //        0.02);
    //
    //    // Convert linear position/velocity to rotational units for based on encoder
    //    Angle position =
    //        Conversions.metersToRotations(
    //            Meters.of(motorSimModel.getPositionMeters()), 1 / 12.0, elevatorRadius);
    //
    //    // Convert linear velocity to angular velocity based on encoder
    //    AngularVelocity velocity =
    //        Conversions.metersToRotationsVel(
    //            MetersPerSecond.of(motorSimModel.getVelocityMetersPerSecond()),
    //            1 / 12.0,
    //            elevatorRadius);
    //
    //    // Update simulated motor readings converts through gear ratio
    //    leaderSim.setPosition(position.times(GEAR_RATIO).in(Degrees));
    //    leaderSim.setVelocity(velocity.times(GEAR_RATIO).in(DegreesPerSecond));
    //    encoderSim.setPosition(motorSimModel.getPositionMeters());
  }
}
