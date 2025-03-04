// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of the REV-based arm IO. This class extends ArmIOREV so that in
 * simulation the arm's physics—including gravity and motion limits are modeled by WPILib's
 * SingleJointedArmSim. The simulated sensor readings (in raw motor rotations) are then injected so
 * that the rest of your code sees values in rotations.
 */
public class ArmIOREVSIM extends ArmIOREV {

  /** Physics simulation model for the arm mechanism */
  private final SingleJointedArmSim armSimModel;

  /** Simulation state for the leader motor */
  private final SparkFlexSim leaderSim;

  private final SparkAbsoluteEncoderSim absoluteEncoderSim;

  /** Constructs a new ArmIOREVSIM instance. */
  public ArmIOREVSIM() {
    super(); // Initialize REV hardware interface components

    // Choose a motor model for simulation; here we assume two NEOs in parallel.
    DCMotor motor = DCMotor.getNeoVortex(1);

    // Retrieve simulation state objects from the REV devices.
    leaderSim = new SparkFlexSim(leader, motor);
    absoluteEncoderSim = leaderSim.getAbsoluteEncoderSim();

    // Define arm physical properties.
    Distance armLength = Inches.of(23.75);
    Mass armMass = Pounds.of(4.84);

    // Calculate the arm's moment of inertia (MOI) using WPILib's helper.
    double armMOI = SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms));

    // Create the arm model. Note that SingleJointedArmSim uses radians, and the GEAR_RATIO here is
    // the same
    // as in the hardware code.
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createSingleJointedArmSystem(motor, armMOI, GEAR_RATIO);

    Angle startingAngle = Degrees.of(0);
    // Initialize the arm simulation:
    armSimModel =
        new SingleJointedArmSim(
            linearSystem,
            motor,
            GEAR_RATIO,
            armLength.in(Meters),
            Degrees.of(0).in(Radians),
            Degrees.of(180).in(Radians),
            false,
            startingAngle.in(Radians));
    leaderSim.setPosition(startingAngle.in(Rotations));
  }

  /**
   * Updates the simulation model and injects simulated sensor values.
   *
   * @param inputs The ArmIOInputs object to update with simulated values.
   */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // First update base class inputs (if any logic exists there).
    super.updateInputs(inputs);

    armSimModel.setInputVoltage(leaderSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    armSimModel.update(0.020); // Simulate a 20ms timestep

    // Similarly, convert angular velocity (rad/s) into motor RPM.
    // First, arm velocity in revolutions per second is (simAngularVelocityRadPerSec / (2π)).
    // Then, motor RPM = (arm rev/s) * 60 * GEAR_RATIO.
    double simulatedMotorRPM =
        Units.radiansPerSecondToRotationsPerMinute(armSimModel.getVelocityRadPerSec() * GEAR_RATIO);

    // Update the simulated motor encoder readings (raw sensor values).
    leaderSim.iterate(simulatedMotorRPM, RobotController.getBatteryVoltage(), 0.02);
    absoluteEncoderSim.iterate(simulatedMotorRPM, 0.02);
  }
}
