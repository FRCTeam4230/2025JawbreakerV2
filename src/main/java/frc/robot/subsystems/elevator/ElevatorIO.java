// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public boolean encoderConnected = false;
    public boolean beamBreakTriggered = false;

    public Angle leaderPosition = Rotations.of(0);
    public Angle leaderRotorPosition = Rotations.of(0);
    public Angle encoderPosition = Rotations.of(0);

    public AngularVelocity leaderVelocity = RotationsPerSecond.of(0);
    public AngularVelocity leaderRotorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity encoderVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current leaderStatorCurrent = Amps.of(0);
    public Current followerStatorCurrent = Amps.of(0);
    public Current leaderSupplyCurrent = Amps.of(0);
    public Current followerSupplyCurrent = Amps.of(0);

    public boolean lowerLimit = false;
    public boolean upperLimit = false;

    public Angle setpoint;

    public Angle dutyCycleEncoderPosition = Rotations.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  default void setDistance(Angle distance) {}

  /** Stop in open loop. */
  default void stop() {}

  default void setVoltage(Voltage voltage) {}
}
