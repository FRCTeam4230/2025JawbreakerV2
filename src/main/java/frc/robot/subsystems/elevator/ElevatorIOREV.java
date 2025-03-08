package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.utils.Conversions;

public class ElevatorIOREV implements ElevatorIO {
  /** The gear ratio between the motor and the elevator mechanism */
  protected static final double GEAR_RATIO = 12.0;
  /**
   * The radius of the elevator pulley/drum, used for converting between rotations and linear
   * distance
   */
  protected final Distance elevatorRadius = Inches.of((1 + 7.0 / 8.0) / 2);

  private Angle setpoint;

  /** Leader motor controller * */
  protected final SparkFlex leader =
      new SparkFlex(ElevatorConstants.LEADER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leader.getEncoder();

  private final Encoder leaderExternalEncoder = new Encoder(5, 6);

  protected final DigitalInput beamBreakSensor =
      new DigitalInput(ElevatorConstants.beamBreakDIOPort);

  /*
  Encoder can take two ports, this gives the correct value in rotations for the elevator (when divided by -8192, which is how many ticks are in a rotation)
  ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder is what seems to be the correct thing to pass to the feedback sensor
  Maybe using ExternalEncoderConfig will work but it doesn't seem to have a way of applying the correct encoder to it, there is also something called SparkFlexExternalEncoder, but it doesn't seem to work yet
    */

  /** Follower * */
  protected final SparkFlex follower =
      new SparkFlex(ElevatorConstants.FOLLOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ElevatorConstants.UPPER_LIMIT_SWITCH_DIO_PORT);
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_DIO_PORT);

  private final SparkClosedLoopController pidController;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  public ElevatorIOREV() {
    leader.configure(
        createSparkFlexConfig(false),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    follower.configure(
        createSparkFlexConfig(true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    leaderEncoder.setPosition(0);
    pidController = leader.getClosedLoopController();
  }

  private SparkFlexConfig createSparkFlexConfig(boolean isFollower) {

    // I don't know if we will need a velocityConversionFacdtory but i'm fairly certain we will
    // maxMotion example
    // https://github.com/BroncBotz3481/FRC2025/blob/main/src/main/java/frc/robot/subsystems/ElevatorSubsystem.java
    // NON maxmotion, (use profiledPidController from wpilib)
    // https://github.com/frc868/2025-Ri3D/blob/main/src/main/java/frc/robot/subsystems/Elevator.java
    var config = new SparkFlexConfig();

    config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.inverted(true);
    config
        .encoder
        .velocityConversionFactor((1.0 / GEAR_RATIO) / 60.0)
        .positionConversionFactor(1.0 / GEAR_RATIO);
    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(ElevatorConstants.kP.get())
        .i(ElevatorConstants.kI.get())
        .d(ElevatorConstants.kD.get())
        .outputRange(-0.5, 0.5)
        .maxMotion
        .maxVelocity(ElevatorConstants.elevatorMaxVelocity)
        .maxAcceleration(ElevatorConstants.elevatorMaxAcceleration)
        .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01);

    config.externalEncoder.inverted(true).countsPerRevolution(8192);

    if (isFollower) {
      config.follow(leader, true);
    }

    return config;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    double elevatorRot = leaderEncoder.getPosition();
    // this is the encoder position
    // rev does it have something like this? inputs.leaderRotorPosition = leader.get //
    double elevatorVelRotPerSec = leaderEncoder.getVelocity();
    // getVelocity returns RPMS so this is totally probably wrong

    inputs.leaderPosition = Rotations.of(elevatorRot);
    inputs.leaderVelocity = RotationsPerSecond.of(elevatorVelRotPerSec);

    inputs.appliedVoltage = Volts.of(leader.getAppliedOutput());
    inputs.leaderStatorCurrent = Amps.of(leader.getOutputCurrent());

    inputs.followerStatorCurrent = Amps.of(follower.getOutputCurrent());
    inputs.encoderPosition = Rotations.of(leader.getEncoder().getPosition());
    inputs.encoderVelocity = RotationsPerSecond.of(leaderEncoder.getVelocity());

    inputs.dutyCycleEncoderPosition = Rotations.of(leaderExternalEncoder.get() / 8192.0);

    inputs.setpoint = setpoint;

    inputs.lowerLimit = !lowerLimitSwitch.get();
    inputs.upperLimit = !upperLimitSwitch.get();

    inputs.beamBreakTriggered = !beamBreakSensor.get();

    if (inputs.lowerLimit && inputs.leaderVelocity.magnitude() < 0) {
      stop();
    }

    //    if (inputs.upperLimit && inputs.leaderVelocity.magnitude() > 0) {
    //      pidController.setReference(
    //          inputs.leaderPosition.minus(Rotations.of(0.1)).in(Rotations),
    //          SparkBase.ControlType.kPosition);
    //    }
  }

  @Override
  public void setDistance(Distance distance) {
    double ff = feedforward.calculate(leaderEncoder.getVelocity());
    setpoint = Conversions.metersToRotations(distance, 1, elevatorRadius);
    pidController.setReference(
        setpoint.in(Rotations), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
