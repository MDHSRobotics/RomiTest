// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  //private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private static final double kWheelDiameterMeters = 0.070; // 2.75591 inches

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Odometry

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(m_gyro.getAngleZ()), new Pose2d(0, 0, new Rotation2d()));
  //private final Field2d m_field;

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use Meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double left, double right) {
    m_diffDrive.tankDrive(left, right, false);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    /*
    m_leftMotors[0].setVoltage(Math.min(leftVolts, 2));
    m_rightMotors[0].setVoltage(-Math.min(rightVolts, 2));
    */
    double voltage = RobotController.getBatteryVoltage();
    m_diffDrive.tankDrive(Math.min(leftVolts/voltage, 1.0), Math.min(rightVolts/voltage, 1.0), false);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftEncoderMeters() {
    return m_leftEncoder.getDistance();
  }


  public double getRightEncoderMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageEncoderMeters() {
    return (getLeftEncoderMeters() + getRightEncoderMeters()) / 2.0;
  }

  public double getLeftEncoderMetersPerSec() {
    return m_leftEncoder.getRate();
  }
    
  public double getRightEncoderMetersPerSec() {
    // TODO Note sure if this should be negated
    return -m_rightEncoder.getRate();
  }

  public double getLeftEncoderInches() {
    double meters = m_leftEncoder.getDistance();
    double inches = Units.metersToInches(meters);
    return inches;
  }

  public double getRightEncoderInches() {
    double meters = m_rightEncoder.getDistance();
    double inches = Units.metersToInches(meters);
    return inches;
  }

  public double getAverageEncoderInches() {
    return (getLeftEncoderInches() + getRightEncoderInches()) / 2.0;
  }

  public void setDeadband(double deadband) {
    m_diffDrive.setDeadband(deadband);
    return;
  }

  public double getAngularPositionInRadians() {
    // Note that the angle from the gyro must be negated because 
    // getAngle returns a clockwise positive
    double angularPositionRadians = -Math.toRadians(-m_gyro.getAngleZ());
    return angularPositionRadians;
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngleZ(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetOdometryToCurrentPose() {
    Pose2d currentPose = m_odometry.getPoseMeters();
    resetEncoders();
    m_odometry.resetPosition(currentPose, new Rotation2d(m_gyro.getAngleZ()));
    //if (Robot.isSimulation()) m_dts.setPose(pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
