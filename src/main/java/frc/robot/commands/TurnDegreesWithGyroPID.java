// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.Drivetrain;

public class TurnDegreesWithGyroPID extends PIDCommand {
  private final Drivetrain m_drive;
  private final double m_targetAngleDegrees;
  private double m_gyroStartAngle;

  /**
   * Creates a new TurnDegreesWithGyro. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed; then stop based on the change in gryo reading.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param m_targeAngleDegrees Degrees to turn. Leverages gyro to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesWithGyroPID(double targetAngleDegrees, Drivetrain drive) {

    super(
      new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
      // Close loop on heading
      drive::getHeading,
      // Set reference to target
      targetAngleDegrees,
      // Pipe output to turn robot
      output -> drive.arcadeDrive(0, output),
      // Require the drive
      drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

    m_targetAngleDegrees = targetAngleDegrees;
    m_drive = drive;
    m_gyroStartAngle = 0.0;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_gyroStartAngle = m_drive.getGyroAngleZ();

    System.out.format("Initial gyro angle is %.2f\n", m_gyroStartAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);

    double finalGyroAngle = m_drive.getGyroAngleZ();
    double overRun = finalGyroAngle - m_gyroStartAngle - m_targetAngleDegrees;
    System.out.format("Final gyro reading = %.2f, which is an error of %.2f\n", finalGyroAngle, overRun);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double currentGyroAngle = m_drive.getGyroAngleZ();
    double deltaAngle = currentGyroAngle - m_gyroStartAngle;

    System.out.format("Current angle = %.2f; target angle = %.2f; delta angle = %.2f\n", currentGyroAngle, m_gyroStartAngle, deltaAngle);

    // End when the controller is at the reference.
    boolean finished = getController().atSetpoint();

    return finished;
  }

}
