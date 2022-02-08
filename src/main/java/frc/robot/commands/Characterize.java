package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.SysIdDrivetrainLogger;

public class Characterize extends CommandBase {
    private final Drivetrain m_drivetrain;
    private SysIdDrivetrainLogger m_logger;   
    private Double m_prevAngle = 0.0;
    private Double m_prevTime = 0.0;
    private boolean m_resetComplete;
    public Characterize(Drivetrain subsystem) {

        m_drivetrain = subsystem;
        addRequirements(m_drivetrain);   
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // reset gyro and encoders
        // set timeperiod to .005
        m_drivetrain.setDeadband(0.0);
        // The following is called for the side-effect of resetting the 
        // drivebase odometers.
        m_drivetrain.resetOdometryToCurrentPose(); 
        m_logger = new SysIdDrivetrainLogger();
        m_logger.updateThreadPriority();
        m_logger.initLogging();
        m_resetComplete = false;
    }
   
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftPosition = m_drivetrain.getLeftEncoderMeters();
        double leftRate = m_drivetrain.getLeftEncoderMetersPerSec();
        double rightPosition = m_drivetrain.getRightEncoderMeters();
        double rightRate = m_drivetrain.getRightEncoderMetersPerSec();
        double angularPosition = m_drivetrain.getAngularPositionInRadians();
        double deltaAngle = angularPosition - m_prevAngle;
        double now = Timer.getFPGATimestamp();
        double deltaTime = now - m_prevTime;
        double angularRate = m_prevTime==0 || deltaTime==0 ? 0.0 : deltaAngle/deltaTime;
        m_prevAngle = angularPosition;
        m_prevTime = now;

        // Resetting encoders takes non-zero time on CAN-based encoders
        // Wait for the reset to complete
        if (!m_resetComplete) {
            if (leftPosition > 0.01 || rightPosition > 0.01) return;
            m_resetComplete = true;
        }
        m_logger.log(leftPosition, rightPosition, leftRate, 
                   rightRate, angularPosition, angularRate);
        m_drivetrain.tankDriveVolts(m_logger.getLeftMotorVoltage(), 
                               m_logger.getRightMotorVoltage());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Characterization done; disabled");
        m_drivetrain.tankDrive(0, 0);
        m_logger.sendData();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}

