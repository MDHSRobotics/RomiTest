package frc;

// This file was cherry-picked from https://bitbucket.org/sciborgs4061/java-robot-2022-beta-public.git
// on Feb 6, 2022

public class SysIdDrivetrainLogger extends SysIdLogger {
    double m_primaryMotorVoltage;
    double m_secondaryMotorVoltage;

    public double getLeftMotorVoltage() {
        return m_primaryMotorVoltage;
      }
      
    public double getRightMotorVoltage() {
        return m_secondaryMotorVoltage;
      }
      
    public void log(double leftPosition, double rightPosition,
                                      double leftVelocity, double rightVelocity,
                                      double measuredAngle, double angularRate) {
        updateData();
        m_data.add(m_timestamp);
        m_data.add(m_primaryMotorVoltage);
        m_data.add(m_secondaryMotorVoltage);
        m_data.add(leftPosition);
        m_data.add(rightPosition);
        m_data.add(leftVelocity);
        m_data.add(rightVelocity);
        m_data.add( measuredAngle);
        m_data.add(angularRate);
        m_primaryMotorVoltage = (m_rotate ? -1 : 1) * m_motorVoltage;
        m_secondaryMotorVoltage = m_motorVoltage;
      }
      
      void reset() {
        super.reset();
        m_primaryMotorVoltage = 0;
        m_secondaryMotorVoltage = 0;
      }
      
      Boolean isWrongMechanism() {
        return !m_mechanism.equals("Drivetrain") && !m_mechanism.equals("Drivetrain (Angular)");
      }
    
}
