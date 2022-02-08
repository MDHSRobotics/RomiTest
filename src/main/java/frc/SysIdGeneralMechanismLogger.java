package frc;

// This file was cherry-picked from https://bitbucket.org/sciborgs4061/java-robot-2022-beta-public.git
// on Feb 6, 2022

public class SysIdGeneralMechanismLogger extends SysIdLogger{
    public double m_primaryMotorVoltage;
    public double getMotorVoltage() {
        return m_primaryMotorVoltage;
    }
    
    public void log(double measuredPosition, double measuredVelocity) {
        updateData();
        m_data.add(m_timestamp);
        m_data.add(m_primaryMotorVoltage);
        m_data.add(measuredPosition); 
        m_data.add(measuredVelocity);    
        m_primaryMotorVoltage = m_motorVoltage;
    }
    
    public void reset() {
        super.reset();
        m_primaryMotorVoltage = 0;
    }
    
    public Boolean isWrongMechanism() {
    return m_mechanism != "Arm" && m_mechanism != "Elevator" &&
           m_mechanism != "Simple";
    }
      
}
