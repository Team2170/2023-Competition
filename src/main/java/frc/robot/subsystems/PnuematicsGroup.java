package frc.robot.subsystems;

import frc.robot.Constants;

import javax.lang.model.util.ElementScanner14;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PnuematicsGroup {
    public DoubleSolenoid internalOne;
    public PneumaticHub m_pH ;
    public PnuematicsGroup(int hubId)
    {
        m_pH = new PneumaticHub(hubId);
        internalOne = m_pH.makeDoubleSolenoid(Constants.PnueMatics.forwardChannel, Constants.PnueMatics.reverseChannel);
    }
    public void extend_piston() {
        internalOne.set(Value.kForward);
    }
    
    public void retract_piston() {
        internalOne.set(Value.kReverse);
    }

    public void stop_piston() {        
        internalOne.set(Value.kOff);
    }

    public void operate(boolean isGrabbing , boolean isReleasing)
    {
        if ( isReleasing )
        {
            retract_piston();
        }
        else if ( isGrabbing)
        {
            extend_piston();
        }
        else{
            stop_piston();
        }
    }
}
