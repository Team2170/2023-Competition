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
import edu.wpi.first.wpilibj.XboxController;

public class RobotArm extends SubsystemBase {
    public MotorGroup joint1;
    public MotorGroup joint2;
    public PnuematicsGroup pnueJoint1;

    public int currentPos = -1;
    public int setpoint = 0;

    public RobotArm() {
        joint1 = new MotorGroup(Constants.LowerArm.MasterId,Constants.LowerArm.FollowerId,Constants.LowerArm.EncoderId, 0.1, 0.1, 0.1, 1, 100,"Lower");
        joint2 = new MotorGroup(Constants.UpperArm.MasterId,Constants.UpperArm.FollowerId,Constants.UpperArm.EncoderId, 0.1, 0.1, 0.1, 1, 100,"Upper");
        pnueJoint1 = new PnuematicsGroup(Constants.ph);
    }

    public void periodic(Boolean loading,Boolean low,Boolean mid,Boolean high,Boolean isManual, double manualDirection) {
        if( isManual )
        {
            
            if(manualDirection > 0.1)
            {
                joint1.raise_arm_manually();
                joint2.raise_arm_manually();
            }else if(manualDirection < -0.1)
            {
                joint1.lower_arm_manually();
                joint2.lower_arm_manually();

            }else{
            }
        }
        else
        {
            boolean arm_moved = false;
            if(loading)
            {
                joint1.raise_arm_loading_position();
                //joint2.raise_arm_loading_position();
                arm_moved = true;
            }
            else if(low)
            {
                joint1.raise_arm_low_scoring();
                //joint2.raise_arm_low_scoring();
                arm_moved = true;
            }
            else if(mid)
            {
                joint1.raise_arm_mid_scoring();
                //joint2.raise_arm_mid_scoring();
                arm_moved = true;
            }
            else if(high)
            {
                joint1.raise_arm_high_scoring();
                //joint2.raise_arm_high_scoring();
                arm_moved = true;

            }
            else{
                joint1.stop_arm();
                //joint2.stop_arm();
            }
            if (arm_moved)
            {
                //joint1.operate();
                //joint2.operate();
            }
        }
        joint1.DisplayEncoder();
        joint2.DisplayEncoder();
    }
}