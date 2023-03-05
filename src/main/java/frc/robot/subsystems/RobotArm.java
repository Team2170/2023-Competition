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
        joint2 = new MotorGroup(Constants.LowerArm.MasterId,Constants.LowerArm.FollowerId,Constants.LowerArm.EncoderAId,Constants.LowerArm.EncoderBId, 0.1, 0.1, 0.1, 1, 100,"Lower");
        joint1 = new MotorGroup(Constants.UpperArm.MasterId,Constants.UpperArm.FollowerId,Constants.UpperArm.EncoderAId,Constants.UpperArm.EncoderBId, 0.1, 0.1, 0.1, 1, 100,"Upper");
        pnueJoint1 = new PnuematicsGroup(Constants.ph);
    }

    public void periodic(Boolean loading,Boolean low,Boolean mid,Boolean high,Boolean isManual, double manualDirection) {
        if( isManual )
        {
            if(manualDirection > 0.2)
            {
                joint1.raise_arm_manually();
                //joint2.raise_arm_manually();
            }else if(manualDirection < -0.2)
            {
                joint1.lower_arm_manually();
                //joint2.lower_arm_manually();
            }else{
                joint1.stop_arm();
                //joint2.stop_arm();
            }
        }
    }
    public void DisplayEncoder()
    {
        joint1.DisplayEncoder();
        joint2.DisplayEncoder();
    }
}