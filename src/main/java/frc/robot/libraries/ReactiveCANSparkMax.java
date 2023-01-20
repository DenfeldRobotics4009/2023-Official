package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

/**
 * A class holding info for, and to control, a single
 * CANSpark max to apply antipush for multiple purposes.
 * 
 * @implNote Antipush will try to apply power to this motor
 * when its distance from its desired speed is higher, thus if
 * the motor moves and no input is given, the motor
 * will apply power to maintain at a zero speed. When the motor
 * does this, it will be applying power to a non-moving motor
 * causing extreme rises in the motors tempurature.
 */
public class ReactiveCANSparkMax {
    CANSparkMax m_Motor;
    RelativeEncoder m_Encoder;
    
    double recursiveError = 0;

    double recursiveError_Max = 0.5;
    double P = 0, I = 0, D = 0;

    /**
     * Creates a new CANSpark max framwork to automatically use antipush systems
     * @param Motor A new CANSparkMax with an encoder on the motor
     */
    public ReactiveCANSparkMax(
        CANSparkMax Motor,
        double RecursiveMax, 
        double P, double I, double D
    ) {
        m_Motor = Motor;
        m_Encoder = Motor.getEncoder();
    }

    /**
     * 
     * @param speed
     */
    public void set(double speed) {
        
    }

    /**
     * drives the motor with a speed of 0 to allow
     * motor to react without input when this function
     * is called periodically
     */
    public void set() {
        set(0);
    }

    double calculateErrorOffset(int i, double S) {
        a_dpid[i].setTarget(0);
    
        double setSpeed = S;
        double idealRPM = S * MAX_NEO_RPM;
        double E = a_mencoder[i].getVelocity() - idealRPM;
    
        DriveError.Entries[i].setDouble(E);
    
        a_dpid[i].setInput(E);
    
        recursiveError[i] += a_dpid[i].calculate(admax, admin); 
        recursiveError[i] = clamp(-MAX_VOLTAGE, MAX_VOLTAGE, recursiveError[i]);
        return clamp(-MAX_VOLTAGE, MAX_VOLTAGE, setSpeed + recursiveError[i] );
      }

}
