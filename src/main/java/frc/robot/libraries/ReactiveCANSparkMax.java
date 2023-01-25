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

    // TODO * Tune default values, make them values ment for drive? obviously operating an arm
    // TODO * with antipush will take MUCH lower values.
    double recursiveErrorGrowthMax = 0.004; // 0.05 / 12 heehoo
    double recursiveErrorClampMax = 1;
    double P = 2, I = 0, D = 0.8;

    PIDController pid = new PIDController(P, I, D, 0);

    /**
     * Creates a new CANSpark max framwork to automatically use antipush systems.
     * @param Motor A new CANSparkMax with an encoder on the motor.
     * @implNote PID values are tuned (approximately) for drive motors by default.
     */
    public ReactiveCANSparkMax(CANSparkMax Motor) {
        m_Motor = Motor;
        m_Encoder = Motor.getEncoder();
        // PID input is the error, rather than speeds. Thus the goal
        // error must always be zero!
        pid.setTarget(0);
    }

    /**
     * Configures the embedded PID tuner used for how much
     * the recursiveError grows or decreases every frame of error.
     * @param P default 2
     * @param I default 0 
     * @param D default 0.8
     * @param maximum default 0.004
     * @return self, for chaining
     */
    public ReactiveCANSparkMax setPID(double P, double I, double D) {
        pid.kP = P; 
        pid.kI = I; 
        pid.kD = D;

        return this;
    }

    /**
     * Configures the maximum growth and clamp values.
     * @param RecursiveErrorClampMax default 1
     * @param RecursiveErrorGrowthMax default 0.004
     * @return self, for chaining
     */
    public ReactiveCANSparkMax setMaximums(double RecursiveErrorClampMax, double RecursiveErrorGrowthMax) {
        recursiveErrorClampMax = RecursiveErrorClampMax;
        recursiveErrorGrowthMax = RecursiveErrorGrowthMax;

        return this;
    }

    /**
     * Sets the motor in with anti push function
     * @param speed The goal speed of the motor from -1 to 1
     */
    public void set(double speed) {
        m_Motor.set(calculateErrorOffset(speed));
    }

    /**
     * Operates as the anti-push set function, though with a toggle ability.
     * Allowing antipush to be toggle via a button or trigger more easily.
     * @param speed The goal speed of the motor from -1 to 1
     * @param normalFunction Will act as the standard CANSparkMax set function when true
     */
    public void set(double speed, boolean normalFunction) {
        if (normalFunction) {
            m_Motor.set(speed);
        } else {
            set(speed);
        }
    }

    /**
     * drives the motor with a speed of 0 to allow
     * motor to react without input when this function
     * is called periodically
     */
    public void set() {
        set(0);
    }

    /**
     * @param S drive speed input
     * @return the sum of the drive input, and the recusive error value
     */
    double calculateErrorOffset(double S) {
        double idealRPM = S * Constants.MAX_NEO_RPM;
        // Distance of real vilocity from wanted vilocity
        double E = m_Encoder.getVelocity() - idealRPM;
        // Communicate to PID loop
        pid.setInput(E);
        // Scale pid output via recursiveError_Max and add to current recursive error
        recursiveError += pid.calculate(recursiveErrorGrowthMax, -recursiveErrorGrowthMax); 
        // Clamp recursive value for safety, though this still allows max power
        recursiveError = clamp(-1, 1, recursiveError);
        // Clamp the sum of the drive speed and recursive value
        return clamp(-1, 1, S + recursiveError);
    }

    /**
     * @param min
     * @param max
     * @param input
     * @return min if input < min, max if input > max
     */
    public static double clamp(double min, double max, double input) {
        if (input > max) {return max;}
        if (input < min) {return min;}
        return input;
    }
}
