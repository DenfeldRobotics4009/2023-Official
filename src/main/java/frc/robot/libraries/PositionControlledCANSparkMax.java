package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * 
 * This is a speed based antipush
 * 
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
public class PositionControlledCANSparkMax {

    public String name = "";
    public static ShuffleboardTab kTab = Shuffleboard.getTab("ControlledCAN");
    public GenericEntry EntrySpeed, EntryError, EntryPos;

    public CANSparkMax m_Motor;
    public RelativeEncoder m_Encoder;
    
    public Timer Interval = new Timer();

    public double idealPosition = 0;

    double P = 0.1, I = 0, D = 0, maxPidOutput = 1; // Default PID tuning

    public PIDController pid = new PIDController(P, I, D, 0);

    /**
     * Positional limits. If both are zero, limits are ignored
     */
    double maxEPos = 0, minEPos = 0;

    /**
     * Creates a new CANSpark max framwork to automatically use antipush systems.
     * 
     * @implNote The encoder position is set to 0 during startup, thus the robot
     * MUST be in its zero position before startup for its manipulator(s) to work properly
     * 
     * @param Motor A new CANSparkMax with an encoder on the motor.
     * @implNote PID values are tuned (approximately) for drive motors by default.
     */
    public PositionControlledCANSparkMax(CANSparkMax Motor, String Name) {
        m_Motor = Motor;
        m_Encoder = Motor.getEncoder();
        m_Encoder.setPosition(0); 
        // PID input is the error, rather than speeds. Thus the goal
        // error must always be zero!
        pid.setTarget(0);

        Interval.start();

        name = Name;
        EntrySpeed = kTab.addPersistent(name + " Speed",  0).getEntry();
        EntryError = kTab.addPersistent(name + " Error",  0).getEntry();
        EntryPos = kTab.addPersistent(name + " Rotations", 0).getEntry();
    }

    /**
     * Configures the embedded PID tuner for how much
     * power is outputted based upon the error
     * @param P default 2
     * @param I default 0 
     * @param D default 0.8
     * @param maximum default 0.004
     * @return self, for chaining
     */
    public PositionControlledCANSparkMax setPID(double P, double I, double D) {
        pid.kP = P; 
        pid.kI = I; 
        pid.kD = D;

        return this;
    }

    /**
     * Configures the maximum PID outputs
     * @param double default 1
     */
    public PositionControlledCANSparkMax setMaximums(
        double MaxPIDOutput,
        double MaxEncoderPosition,
        double MinEncoderPosition
    ) {
        maxPidOutput = MaxPIDOutput;

        maxEPos = MaxEncoderPosition;
        minEPos = MinEncoderPosition;

        return this;
    }

    /**
     * Sets the motor in with anti push function
     * @param speed The goal speed of the motor from -1 to 1
     */
    public void set(double speed) {
        double s = calculateErrorOffset(speed);

        EntryPos.setDouble(m_Encoder.getPosition());

        if (maxEPos != 0 && minEPos != 0) {
            if (m_Encoder.getPosition() >= maxEPos && s > 0) {
                s = 0; 
                LimitMax();
            }
            else if (m_Encoder.getPosition() <= minEPos && s < 0) {
                s = 0; 
                LimitMin();
            }
        }

        m_Motor.set(s);
        EntrySpeed.setDouble(s);
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
    public double calculateErrorOffset(double S) {

        final double i = 0.1;
        if (Interval.hasElapsed(i)) {
            idealPosition += S * (5676/60) * i;
        }

        // Distance of real vilocity from wanted vilocity
        double E = m_Encoder.getPosition() - idealPosition;
        EntryError.setDouble(E); // Display
        // Communicate to PID loop
        pid.setInput(E);
        // Clamp the sum of the drive speed and recursive value
        return clamp(-1, 1, pid.calculate(maxPidOutput, -maxPidOutput));
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

    /**
     * Sets the position goal to the encoders current position
     */
    public void Reset() {
        idealPosition = m_Encoder.getPosition();
    }

    /**
     *If this function is called when
     * maxEPos is 0, it will jerk towards 0
     */
    public void LimitMax() {
        idealPosition = maxEPos;
    }

    /**
     * If this function is called when
     * minEPos is 0, it will jerk towards 0
     */
    public void LimitMin() {
        idealPosition = minEPos;
    }

    /**
     * Updates displayed values
     */
    public void Update() {
        EntryPos.setDouble(m_Encoder.getPosition());
    }
}
