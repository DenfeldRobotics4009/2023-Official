package frc.robot.libraries;
import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    
    
    /**
     * @author Nikolai (AdmiralTryhard)
     */
    
    
    public double pvalue;
    public double ivalue;
    public double dvalue;
    public Timer startcorrect = new Timer();
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double input;
    public double target;
    public double accepted;
    public double last_error;
    public double PID_value;
    public double final_value;
    public double error;

    /**
     * this is what you need to set up the PID controller. You must use this method before doing anything else.
     * @param input The number you are trying to control with the PID loop
     * @param kP Proportional
     * @param kI Integral
     * @param kD Derivative
     * @param F constant
     */
    public PIDController(double P, double I, double D, double F){
        kP = P;
        kI = I;
        kD = D;
        kF = F;
        
    }

    /**
    * Just to return the values of PID in an array in order P, I, D.
    */
    public double[] getPID(){
        double[] PIDvalues = new double[] {kP, kI, kD, kF};
        return PIDvalues;
    }
    /**
     * this function tells the pid controller where you want your number to be
     * @param target this is where you're trying to get the input to.
     */
    public void setTarget(double goal){
        target = goal;

    }

    /**
     * this will spout out the target number you previously stated
     */
    
    public double getTarget(){
        return target;
    }

    /**
     * 
     * @param tobechanged the real-time number you will have to be changed
     */
    public void setInput(double tobechanged){
        input = tobechanged;
    }

    /**
     * 
     * spits out the input that you should have set.
     */
    public double getInput(){
        return input;
    }

    /**
     * Use this if you want to set a tolerance on your pidcontroller
     * @param tolerance the number you want to allow for a maximum tolerance
     */
    public void setTolerance(double tolerance){
        accepted = tolerance;
    }

    public double getTolerance(){
        return accepted;
    }

    /**
     * this is where the PID value is calculated and used at your discretion
     * @param maximumoutput based on your number being controlled, you may want to set limits (1 is a typical limit as it represents 100% power)
     * @param minimumoutput this is essentially the same as the maximum, but likely in the opposite direction (-1 is the typical limit as -1 represents 100% power in the opposite direction)
     * 
     */
    public double calculate(double maximumoutput, double minimumoutput){
        error = getTarget() - getInput();

        pvalue = error * kP;

        dvalue = kD * ((error - last_error)/0.05);

        last_error = error;
        
        if(last_error * error <= 0 || Math.abs(error) <= getTolerance()){
            ivalue = 0;
        }
        else {
            ivalue += (error * 0.05 * kI);
        }
       
        PID_value = pvalue +ivalue + dvalue;

    
      
        if(Math.abs(error) < getTolerance()){
            final_value = 0;
        }
        else if(PID_value > 0) {
            final_value = PID_value + kF;
        }
        else{
        final_value = PID_value - kF;
        }
        if(final_value >= maximumoutput){
            return maximumoutput;
        }
        else if(final_value <= minimumoutput){
            return minimumoutput;
        }
        else {
            return final_value;
        }
    }

    /**
     * 
     * This is a commonly used team function that allows for full manual control when you wish to input it. Otherwise, a timer will go off to check.
     * After a short period of time, the PID Controller will act to maintain the last known input when you had manual control.
     * DOUBLE CHECK that the manual input affects the number that you set as the input for the PID Controller. 
     * Otherwise, you may be in for a wacky ride.
     * @param manualinput this is the number of an variable that you could get from manual control. Like, say 
     * for a motor, this might be a speed variable you get from a joystick or other source outside of this PID controller.
     * The input from configure earlier will be used as your set point as things go along.
     * @param maximumoutput if your output can't exceed a certain number, set this to that number so things won't go over 100%
     * @param minimuminput same as above, but in the opposite direction
     *
     */
    public double semiAutomate(double manualinput, double maximumoutput, double minimumoutput){

        startcorrect.start();

        if (Math.abs(manualinput) > 0){
            setInput(input);
            setTarget(input);
            startcorrect.reset();
            return manualinput;
        }
        else if (startcorrect.get() < 0.33){
            return 0;
        }
        else {
            return calculate(maximumoutput, minimumoutput);
        }
    }

} 