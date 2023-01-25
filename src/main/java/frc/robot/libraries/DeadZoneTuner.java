package frc.robot.libraries;

public class DeadZoneTuner{
    

    /**
     * 
     * This should handle customizations (especially for deadzones) that one may want to use on items that have a scale range
     * This should also be able to handle any additional scaling outside of the deadzones.
     */
    
     /**
     * adjustForDeadzone is entirely meant for the deadzone to be used, and it automatically utilizes a previously derived math equation
     * to start
     * @param input this number is typically a controller value, regardless of axis.
     * @param minimumrange
     * @param maximumrange
     * @param deadzone how big is it?
     * @param isInverted just to quickly
     * 
     */
    
    public static double adjustForDeadzone(double input, double deadzone, boolean isInverted){
        double result;
        if(Math.abs(input) <= deadzone){
            result = 0;
        }
        else{
            result = Math.signum(input) *((Math.abs(input) - deadzone) *(1/(1 - deadzone)));
        }
        if (isInverted){
            return result * -1;
        }
        else{
        return result;
        }
    }
    /** 1st and foremost, this assumes that the minimum input is the opposite of maximum, otherwise initial scale for your input (as it is the most common for our applications)
     * This will let you adjust the input to any scale that you want by setting the minimum and maximum.
     * @param input this is the number that you will be adjusting to scale (usually on a scale of its own)
     * @param minimuminput this is the smallest number your input can give
     * @param maximuminput this is the biggest number your input can give
     * @param minimumdesired this will be the smallest number that the function will input that you want if you bring your input to its minimum
     * @param maximumdesired this will be the biggest if you put in the max.
     * 
     */
    public double changescale(double input, double minimumdesired, double maximumdesired, double minimuminput, double maximuminput){
        double output;

        double initialscale = (input + maximuminput) / (maximuminput - minimuminput);

        output = (initialscale * (maximumdesired - minimumdesired) + minimumdesired);
        
        return output;
    }
}