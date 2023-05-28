package frc.robot.lib.math;

import frc.robot.Constants;

public class NRUnits {
    
    public static double constrainRad(double rad) {
        return (((rad % Constants.TAU) + Constants.TAU) % Constants.TAU) - Constants.TAU/2;
    }

}
