package frc.robot.lib.math;

import frc.robot.Constants;

public class NRUnits {
    
    public static double constrainRad(double rad) {
        return (((rad % Constants.TAU) + Constants.TAU) % Constants.TAU) - Constants.TAU/2;
    }

    public static class Drive {

        public static class Turn {
            public static double NUToRad(double NU) {
                return NU / Constants.Drive.TURN_GEAR_RATIO * Constants.TAU;
            }

            public static double radToNU(double rad) {
                return rad * Constants.Drive.TURN_GEAR_RATIO * Constants.TAU;
            }
        }

        public static class Move {
            public static double NUToRad(double NU) {
                return NU / Constants.Drive.MOVE_GEAR_RATIO * Constants.TAU;
            }

            public static double radToNU(double rad) {
                return rad * Constants.Drive.MOVE_GEAR_RATIO * Constants.TAU;
            }

            public static double NUToMeters(double NU) {
                return NU / Constants.Drive.MOVE_GEAR_RATIO * (Constants.Drive.WHEEL_RADIUS*Constants.TAU);
            }

            public static double metersToNU(double meters) {
                return meters / (Constants.Drive.WHEEL_RADIUS*Constants.TAU) * Constants.Drive.MOVE_GEAR_RATIO;
            }
        }

    }

}
