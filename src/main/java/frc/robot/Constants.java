package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.lib.math.NRUnits;

public final class Constants {
    
    public static final double TAU = Math.PI * 2;

    public static final class Arm {

    }

    public static final class Auto {

    }

    public static final class Drive {
        public static final int GYRO_PORT = 0;

        public static final class GYRO_MOUNT_POSE {
            public static final double YAW = -0.263672;
            public static final double PITCH = 0.307617;
            public static final double ROLL = -0.483398;
        }

        public static final int FRONT_RIGHT_TURN_PORT = 0;
        public static final int FRONT_LEFT_TURN_PORT = 1;
        public static final int BACK_LEFT_TURN_PORT = 2;
        public static final int BACK_RIGHT_TURN_PORT = 3;

        public static final int FRONT_RIGHT_MOVE_PORT = 4;
        public static final int FRONT_LEFT_MOVE_PORT = 5;
        public static final int BACK_LEFT_MOVE_PORT = 6;
        public static final int BACK_RIGHT_MOVE_PORT = 7;
        
        public static final int FRONT_RIGHT_SENSOR_PORT = 0;
        public static final int FRONT_LEFT_SENSOR_PORT = 1;
        public static final int BACK_LEFT_SENSOR_PORT = 2;
        public static final int BACK_RIGHT_SENSOR_PORT = 3;

        public static final double FRONT_RIGHT_OFFSET_NU = 0.043701 - 0.5;
        public static final double FRONT_LEFT_OFFSET_NU = 0.194336 - 0.5;
        public static final double BACK_LEFT_OFFSET_NU = 0.286133;
        public static final double BACK_RIGHT_OFFSET_NU = -0.109863 + 0.5;

        public static final double MOVE_STATOR_CURRENT = 60;
        public static final double MOVE_SUPPLY_CURRENT = 80;
        public static final double TURN_STATOR_CURRENT = 60;
        public static final double TURN_SUPPLY_CURRENT = 80;

        public static final double MOVE_GEAR_RATIO = 8.14;
        public static final double TURN_GEAR_RATIO = 150. / 7;

        public static final double WHEEL_RADIUS = Units.inchesToMeters(1.925);

        public static final double MOVE_KV = 0.01;
        public static final double MOVE_KS = 0.0;
        public static final double MOVE_KP = 0.01;
        public static final double MOVE_KI = 0.0;
        public static final double MOVE_KD = 0.0;

        public static final double TURN_KV = 0.01;
        public static final double TURN_KS = 0.0;
        public static final double TURN_KP = 0.5;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.0;

        public static final double MAX_MOVE_VELOCITY_NU = 103;
        public static final double MAX_MOVE_VELOCITY_MPS = NRUnits.Drive.Move.NUToMeters(MAX_MOVE_VELOCITY_NU);

        public static final double TURN_CRUISE_VELOCITY = 103;
        public static final double TURN_ACCELERATION = 180;
        public static final double TURN_JERK = 0;
    }

    public static final class Field {

    }

    public static final class Grabber {

    }

    public static final class Joysticks {
        public static final int LEFT_JOYSTICK_PORT = 1;
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int CONTROLLER_PORT = 2;
    }

    public static final class Logging {

    }

    public static final class Misc {

    }

}
