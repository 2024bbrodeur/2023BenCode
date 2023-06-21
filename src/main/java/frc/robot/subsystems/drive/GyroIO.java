package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    
    @AutoLog
    public static class GyroIOInputs {
        public double yaw; //Radians [-Tau/2, Tau/2]
        public double pitch; //Radians [-Tau/2, Tau/2]
        public double roll; //Radians [-Tau/2, Tau/2]
    }

    public void updateInputs(GyroIOInputs inputs);

}
