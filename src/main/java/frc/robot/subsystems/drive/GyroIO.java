package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.Pigeon2Configurator;
import com.ctre.phoenixpro.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class GyroIO {
    
    private Pigeon2 gyro;
    private Pigeon2Configuration gyroConfig;
    private Pigeon2Configurator gyroConfigurator;

    public GyroIO() {
        gyro = new Pigeon2(Constants.Drive.GYRO_PORT, "drivet");
        gyroConfig = new Pigeon2Configuration();
        gyroConfigurator = gyro.getConfigurator();

        config();
    }

    private void config() {
        gyroConfig.FutureProofConfigs = true;
        gyroConfig.MountPose.MountPoseYaw = Constants.Drive.GYRO_MOUNT_POSE.YAW;
        gyroConfig.MountPose.MountPosePitch = Constants.Drive.GYRO_MOUNT_POSE.PITCH;
        gyroConfig.MountPose.MountPoseRoll = Constants.Drive.GYRO_MOUNT_POSE.ROLL;
        gyroConfigurator.apply(gyroConfig);
        gyro.setYaw(0);
    }

    public double getAngle() {
        return NRUnits.constrainRad(getYaw() * Constants.TAU/360);
    }

    public double getYaw() {
        return gyro.getYaw().getValue();
    }

    public double getPitch() {
        return gyro.getPitch().getValue();
    }

    public double getRoll() {
        return gyro.getRoll().getValue();
    }

}
