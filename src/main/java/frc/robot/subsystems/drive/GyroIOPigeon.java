package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.Pigeon2Configurator;
import com.ctre.phoenixpro.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class GyroIOPigeon implements GyroIO {
    
    private Pigeon2 gyro;
    private Pigeon2Configuration gyroConfig;
    private Pigeon2Configurator gyroConfigurator;

    public GyroIOPigeon() {
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

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = NRUnits.constrainRad(gyro.getYaw().getValue() * Constants.TAU/360);
        inputs.pitch = NRUnits.constrainRad(gyro.getPitch().getValue() * Constants.TAU/360);
        inputs.roll = NRUnits.constrainRad(gyro.getRoll().getValue() * Constants.TAU/360);
    }
    
}
