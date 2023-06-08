package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    
    private ModuleIO[] modules;
    private GyroIO gyro;

    public Drive() {
        modules = new ModuleIO[4];
        for(int i = 0; i < modules.length; i++) {
            modules[i] = new ModuleIOTalonFX(i);
        }

        gyro = new GyroIO();
    }

    

}
