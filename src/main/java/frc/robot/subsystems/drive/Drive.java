package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.util.SwerveState;

public class Drive extends SubsystemBase {
    
    private Module[] modules;
    private GyroIO gyro;
    private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public boolean isFieldCentric;

    @Override
    public void periodic() {
        for(Module module : modules) {
            module.periodic();
        }

        gyro.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Gyro", inputs);

    }

    public Drive() {
        modules = new Module[4];
        for(int i = 0; i < modules.length; i++) {
            modules[i] = new Module(new ModuleIOTalonFX(i), i);
        }

        gyro = new GyroIOPigeon();

        isFieldCentric = true;
    }

    /*
     * x: [-1, 1]
     * y: [-1, 1]
     * omega: [-1, 1]
     */
    public void set(double x, double y, double omega) {
        if(isFieldCentric) {
            double angleDiff = Math.atan2(y, x) - getGyroAngle(); //difference between input angle and gyro angle gives desired field relative angle
            double r = Math.sqrt(x*x + y*y); //magnitude of translation vector
            x = r * Math.cos(angleDiff);
            y = r * Math.sin(angleDiff);
        }
        
        //Repeated equations
        double a = omega * Constants.Drive.WIDTH/2;
        double b = omega * Constants.Drive.LENGTH/2;

        //The addition of the movement and rotational vector
        Translation2d t0 = new Translation2d(x-b, y-a);
        Translation2d t1 = new Translation2d(x+b, y-a);
        Translation2d t2 = new Translation2d(x+b, y+a);
        Translation2d t3 = new Translation2d(x-b, y+a);

        //convert to polar
        SwerveState[] setStates = new SwerveState[] {
            SwerveState.fromTranslation2d(t0),
            SwerveState.fromTranslation2d(t1),
            SwerveState.fromTranslation2d(t2),
            SwerveState.fromTranslation2d(t3)
        };

        setStates = SwerveState.normalize(setStates);

        // for(int i = 0; i < 4; i++) {
        //     Logger.getInstance().processInputs("Drive/SetSwerveStates", setStates);
        // }

        set(setStates);
    }

    public void set(SwerveState[] states) {
        for(int i = 0; i < modules.length; i++) {
            modules[i].set(states[i]);
        }
    }

    public double getGyroAngle() {
        return inputs.yaw;
    }

    public static Drive instance;

    public static Drive getInstance() {
        if(instance == null) instance = new Drive();
        return instance;
    }

}
