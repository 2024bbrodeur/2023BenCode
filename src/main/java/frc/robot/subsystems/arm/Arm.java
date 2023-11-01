package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    ArmIO io;
    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
    }

    public Arm() {
        io = new ArmIOTalonFX();
    }

    public void setPosition(Translation2d position) {
        io.setExtendPosition(Math.sqrt(position.getX()*position.getX() + position.getY()*position.getY()));
        io.setPivotPosition(Math.atan2(position.getY(), position.getX()));
    }

    public void setVelocity(Translation2d velocity) {
        io.setExtendVelocity(Math.sqrt(velocity.getX()*velocity.getX() + velocity.getY()*velocity.getY()));
        io.setPivotVelocity(Math.atan2(velocity.getY(), velocity.getX()));
    }

    public void setExtendPosition(double position) {
        io.setExtendPosition(position);
    }

    public void setExtendVelocity(double velocity) {
        io.setExtendVelocity(velocity);
    }

    public void setPivotPosition(double position) {
        io.setPivotPosition(position);
    }

    public void setPivotVelocity(double velocity) {
        io.setPivotVelocity(velocity);
    }

    private static Arm instance;

    public static Arm getInstance() {
        if(instance == null) instance = new Arm();
        return instance;
    }

}
