package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.util.SwerveState;

public class Module {
    
    public int index;

    private ModuleIO io;
    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(ModuleIO io, int index) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module"+index, inputs);
    }

    public void set(SwerveModuleState state) {
        set(state.speedMetersPerSecond / Constants.Drive.MAX_MOVE_VELOCITY_MPS, NRUnits.constrainRad(state.angle.getRadians()));
    }

    public void set(SwerveState state) {
        set(state.move, state.turn);
    }

    public void set(double move, double turn) {
        set(move, turn, true);
    }

    public void set(double move, double turn, boolean optimizeTurn) {
        if(move == 0 && optimizeTurn){
            io.setMove(0);
            return;
        }
        double lastTurn = inputs.turnPositionRotor;

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = NRUnits.constrainRad((lastTurn + angleChange));

        io.setTurn(nextPos);
        io.setMove(move * multiplier);
    }

    private int multiplier;

    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn); //Gets the two potential angles we could go to

        // Calculate the distance between those and the last angle the module was at
        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        // If the original distance is less, we want to go there
        if(originalDistance <= oppositeDistance){
            // moveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            // moveConfigurator.apply(moveConfig);
            multiplier = -1;
            return potAngles[0];
        }
        else{ //If we want to go to the opposite of the desired angle, we have to tell the motor to move "backwards"
            // moveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            // moveConfigurator.apply(moveConfig);
            multiplier = 1;
            return potAngles[1];
        } 
    }

    // Find the two angles we could potentially go to
    public double[] potentialAngles(double angle){
        //Constrain the variable to desired domain
        angle = NRUnits.constrainRad(angle);

        //Figure out the opposite angle
        double oppositeAngle = angle + Constants.TAU/2;

        //Constrain the opposite angle
        oppositeAngle = NRUnits.constrainRad(oppositeAngle);

        //Put them into a size 2 array
        double[] angles = {angle, oppositeAngle};

        return angles;
    }

    public double findAngleChange(double turn, double lastTurn){
        double distance = turn - lastTurn;
        //double sign = Math.signum(distance);   //Either 1 or -1 -> represents positive or negative

        if(Math.abs(turn - (lastTurn + Constants.TAU)) < Math.abs(distance)){
            // If this is true, it means that lastTurn is in the negatives and is trying to reach a positive, meaning that it must move positive
            distance = turn - (lastTurn + Constants.TAU);
            //sign = +1;
        }

        if(Math.abs(turn+Constants.TAU - (lastTurn)) < Math.abs(distance)){
            // If this is true, it means that turn is in the negatives and lastTurn is trying to reach a negative, meaning that you must move negative 
            distance = turn+Constants.TAU - lastTurn;
            //sign = -1;
        }

        return distance;
    }

    public double findDistance(double turn, double lastTurn){
        double distance = Math.min(Math.abs(turn - lastTurn), Math.abs(turn+Constants.TAU - lastTurn));
        distance = Math.min(distance, Math.abs(turn - (lastTurn+Constants.TAU)));

        return distance;
    }

}