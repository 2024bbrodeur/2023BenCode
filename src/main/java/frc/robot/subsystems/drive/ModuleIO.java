package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import frc.robot.Constants;

public class ModuleIO {

    private int modIndex;

    private TalonFX moveMotor;
    private TalonFXConfiguration moveConfig;
    private TalonFXConfigurator moveConfigurator;

    private VelocityDutyCycle moveControl;

    private TalonFX turnMotor;
    private TalonFXConfiguration turnConfig;
    private TalonFXConfigurator turnConfigurator;

    private MotionMagicDutyCycle turnControl;

    private CANcoder turnSensor;
    private CANcoderConfiguration turnSensorConfig;
    private CANcoderConfigurator turnSensorConfigurator;

    private double turnSensorOffset;

    public ModuleIO(int modIndex) {
        this.modIndex = modIndex;

        switch(modIndex) {
            case 0:
                moveMotor = new TalonFX(Constants.Drive.FRONT_RIGHT_MOVE_PORT, "drivet");
                turnMotor = new TalonFX(Constants.Drive.FRONT_RIGHT_TURN_PORT, "drivet");
                turnSensor = new CANcoder(Constants.Drive.FRONT_RIGHT_SENSOR_PORT, "drivet");
                turnSensorOffset = Constants.Drive.FRONT_RIGHT_OFFSET_NU;
                break;
            case 1:
                moveMotor = new TalonFX(Constants.Drive.FRONT_LEFT_MOVE_PORT, "drivet");
                turnMotor = new TalonFX(Constants.Drive.FRONT_LEFT_TURN_PORT, "drivet");
                turnSensor = new CANcoder(Constants.Drive.FRONT_LEFT_SENSOR_PORT, "drivet");
                turnSensorOffset = Constants.Drive.FRONT_LEFT_OFFSET_NU;
                break;
            case 2:
                moveMotor = new TalonFX(Constants.Drive.BACK_LEFT_MOVE_PORT, "drivet");
                turnMotor = new TalonFX(Constants.Drive.BACK_LEFT_TURN_PORT, "drivet");
                turnSensor = new CANcoder(Constants.Drive.BACK_LEFT_SENSOR_PORT, "drivet"); 
                turnSensorOffset = Constants.Drive.BACK_LEFT_OFFSET_NU;
                break;
            case 3:
                moveMotor = new TalonFX(Constants.Drive.BACK_RIGHT_MOVE_PORT, "drivet");
                turnMotor = new TalonFX(Constants.Drive.BACK_RIGHT_TURN_PORT, "drivet");
                turnSensor = new CANcoder(Constants.Drive.BACK_RIGHT_SENSOR_PORT, "drivet");
                turnSensorOffset = Constants.Drive.BACK_RIGHT_OFFSET_NU;
                break;
        }

        moveConfig = new TalonFXConfiguration();
        turnConfig = new TalonFXConfiguration();
        turnSensorConfig = new CANcoderConfiguration();

        moveConfigurator = moveMotor.getConfigurator();
        turnConfigurator = turnMotor.getConfigurator();
        turnSensorConfigurator = turnSensor.getConfigurator();

        moveControl = new VelocityDutyCycle(0);
        moveControl.EnableFOC = true;
        moveControl.UpdateFreqHz = 50;

        turnControl = new MotionMagicDutyCycle(0);
        turnControl.UpdateFreqHz = 50;

        config();
    }

    private void config() {
        moveConfig.Audio.BeepOnBoot = true;
        moveConfig.ClosedLoopGeneral.ContinuousWrap = false;
        moveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        moveConfig.CurrentLimits.StatorCurrentLimit = Constants.Drive.MOVE_STATOR_CURRENT;
        moveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        moveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drive.MOVE_SUPPLY_CURRENT;
        moveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        moveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        moveConfig.Feedback.SensorToMechanismRatio = Constants.Drive.MOVE_GEAR_RATIO;
        moveConfig.FutureProofConfigs = true;
        moveConfig.MotorOutput.DutyCycleNeutralDeadband = 0;
        moveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        moveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        moveConfig.Slot0.kV = Constants.Drive.MOVE_KV;
        moveConfig.Slot0.kS = Constants.Drive.MOVE_KS;
        moveConfig.Slot0.kP = Constants.Drive.MOVE_KP;
        moveConfig.Slot0.kI = Constants.Drive.MOVE_KI;
        moveConfig.Slot0.kD = Constants.Drive.MOVE_KD;
        moveMotor.clearStickyFaults();
        moveMotor.setRotorPosition(0);
        moveConfigurator.apply(moveConfig);

        turnConfig.Audio.BeepOnBoot = true;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        turnConfig.CurrentLimits.StatorCurrentLimit = Constants.Drive.TURN_STATOR_CURRENT;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drive.TURN_SUPPLY_CURRENT;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfig.Feedback.SensorToMechanismRatio = Constants.Drive.TURN_GEAR_RATIO;
        turnConfig.FutureProofConfigs = true;
        turnConfig.MotorOutput.DutyCycleNeutralDeadband = 0;
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0.kV = Constants.Drive.TURN_KV;
        turnConfig.Slot0.kS = Constants.Drive.TURN_KS;
        turnConfig.Slot0.kP = Constants.Drive.TURN_KP;
        turnConfig.Slot0.kI = Constants.Drive.TURN_KI;
        turnConfig.Slot0.kD = Constants.Drive.TURN_KD;
        turnConfig.MotionMagic.MotionMagicAcceleration = Constants.Drive.TURN_ACCELERATION;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Drive.TURN_CRUISE_VELOCITY;
        turnConfig.MotionMagic.MotionMagicJerk = Constants.Drive.TURN_JERK;
        turnMotor.clearStickyFaults();
        turnMotor.setRotorPosition(0);
        turnConfigurator.apply(moveConfig);

        turnSensorConfig.FutureProofConfigs = true;
        turnSensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        turnSensorConfig.MagnetSensor.MagnetOffset = turnSensorOffset;
        turnSensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turnSensorConfigurator.apply(turnSensorConfig);
        turnMotor.setRotorPosition(turnSensor.getAbsolutePosition().getValue());
    }

    public void setAngle(double angle) {
        turnControl.Position = angle / Constants.TAU;
        turnMotor.setControl(turnControl);
    }

    public void setSpeed(double speed) {
        moveControl.Velocity = speed * Constants.Drive.MAX_MOVE_VELOCITY;
        moveMotor.setControl(moveControl);
    }

    public double getAngle() {
        return turnMotor.getRotorPosition().getValue() * Constants.TAU;
    }

    public double getSpeed() {
        return moveMotor.getRotorPosition().getValue() * Constants.TAU;
    }
    
}
