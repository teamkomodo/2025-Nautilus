package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDGains;
import frc.robot.util.Util;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

public class IntakeSubsystem extends SubsystemBase {

    private final DoublePublisher intakeVelocityPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("intakeDutyCycle").publish();
    private final BooleanPublisher pieceIntakedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceIntaked").publish();
    private final BooleanPublisher pieceLoadedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceInShooter").publish();
    private final BooleanPublisher pieceIntakedSensorPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceIntakedSensor").publish();
    private final BooleanPublisher pieceLoadedSensorPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceInShooterSensor").publish();

    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeConfig;
    private final SparkMax intakeMotor2;
    private final SparkMaxConfig intakeConfig2;
    
    private final RelativeEncoder intakeEncoder;
    private final SparkClosedLoopController intakeController;

    public final DigitalInput noteIntakedSensor;
    public final DigitalInput noteLoadedSensor;

    public String currentCommand = "idle";

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    public boolean noteIntaked;
    public boolean noteLoaded;
    
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeConfig = new SparkMaxConfig();
        intakeMotor2 = new SparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeConfig2 = new SparkMaxConfig();

        noteIntakedSensor = new DigitalInput(1); //FIXME: find port number
        noteLoadedSensor = new DigitalInput(2); //FIXME: find port number
        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);

        intakeController = intakeMotor.getClosedLoopController();

        configMotors();
    }

    public void teleopInit() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        updateTelemetry();
        filterCurrent();
        checkSensors();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + intakeMotor.getOutputCurrent() * currentFilterConstant;
    }

    public void checkSensors() {
        boolean currentNoteIntaked = getNoteDetection(noteIntakedSensor);
        boolean currentNoteLoaded = getNoteDetection(noteLoadedSensor);

        if(currentNoteIntaked && getIntakeVelocity() > 0) {
            noteIntaked = true;
        }

        if(currentNoteLoaded && currentCommand == "idle") {
            noteIntaked = true;
            noteLoaded = true;
        }

        if(currentNoteLoaded && getIntakeVelocity() > 0 && currentCommand == "intake") {
            currentCommand = "idle";
            holdIntake();
            noteIntaked = true;
            noteLoaded = true;
        }
        
        if(!currentNoteIntaked && currentCommand == "eject") {
            noteLoaded = false;
            noteIntaked = false;
        }
    }

    public void updateTelemetry() {
        pieceIntakedPublisher.set(getPieceIntaked());
        pieceLoadedPublisher.set(getPieceLoaded());
        pieceIntakedSensorPublisher.set(noteIntakedSensor.get());
        pieceLoadedSensorPublisher.set(noteLoadedSensor.get());
        intakeVelocityPublisher.set(intakeMotor.getOutputCurrent());
    }

    public void setIntakeVelocity(double velocity) {
         intakeController.setReference(velocity, ControlType.kVelocity);
    }
    
    public void setIntakeDutyCycle(double dutyCycle) {
        intakeController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public Command intake() {
            currentCommand = "intake";
            return Commands.runEnd(() -> setIntakeDutyCycle(0.5), () -> {setIntakeDutyCycle(0);}).until(() -> getNoteDetection(noteLoadedSensor));
    }

    public Command eject() {
        currentCommand = "eject";
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setIntakeDutyCycle(-0.5)),
            Commands.waitSeconds(0.7),
            Commands.runOnce(() -> holdIntake()),
            Commands.runOnce(() -> {noteIntaked = false;}),
            Commands.runOnce(() -> {noteLoaded = false;})
        );
    }

    public void unIntake() {
        new SequentialCommandGroup(
            Commands.runOnce(() -> setIntakeDutyCycle(-0.5)),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> holdIntake())
        );
    }

    public void holdIntake() {
        currentCommand = "idle";
        intakeController.setReference(intakeEncoder.getPosition(), ControlType.kPosition);
    }

    public boolean getNoteDetection(DigitalInput beamBreak) {
        return beamBreak.get();
    }

    public boolean getPieceIntaked() {
        return noteIntaked;
    }

    public boolean getPieceLoaded() {
        return noteLoaded;
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    public void configMotors() {

        intakeConfig
        .inverted(true)
        .smartCurrentLimit(80);
        
        intakeConfig.closedLoop
        .pid(0, 1, 1);

        intakeConfig2
        .inverted(false)
        .smartCurrentLimit(80)
        .follow(INTAKE_MOTOR_1_ID, true);

    }
}