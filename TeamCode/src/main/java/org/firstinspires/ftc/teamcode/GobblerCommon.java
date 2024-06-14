package org.firstinspires.ftc.teamcode;

import java.util.Collections;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.ArrayList;

@Config
public class GobblerCommon {
    public enum IntakeOptions {
        STOP, IN, OUT
    }

    // Hardware
    private final HardwareMap hardwareMap;
    private Servo launcher;
    private Servo wrist;
    private AnalogInput armAngle;
    private CRServo arm;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private CRServo intakeLeft;
    private CRServo intakeRight;

    // Drive
    private double rot;
    private double vx;
    private double vy;

    // Arm
    public static double ARM_D = 20;
    public static double ARM_P = 3.8;
    public static double DEGREES_MAX = 120;
    public static double VOLTAGE_MAX = 0.6;
    public static double VOLTAGE_MIN = 3.3;
    private List<Double> oldArmPositions;
    private List<Double> speedFactors;
    private double armError;
    private double armPosition;
    private double armPower = 0;
    private double armSpeed = 0;
    private double armTargetPosition;
    private double armVoltage;
    private double dPart = 0;
    private double limitedArmPower = 0;
    private double limitedArmTargetPosition;
    private double pPart = 0;

    // Wrist
    private double wristPosition = 0;

    // Intake
    private IntakeOptions rightIntakeOption = IntakeOptions.STOP;
    private IntakeOptions leftIntakeOption = IntakeOptions.STOP;


    public GobblerCommon(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void run(boolean isStopRequested) {
        runDrive();
        runArm(isStopRequested);
        runIntake();
        runWrist();
    }

  /*
   * drive takes vx, vy, and rot and it turns them into
   motor velocities for each of the four wheels
   */

    public void setVx(double value) {
        this.vx = value;
    }

    public void setVy(double value) {
        this.vy = value;
    }

    public void setRot(double value) {
        this.rot = value;
    }

    private void runDrive() {
        double wheel1;
        double wheel2;
        double wheel3;
        double wheel4;

        wheel1 = vx + vy + rot;
        wheel2 = (vx - vy) - rot;
        wheel3 = (vx - vy) + rot;
        wheel4 = (vx + vy) - rot;
        frontLeft.setVelocity(wheel1);
        frontRight.setVelocity(wheel2);
        backLeft.setVelocity(wheel3);
        backRight.setVelocity(wheel4);
    }

  /*
   * arm takes arm target position and makes the motor go there. I has a
   control system to make the arm slow down before it reaches its destination.
   */

    private void runArm(boolean isStopRequested) {
        if (!isStopRequested) {
            armVoltage = armAngle.getVoltage();
        }
        armPosition = armVoltageToDegrees(armVoltage);
        limitedArmTargetPosition = armTargetPosition;

        oldArmPositions.add(armPosition);
        // Remove oldest item from list
        oldArmPositions.remove(0);
        armSpeed = 0;

        for (int i = 0; i < oldArmPositions.size(); i += 1) {
            armSpeed += oldArmPositions.get(i) * speedFactors.get(i);
        }
        armError = limitedArmTargetPosition - armPosition;
        pPart = ARM_P * armError;
        dPart = Math.abs(armError) >= 10 ? ARM_D * armSpeed : 0;
        armPower = Math.min(Math.max(pPart - dPart, -100), 100);
        limitedArmPower = Math.min(Math.max(armPower, limitedArmPower - 10), limitedArmPower + 10);
        arm.setPower(limitedArmPower / 100);
    }

    public void moveArm(double amount) {
        armTargetPosition = Math.min(Math.max(armTargetPosition - 1 * square(amount), 0), DEGREES_MAX);
    }

    public void armToPickupPosition() {
        armTargetPosition = 0;
    }

    public void armToDrivePosition() {
        armTargetPosition = 5;
    }

    public void armToHighDropPosition() {
        armTargetPosition = 112;
    }

    public void armToLowDropPosition() {
        armTargetPosition = 120;
    }

    public void armToPrepareToSuspendPosition() {
        armTargetPosition = 95;
    }

    public void armToSuspendPosition() {
        armTargetPosition = 60;
    }

    public double getArmTargetPosition() {
        return armTargetPosition;
    }

    public double getArmVoltage() {
        return armVoltage;
    }

    public double getArmPosition() {
        return armPosition;
    }

    public double getArmPower() {
        return limitedArmPower;
    }

    private double armVoltageToDegrees(double voltage) {
        return (DEGREES_MAX / (VOLTAGE_MAX - VOLTAGE_MIN)) * (voltage - VOLTAGE_MIN);
    }

    /*
     * Takes intake left and right and uses them to set power to the intake
     */

    public void setLeftIntake(IntakeOptions option) {
        this.leftIntakeOption = option;
    }

    public void setRightIntake(IntakeOptions option) {
        this.rightIntakeOption = option;
    }

    public String getLeftIntakeState() {
        return leftIntakeOption.toString();
    }

    public String getRightIntakeState() {
        return rightIntakeOption.toString();
    }

    private void runIntake() {
        switch (leftIntakeOption) {
            case STOP:
                intakeLeft.setPower(0);
                break;
            case IN:
                intakeLeft.setPower(-1);
                break;
            case OUT:
                intakeLeft.setPower(1);
                break;
        }
        switch (rightIntakeOption) {
            case STOP:
                intakeRight.setPower(0);
                break;
            case IN:
                intakeRight.setPower(1);
                break;
            case OUT:
                intakeRight.setPower(-1);
                break;
        }
    }

    /**
     * Launcher
     */
    public void launchDrone() {
        launcher.setPosition(1);
    }
    public void resetLauncher() {
        launcher.setPosition(0);
    }

    /**
     * Wrist
     */
    public void moveWrist(double amount) {
        wristPosition = Math.min(Math.max(wristPosition + square(amount) / 200, 0), 1);
    }

    public void wristToPickupPosition(){
        wristPosition = 0.92;
    }

    public void wristToDrivePosition(){
        wristPosition = 0;
    }

    public void wristToLowDropPosition(){
        wristPosition = 0.21;
    }

    public void wristToHighDropPosition(){
        wristPosition = 0;
    }

    public double getWristPosition() {
        return wristPosition;
    }

    private void runWrist() {
        wrist.setPosition(wristPosition);
    }

    /**
     * Describe this function...
     */
    public void initialize() {
        // Bulk Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init hardware
        launcher = hardwareMap.get(Servo.class, "Launcher");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        armAngle = hardwareMap.get(AnalogInput.class, "Arm Angle");
        arm = hardwareMap.get(CRServo.class, "Arm");
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");
        intakeLeft = hardwareMap.get(CRServo.class, "Intake Left");
        intakeRight = hardwareMap.get(CRServo.class, "Intake Right");

        // Arm
        armVoltage = armAngle.getVoltage();
        armPosition = armVoltageToDegrees(armVoltage);
        armTargetPosition = armPosition;
        limitedArmTargetPosition = armTargetPosition;
        speedFactors = Arrays.asList(-0.083, -0.059, -0.035, -0.012, 0.012, 0.035, 0.059, 0.083);
        oldArmPositions = new ArrayList<>(Collections.nCopies(speedFactors.size(), 0.0));

        // Config Motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Enable Encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}
