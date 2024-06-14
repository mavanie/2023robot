package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Green Gobbler Studio")
@Config
public class GreenGobbler extends LinearOpMode {

    private GobblerCommon common;
    private FtcDashboard dashboard;

    double rot;
    double vx;
    double vy;
    public static int FAST_FORWARD_FACTOR = 3000;
    public static int SLOW_FORWARD_FACTOR = 740;
    public static int FAST_ROTATION_FACTOR = 2000;
    public static int SLOW_ROTATION_FACTOR = 1000;
    public static int FAST_SIDEWAYS_FACTOR = 1500;
    public static int SLOW_SIDEWAYS_FACTOR = 700;

    Gamepad g1;
    Gamepad g2;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run(isStopRequested());
                sendTelemetry();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void initialize() {
        common = new GobblerCommon(hardwareMap);
        common.initialize();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // Gamepad
        g1 = new Gamepad();
        g2 = new Gamepad();
    }

    /**
     * Describe this function...
     */
    private void controls() {
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        if (g2.a) {
            common.armToPickupPosition();
            common.wristToPickupPosition();
        }
        if (g2.b) {
            common.armToDrivePosition();
            common.wristToDrivePosition();
        }
        if (g2.y) {
            common.armToLowDropPosition();
            common.wristToLowDropPosition();
        }
        if (g2.x) {
            common.armToHighDropPosition();
            common.wristToHighDropPosition();
        }
        if (g2.dpad_up) {
            common.armToPrepareToSuspendPosition();
            common.wristToDrivePosition();
        }
        if (g2.dpad_down) {
            common.armToSuspendPosition();
            common.wristToDrivePosition();
        }
        if (g1.start) {
            common.resetLauncher();
        } else if (g1.back) {
            common.launchDrone();
        }
        common.moveArm(gamepad2.left_stick_y);

        boolean movingFastSideways = g1.a && Math.abs(g1.left_stick_x) > Math.abs(g1.left_stick_y);
        boolean movingFastForward = g1.a && Math.abs(g1.left_stick_y) > Math.abs(g1.left_stick_x);
        vx = movingFastSideways ? 0 : -GobblerCommon.square(g1.left_stick_y) * (g1.a ? FAST_FORWARD_FACTOR : SLOW_FORWARD_FACTOR);
        vy = movingFastForward ? 0 : GobblerCommon.square(g1.left_stick_x) * (g1.a ? FAST_SIDEWAYS_FACTOR : SLOW_SIDEWAYS_FACTOR);
        rot = GobblerCommon.square(g1.right_trigger - g1.left_trigger) * (g1.a ? FAST_ROTATION_FACTOR : SLOW_ROTATION_FACTOR);

        common.setVx(vx);
        common.setVy(vy);
        common.setRot(rot);

        if (g2.right_trigger >= 0.5) {
            common.setRightIntake(GobblerCommon.IntakeOptions.IN);
        } else if (g2.right_bumper) {
            common.setRightIntake(GobblerCommon.IntakeOptions.OUT);
        } else {
            common.setRightIntake(GobblerCommon.IntakeOptions.STOP);

        }
        if (g2.left_bumper) {
            common.setLeftIntake(GobblerCommon.IntakeOptions.OUT);
        } else if (g2.left_trigger >= 0.5) {
            common.setLeftIntake(GobblerCommon.IntakeOptions.IN);
        } else {
            common.setLeftIntake(GobblerCommon.IntakeOptions.STOP);
        }
        common.moveWrist(g2.right_stick_y);

    }

    /**
     * Describe this function...
     */
    private void sendTelemetry() {

        telemetry.addData("Intake Left", common.getLeftIntakeState());
        telemetry.addData("Intake Right", common.getRightIntakeState());
        telemetry.addData("Potentiometer", "%.2f", common.getArmVoltage());
        telemetry.addData("Arm Position", "%.1f", common.getArmPosition());
        telemetry.addData("Arm Target Position", "%.1f", common.getArmTargetPosition());
        // telemetry.addData("Error", "%.1f", Error2);
        // telemetry.addData("Arm Speed", "%.1f", Arm_Speed);
        // telemetry.addData("P part", "%.1f", P_Part);
        // telemetry.addData("D part", "%.1f", D_Part);
        telemetry.addData("Arm Power", "%.1f", common.getArmPower());
        telemetry.addData("Wrist position", "%.1f", common.getWristPosition());
        telemetry.update();
    }

}
