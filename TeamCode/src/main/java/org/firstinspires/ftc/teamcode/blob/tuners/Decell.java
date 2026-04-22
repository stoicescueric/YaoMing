package org.firstinspires.ftc.teamcode.blob.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import org.firstinspires.ftc.teamcode.blob.driveTrain.Blob;

@TeleOp(name = "Tuner: Predictive Deceleration", group = "Blob Tuning")
public class Decell extends LinearOpMode {

    Blob blob;
    ElapsedTime timer = new ElapsedTime();

    // State Machine
    enum State { IDLE, ACCEL_FORWARD, BRAKE_FORWARD, ACCEL_STRAFE, BRAKE_STRAFE }
    State state = State.IDLE;

    // Tracking variables
    double startPos = 0;
    double brakeVelocity = 0;
    GamePadController gg;

    // Results
    double calculatedXDecel = 0;
    double calculatedYDecel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        blob = new Blob(hardwareMap, Blob.State.DRIVE);

        telemetry.addLine("== Predictive Decel Tuner ==");
        telemetry.addLine("Ensure you have at least 6 feet of clear space.");
        telemetry.addLine("Press [A] to test Forward Deceleration (X)");
        telemetry.addLine("Press [B] to test Strafe Deceleration (Y)");
        telemetry.addLine("Press [X] to EMERGENCY STOP");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gg.update();
            blob.odo.update();

            if (gg.x()) {
                state = State.IDLE;
            }

            switch (state) {
                case IDLE:
                    blob.setTargetVector(0, 0, 0);

                    if (gg.aOnce()) {
                        state = State.ACCEL_FORWARD;
                        timer.reset();
                    } else if (gg.bOnce()) {
                        state = State.ACCEL_STRAFE;
                        timer.reset();
                    }
                    break;


                case ACCEL_FORWARD:
                    blob.setTargetVector(0, 1.0, 0);

                    if (timer.seconds() > 1.0) {
                        brakeVelocity = Math.abs(blob.odo.getVelX());
                        startPos = blob.odo.getX();

                        blob.setTargetVector(0, 0, 0); // BRAKE!
                        state = State.BRAKE_FORWARD;
                    }
                    break;

                case BRAKE_FORWARD:
                    blob.setTargetVector(0, 0, 0);

                    if (Math.abs(blob.odo.getVelX()) < 0.5) {
                        double distance = Math.abs(blob.odo.getX() - startPos);
                        calculatedXDecel = (brakeVelocity * brakeVelocity) / (2.0 * distance);
                        state = State.IDLE;
                    }
                    break;
                case ACCEL_STRAFE:
                    blob.setTargetVector(1.0, 0, 0);

                    if (timer.seconds() > 1.2) {
                        brakeVelocity = Math.abs(blob.odo.getVelY());
                        startPos = blob.odo.getY();

                        blob.setTargetVector(0, 0, 0);
                        state = State.BRAKE_STRAFE;
                    }
                    break;

                case BRAKE_STRAFE:
                    blob.setTargetVector(0, 0, 0);

                    if (Math.abs(blob.odo.getVelY()) < 0.5) {
                        double distance = Math.abs(blob.odo.getY() - startPos);
                        calculatedYDecel = (brakeVelocity * brakeVelocity) / (2.0 * distance);
                        state = State.IDLE;
                    }
                    break;
            }
            telemetry.addData("Current State", state);
            telemetry.addLine();
            telemetry.addData("Calculated X Decel (Forward)", "%.2f", calculatedXDecel);
            telemetry.addData("Calculated Y Decel (Strafe)", "%.2f", calculatedYDecel);
            telemetry.addLine();
            telemetry.addLine("Run each test 3-4 times and average the results.");
            telemetry.addLine("Then copy them into BlobConstants.java!");
            telemetry.update();
        }
    }
}