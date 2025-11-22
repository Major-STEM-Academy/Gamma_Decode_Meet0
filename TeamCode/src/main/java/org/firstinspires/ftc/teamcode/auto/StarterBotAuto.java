package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
public class StarterBotAuto extends LinearOpMode {

    private DcMotorEx hogback;
    private CRServo flyWheell;
    private CRServo flyWheelr;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private DcMotorEx leftBackDrive;


    // hogback velocities
    final double LAUNCHER_VELOCITY = 1750;
    final double LAUNCHER_MIN_VELOCITY = 1650;

    // Feed timing
    final double FEED_TIME = 0.8;
    final double STOP_SPEED = 0.0;

    int shotsToFire = 3;

    enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime feederTimer = new ElapsedTime();

    // ******** ENCODER CONSTANTS ********
    static final double TICKS_PER_ROTATION = 560;   // GoBilda 5202/5203 motors
    static final double ROTATIONS_TO_DRIVE = 6;     // you said 6 rotations
    static final double TICKS_TO_DRIVE = TICKS_PER_ROTATION * ROTATIONS_TO_DRIVE;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware map .
        hogback = hardwareMap.get(DcMotorEx.class, "hogback");
        flyWheell = hardwareMap.get(CRServo.class, "flyWheell");
        flyWheelr = hardwareMap.get(CRServo.class, "flyWheelr");

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "motorbr");

        flyWheell.setPower(0);
        flyWheell.setPower(0);

        hogback.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset encoders
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initialized - READY");
        telemetry.update();

        waitForStart();

        // ************** DRIVE FORWARD 6 ROTATIONS **************
        driveForwardWithEncoders(0.4, (int) TICKS_TO_DRIVE);

        // **************** TURN LEFT 45° ****************
        turnLeft45(0.4);

        // **************** START SHOOTING ****************
        launchState = LaunchState.SPIN_UP;

        while (opModeIsActive()) {

            switch (launchState) {

                case SPIN_UP:
                    hogback.setVelocity(LAUNCHER_VELOCITY);

                    if (hogback.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                        launchState = LaunchState.LAUNCH;

                        flyWheell.setPower(1);
                        flyWheelr.setPower(1);
                        feederTimer.reset();
                    }
                    break;

                case LAUNCH:
                    if (feederTimer.seconds() > FEED_TIME) {

                        flyWheell.setPower(0);
                        flyWheelr.setPower(0);

                        shotsToFire--;

                        if (shotsToFire <= 0) {
                            hogback.setPower(0);
                            launchState = LaunchState.IDLE;
                        }
                    }
                    break;

                case IDLE:
                    hogback.setPower(0);
                    break;
            }

            telemetry.addData("Launcher Velocity", hogback.getVelocity());
            telemetry.addData("State", launchState);
            telemetry.update();
        }
    }

    // ************** ENCODER DRIVE FORWARD **************
    public void driveForwardWithEncoders(double power, int ticks) {
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(ticks);
        leftFrontDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        while (opModeIsActive() && leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            telemetry.addData("Driving", "Forward...");
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // ************** TURN LEFT 45° (ENCODER TURN) **************
    public void turnLeft45(double power) {

        // 45° = 1/8 of a full rotation (depends on robot)
        int turnTicks = (int)(TICKS_PER_ROTATION * 1.2);  // tweak as needed

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(-turnTicks); // left wheel backward
        rightFrontDrive.setTargetPosition(turnTicks); // right wheel forward
        leftBackDrive.setTargetPosition(-turnTicks); // left wheel backward
        rightBackDrive.setTargetPosition(turnTicks); // right wheel forward

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
