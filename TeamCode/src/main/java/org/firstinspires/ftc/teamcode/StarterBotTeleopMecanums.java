package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {

    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = -1;

    final double HOGBACK_TARGET_VELOCITY = 1900;
    final double HOGBACK_MIN_VELOCITY = 1800;

    final double BUMPER_FEED_TIME = 0.55;
    final double BUMPER_FEED_POWER = -0.3;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx hogback;
    private CRServo leftFeeder, rightFeeder;

    ElapsedTime feederTimer = new ElapsedTime();

    int maxShots = 0;

    // 🔹 UPDATED STATE MACHINE
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        JOG_FEEDER   // ← NEW STATE
    }

    private LaunchState launchState;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorbr");
        hogback = hardwareMap.get(DcMotorEx.class, "hogback");
        leftFeeder = hardwareMap.get(CRServo.class, "flyWheell");
        rightFeeder = hardwareMap.get(CRServo.class, "flyWheelr");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hogback.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        hogback.setZeroPowerBehavior(BRAKE);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        hogback.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        mecanumDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        // Y starts hogback and keeps it running
        if (gamepad1.y) {
            hogback.setVelocity(HOGBACK_TARGET_VELOCITY);
        }

        // A stops hogback
        if (gamepad1.a) {
            hogback.setVelocity(STOP_SPEED);
        }

        // Right trigger fires 3 shots
        if (gamepad1.right_trigger > 0.5) {
            launchMultiple(3);
        }

        // Right bumper jogs feeders for 0.1s
        if (gamepad1.right_bumper && launchState == LaunchState.IDLE) {
            feederTimer.reset();
            leftFeeder.setPower(BUMPER_FEED_POWER);
            rightFeeder.setPower(BUMPER_FEED_POWER);
            launchState = LaunchState.JOG_FEEDER;
        }

        launch(maxShots > 0);

        telemetry.addData("State", launchState);
        telemetry.addData("Hogback Velocity", hogback.getVelocity());
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate),
                1
        );

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {

            case IDLE:
                if (shotRequested && maxShots > 0) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                hogback.setVelocity(HOGBACK_TARGET_VELOCITY);
                if (hogback.getVelocity() > HOGBACK_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    maxShots--;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = (maxShots > 0)
                            ? LaunchState.SPIN_UP
                            : LaunchState.IDLE;
                }
                break;

            // 🆕 FEEDER JOG HANDLER
            case JOG_FEEDER:
                if (feederTimer.seconds() >= BUMPER_FEED_TIME) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public void launchMultiple(int numberOfShots) {
        if (launchState == LaunchState.IDLE) {
            maxShots = numberOfShots;
            launchState = LaunchState.SPIN_UP;
        }
    }
}
