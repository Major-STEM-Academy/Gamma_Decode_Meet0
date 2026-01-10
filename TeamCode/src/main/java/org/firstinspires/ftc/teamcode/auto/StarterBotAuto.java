package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
public class StarterBotAuto extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private Limelight3A limelight;

    private DcMotorEx hogback;
    private CRServo flyWheell;
    private CRServo flyWheelr;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;

    // ---------------- SHOOTER CONSTANTS ----------------
    final double LAUNCHER_VELOCITY = 1750;
    final double LAUNCHER_MIN_VELOCITY = 1650;

    final double FEED_TIME_SECONDS = 0.5;
    final double FULL_SPEED = 1.0;
    final double STOP_SPEED = 0.0;

    // ---------------- STATE MACHINE ----------------
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        LAUNCHED
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime feederTimer = new ElapsedTime();

    int shotsFired = 0;
    int maxShots = 1;

    // ---------------- LAUNCHER STATE MACHINE ----------------
    private void updateLauncher(boolean shotRequested) {

        telemetry.addData("Launcher State", launchState);

        switch (launchState) {

            case IDLE:
                if (shotRequested && shotsFired < maxShots) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                hogback.setVelocity(LAUNCHER_VELOCITY);
                if (hogback.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                flyWheell.setPower(FULL_SPEED);
                flyWheelr.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    flyWheell.setPower(STOP_SPEED);
                    flyWheelr.setPower(STOP_SPEED);
                    shotsFired++;
                    launchState = LaunchState.LAUNCHED;
                }
                break;

            case LAUNCHED:
                if (hogback.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.SPIN_UP;
                } else if (shotsFired < maxShots) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ---------------- MAIN AUTONOMOUS ----------------
    @Override
    public void runOpMode() throws InterruptedException {

        // ----------- Hardware Map -----------
        hogback = hardwareMap.get(DcMotorEx.class, "hogback");
        flyWheell = hardwareMap.get(CRServo.class, "flyWheell");
        flyWheelr = hardwareMap.get(CRServo.class, "flyWheelr");

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "motorbr");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        // ----------- Motor Directions -----------
        hogback.setDirection(DcMotorEx.Direction.REVERSE);

        flyWheell.setDirection(CRServo.Direction.FORWARD);
        flyWheelr.setDirection(CRServo.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // ----------- Initial States -----------
        flyWheell.setPower(0);
        flyWheelr.setPower(0);

        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initialized - READY");
        telemetry.update();

        waitForStart();

        // ---------------- SHOOT ----------------
        shotsFired = 0;
        launchState = LaunchState.IDLE;

        while (opModeIsActive() && shotsFired < maxShots) {

            boolean requestShot =
                    (launchState == LaunchState.IDLE || launchState == LaunchState.LAUNCHED);

            updateLauncher(requestShot);

            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Shooter Velocity", hogback.getVelocity());
            telemetry.update();
        }

        // Stop shooter
        hogback.setVelocity(0);
        flyWheell.setPower(0);
        flyWheelr.setPower(0);

        // ---------------- DRIVE BACKWARD ----------------
        leftFrontDrive.setPower(-0.4);
        rightFrontDrive.setPower(-0.4);
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(-0.4);

        sleep(1000);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // ---------------- END ----------------
        while (opModeIsActive()) {
            idle();
        }
    }
}
