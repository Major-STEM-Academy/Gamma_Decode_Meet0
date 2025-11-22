package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
public class StarterBotAuto extends LinearOpMode {

    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;

    // Launcher speeds
    final double LAUNCHER_VELOCITY = 1750;      // max spin-up
    final double    LAUNCHER_MIN_VELOCITY = 1650;  // must reach this before feeding

    // Feed timing
    final double FEED_TIME = 0.5;
    final double STOP_SPEED = 0.0;

    int shotsToFire = 1;  // 1 burst of 3 balls

    enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime feederTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware initialization
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addLine("Initialized - READY");
        telemetry.update();

        waitForStart();

        // *********** MOVE TO GOAL ***********
        driveForward(0.4, 1.2);  // power, time (seconds)
        sleep(300); // settle the robot

        // *********** START SHOOTER ***********
        launchState = LaunchState.SPIN_UP;

        while (opModeIsActive()) {

            switch (launchState) {

                case SPIN_UP:
                    launcher.setVelocity(LAUNCHER_VELOCITY);

                    if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                        launchState = LaunchState.LAUNCH;

                        leftFeeder.setPower(1);
                        rightFeeder.setPower(1);
                        feederTimer.reset();
                    }
                    break;

                case LAUNCH:
                    if (feederTimer.seconds() > FEED_TIME) {

                        leftFeeder.setPower(STOP_SPEED);
                        rightFeeder.setPower(STOP_SPEED);

                        shotsToFire--;

                        if (shotsToFire <= 0) {
                            launcher.setPower(0);
                            launchState = LaunchState.IDLE;
                        }
                    }
                    break;

                case IDLE:
                    launcher.setPower(0);
                    break;
            }

            telemetry.addData("Launcher Velocity", launcher.getVelocity());
            telemetry.addData("State", launchState);
            telemetry.update();
        }
    }

    // *********** DRIVE FORWARD FUNCTION ***********
    public void driveForward(double power, double timeSeconds) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep((long)(timeSeconds * 1000));
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
