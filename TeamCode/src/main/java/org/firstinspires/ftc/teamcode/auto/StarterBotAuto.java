package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
public class StarterBotAuto extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotorEx hogback;
    private CRServo flyWheell;
    private CRServo flyWheelr;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private DcMotorEx leftBackDrive;

    // Shooter velocities
    final double LAUNCHER_VELOCITY = 1750;   // hogback
    final double LAUNCHER_MIN_VELOCITY = 1650;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        LAUNCHED
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime feederTimer = new ElapsedTime();

    final double FEED_TIME_SECONDS = 0.5;
    final double FULL_SPEED = 1.0;
    final double STOP_SPEED = 0.0;

    int shotsFired = 0;
    int maxShots = 1;

    private void updateLauncher(boolean shotRequested) {
        telemetry.addData("Current State (in update)", launchState); // Added for debug

        switch (launchState) {

            case IDLE: //only request shot if there hasn't been max # of shots yet
                if (shotRequested && shotsFired < maxShots) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                //set velocity to target velocity
                hogback.setVelocity(LAUNCHER_VELOCITY);
                //if velocity is at target range then go to launch
                if (hogback.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                //start running feeders
                flyWheell.setPower(FULL_SPEED);
                flyWheelr.setPower(FULL_SPEED);
                feederTimer.reset(); //start time for feeders
                launchState = LaunchState.LAUNCHING; //go to next state
                break;

            case LAUNCHING:
                //after feed time, go to next state and add shotsFired count
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    flyWheell.setPower(STOP_SPEED);
                    flyWheelr.setPower(STOP_SPEED);
                    launchState = LaunchState.LAUNCHED;
                    shotsFired++; // IMPORTANT
                }
                break;

            case LAUNCHED:
                // check if shooter velocity is below minimum required velocity
                //if velocity<min velocity, go back to increase velocity
                //else if hasn't reached max # of shots yet, then loop again
                //else, stay in launched state
                if(hogback.getVelocity()< LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.SPIN_UP;
                }else if (shotsFired < maxShots) {
                    launchState = LaunchState.IDLE;
                }else{

                }break;

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {




        // Hardware map
        hogback = hardwareMap.get(DcMotorEx.class, "hogback");
        flyWheell = hardwareMap.get(CRServo.class, "flyWheell");
        flyWheelr = hardwareMap.get(CRServo.class, "flyWheelr");

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "motorbr");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        flyWheell.setDirection(CRServo.Direction.FORWARD);
        flyWheelr.setDirection(CRServo.Direction.REVERSE);

        // Flywheels OFF at start
        flyWheell.setPower(0);
        flyWheelr.setPower(0);

        hogback.setDirection(DcMotorEx.Direction.REVERSE);

        // Drive motor directions
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addLine("Initialized - READY");
        telemetry.update();

        waitForStart();

/*----------------LIMELIGHT----------------------------
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        if (result != null && result.isValid()) {
           Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }
        }

 */
        // --------------------------- SHOOTING ------------------------------

// ensure launcher motor is set to encoder in order to control velocity

        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shotsFired = 0;
        launchState = LaunchState.IDLE;

// Loop until max # of balls are shot
        while (opModeIsActive() && shotsFired < maxShots) {

            // Always request a shot when idle. idle should also be able to go to spin_up
            boolean requestShot = (launchState == LaunchState.IDLE || launchState == LaunchState.LAUNCHED);

            updateLauncher(requestShot);

            telemetry.addData("Launch State", launchState);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Velocity", hogback.getVelocity());
            telemetry.update();
        }

// Stop shooter after max # of shots
        hogback.setVelocity(0);//stop shooter
        flyWheell.setPower(0);
        flyWheelr.setPower(0);



        // -------------------------- MOVE BACKWARD ----------------------------

            leftFrontDrive.setPower(-0.4);
            rightFrontDrive.setPower(-0.4);
            leftBackDrive.setPower(-0.4);
            rightBackDrive.setPower(-0.4);

            sleep(1000); // adjust distance here

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

        //---------------------------- IDLE -----------------------------
            while(opModeIsActive()){
                idle();
            }

        }

    }