package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
public class StarterBotAuto extends LinearOpMode {

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


        // --------------------------- SHOOTING ------------------------------

        // Spin-up hogback using velocity
        hogback.setVelocity(LAUNCHER_VELOCITY);

        // spin up flywheels using power
        flyWheell.setPower(1.0);
        flyWheelr.setPower(1.0);

        sleep(1200); // Let wheels stabilize

        // Shoot 3 balls
        for (int i = 0; i < 3; i++) {

            // Feed ball
            hogback.setVelocity(LAUNCHER_VELOCITY);
            sleep(300);

            // pause between shots
            hogback.setVelocity(0);
            sleep(200);
        }

        // Stop shooter
        flyWheell.setPower(0);
        flyWheelr.setPower(0);
        hogback.setVelocity(0);


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
    }
}
