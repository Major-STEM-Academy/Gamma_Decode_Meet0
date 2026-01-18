package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
public class StarterBotAuto extends LinearOpMode {

    // Drive motors
    private DcMotor motorfl, motorfr, motorbl, motorbr;

    // Shooter motors/servos
    private DcMotor hogback;
    private CRServo flyWheell, flyWheelr;

    // ----------- Shooter Constants -----------
    final double FEED_TIME_SECONDS = 0.5;
    final double BETWEEN_SHOTS_DELAY = 0.6;
    final int NUM_SHOTS = 3;

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----------- Hardware Map -----------
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");

        hogback = hardwareMap.get(DcMotor.class, "hogback");
        flyWheell = hardwareMap.get(CRServo.class, "flyWheell");
        flyWheelr = hardwareMap.get(CRServo.class, "flyWheelr");

        // ----------- Motor Directions -----------
        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
        motorfr.setDirection(DcMotor.Direction.FORWARD);
        motorbr.setDirection(DcMotor.Direction.FORWARD);

        hogback.setDirection(DcMotor.Direction.REVERSE);
        flyWheell.setDirection(CRServo.Direction.FORWARD);
        flyWheelr.setDirection(CRServo.Direction.REVERSE);

        // ----------- Initial States -----------
        motorfl.setPower(0);
        motorfr.setPower(0);
        motorbl.setPower(0);
        motorbr.setPower(0);

        flyWheell.setPower(0);
        flyWheelr.setPower(0);
        hogback.setPower(0);

        telemetry.addLine("Initialized - READY");
        telemetry.update();

        waitForStart();

        // ----------- Shoot Routine (3 Balls) -----------

        // Spin up shooter
        hogback.setPower(FULL_SPEED);
        flyWheell.setPower(FULL_SPEED);
        flyWheelr.setPower(FULL_SPEED);

        // Allow shooter to reach speed
        sleep(800);

        // Shoot 3 balls
        for (int i = 0; i < NUM_SHOTS && opModeIsActive(); i++) {
            sleep((long) (FEED_TIME_SECONDS * 1000));
            sleep((long) (BETWEEN_SHOTS_DELAY * 1000));
        }

        // Stop shooter
        flyWheell.setPower(STOP_SPEED);
        flyWheelr.setPower(STOP_SPEED);
        hogback.setPower(STOP_SPEED);

        // ----------- Drive Backward -----------
        motorfl.setPower(-0.4);
        motorfr.setPower(-0.4);
        motorbl.setPower(-0.4);
        motorbr.setPower(-0.4);

        sleep(4000);

        // Stop drive motors
        motorfl.setPower(0);
        motorfr.setPower(0);
        motorbl.setPower(0);
        motorbr.setPower(0);

        // Keep opmode alive
        while (opModeIsActive()) {
            idle();
        }
    }
}
