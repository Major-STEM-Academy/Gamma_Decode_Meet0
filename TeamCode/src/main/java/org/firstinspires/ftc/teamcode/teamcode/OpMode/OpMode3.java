package org.firstinspires.ftc.teamcode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

//
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;
    private boolean bMoveUpSlider = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Put initialization blocks here.

        waitForStart();
        while (opModeIsActive()) {
            double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -1.0 * gamepad1.left_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;
            robot.setDrivePower(flPower, frPower, blPower, brPower);

            // robot.setDrivePower(vertical + turn + horizontal, vertical
            // - turn - horizontal, vertical + turn - horizontal, vertical - turn + horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));


//make sure one of the directions is correct/reversed
            //misumi slide start
            // To use continuous servo:
            // 1) change to continuous  rotation servo by using servo programmer
            // 2) on driver station, configured it as continuous rotation servo
            // 3) in Java code, use class "CRServo"
            if (gamepad2.left_stick_y > 0.7) { //if joystick moved up
                //misumi slide extends

            } else if (gamepad2.left_stick_y < -0.7) {// if joystick moves down
                // misumi slide retract

            } else { //stop
            }


//grabberX on horizontal (misumi slide)
            if (gamepad2.left_trigger > 0.5) {

            } else if (gamepad2.left_bumper) {

            } else {
                //  robot.grabberX.setPosition(0);
            }


//tilt servo #2
            /*
            if (gamepad2.left_stick_y > 0.7) {
                robot.grabberYtilt.setPosition(0.0);

            } else if (gamepad2.left_stick_y < -0.7) {
                robot.grabberYtilt.setPosition(0.6);
            }

*/



    /*void liftHexArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();
        {

        }



     */

     /*

        robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftHex.setTargetPosition(ticks);
        robot.liftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftHex.setPower(power);
        while(opModeIsActive()
                && robot.liftHex.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();





    private void TiltLiftOne ( double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
        double liftPowerBegin, int liftTimeMs, double liftPowerEnd){
            //tilt the lift to be upright
            robot.liftHex.setPower(crankPowerBegin);   //set motor power
            sleep(crankTimeMs);          // let motor run for some time seconds.
            robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

            // Extend liftArm
            robot.liftArm.setPower(liftPowerBegin);
            sleep(liftTimeMs);             // let motor run for some time seconds.
            robot.liftArm.setPower(liftPowerEnd);

        }

        }


      */

        }

    }
}