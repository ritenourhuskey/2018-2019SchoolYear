package org.firstinspires.ftc.teamcode;
import android.view.FocusFinder;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.GoldAlignExample;

import java.util.FormatFlagsConversionMismatchException;

@Autonomous(name="Competition", group="Pushbot")
//@Disabled
public class Competition extends LinearOpMode{
    // Competition

/*
    The robot is latched sideways. The front faces to the right.
    Change all programs unless camera is on the other side of the latching.

 */

    /* Public OpMode members. */
    ProjectRoverRuckusMecanumDriveHardware robot = new ProjectRoverRuckusMecanumDriveHardware();
    private ElapsedTime runtime  = new ElapsedTime();
    GoldAlignDetector detector = new GoldAlignDetector();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark NeveRest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     FORWARD_SPEED   = 0.6;
    static final double     TURN_SPEED      = 0.55;
    static final double     SUPERSONICSPEED      = 1.0;
    double                  stop            = 0.0;

    double boneDispenser_up_position = 0.0;
    double boneDispenser_down_position = 0.4;

    double latch_up_speed = 0.5;
    double latch_down_speed = 0.5;

    long sleepTime = 1000;

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);
        robot.boneDispenser.setPosition(boneDispenser_up_position);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3){
            moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3){
            strafeLeft(FORWARD_SPEED);
            telemetry.addData("Status", "Moving left");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5){
            moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }

        runtime.reset();
        // Sampling code
        while(detector.getAligned() == false){
            encoderDriveRight(FORWARD_SPEED, FORWARD_SPEED, 20, 4.0);
        }
        if (detector.getAligned() == true){
            // Knock mineral with mineral collector
            encoderDriveForward(FORWARD_SPEED, 3, 0.15);
            encoderDriveBackward(FORWARD_SPEED, 3, 0.15);
            encoderDriveRight(FORWARD_SPEED, FORWARD_SPEED, 20-robot.leftDriveFront.getCurrentPosition(), 4.0-getRuntime());
        }

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.15){
            turnLeft(TURN_SPEED);
            telemetry.addData("Status", "Turning left");
            telemetry.update();
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.15){
            strafeRight(FORWARD_SPEED);
            telemetry.addData("Status", "Moving right");
            telemetry.update();
        }

        encoderDriveForward(FORWARD_SPEED, 44, 3.0);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5){
            turnLeft(TURN_SPEED);
            telemetry.addData("Status", "Turning left");
            telemetry.update();
        }
        while(opModeIsActive()) {
            robot.boneDispenser.setPosition(boneDispenser_down_position);
            telemetry.addData("Status", "Claimed");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.0){
            moveForward(SUPERSONICSPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }
        telemetry.addData("Status", "Parked");
        telemetry.update();
    }

    public void moveForward(double forwardSpeed){
        robot.leftDriveFront.setPower(forwardSpeed);
        robot.rightDriveFront.setPower(forwardSpeed);
        robot.leftDriveBack.setPower(forwardSpeed);
        robot.rightDriveBack.setPower(forwardSpeed);
    }
    public void turnLeft(double turnSpeed) {
        robot.leftDriveFront.setPower(-turnSpeed);
        robot.rightDriveFront.setPower(turnSpeed);
        robot.leftDriveBack.setPower(-turnSpeed);
        robot.rightDriveBack.setPower(turnSpeed);
    }
    public void turnRight (double turnSpeed) {
        robot.leftDriveFront.setPower(turnSpeed);
        robot.rightDriveFront.setPower(-turnSpeed);
        robot.leftDriveBack.setPower(turnSpeed);
        robot.rightDriveBack.setPower(-turnSpeed);
    }
    public void moveBackwards(double backwardSpeed) {
        robot.leftDriveFront.setPower(-backwardSpeed);
        robot.rightDriveFront.setPower(-backwardSpeed);
        robot.leftDriveBack.setPower(-backwardSpeed);
        robot.rightDriveBack.setPower(-backwardSpeed);
    }
    public void strafeLeft(double forwardSpeed){
        robot.leftDriveFront.setPower(forwardSpeed);
        robot.rightDriveFront.setPower(-forwardSpeed);
        robot.leftDriveBack.setPower(-forwardSpeed);
        robot.rightDriveBack.setPower(forwardSpeed);
    }
    public void strafeRight(double forwardSpeed) {
        robot.leftDriveFront.setPower(-forwardSpeed);
        robot.rightDriveFront.setPower(forwardSpeed);
        robot.leftDriveBack.setPower(forwardSpeed);
        robot.rightDriveBack.setPower(-forwardSpeed);
    }


    //method that drives the robot forward
    public void encoderDriveForward(double speed,
                                    double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftDriveFront.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightDriveFront.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newRightBackTarget = robot.rightDriveBack.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);

            robot.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            robot.rightDriveFront.setTargetPosition(newRightFrontTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveFront.setPower(Math.abs(speed));
            robot.rightDriveFront.setPower(Math.abs(speed));
            robot.leftDriveBack.setPower(Math.abs(speed));
            robot.rightDriveBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) && robot.rightDriveBack.isBusy() && robot.leftDriveBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveFront.getCurrentPosition(),
                        robot.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    //method that drives the robot backwards
    public void encoderDriveBackward(double speed,
                                     double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftDriveFront.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightDriveFront.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newRightBackTarget = robot.rightDriveBack.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);

            robot.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            robot.rightDriveFront.setTargetPosition(newRightFrontTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveFront.setPower(Math.abs(speed));
            robot.rightDriveFront.setPower(Math.abs(speed));
            robot.leftDriveBack.setPower(Math.abs(speed));
            robot.rightDriveBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) && robot.rightDriveBack.isBusy() && robot.leftDriveBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveFront.getCurrentPosition(),
                        robot.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    //method that drives the robot the left
    public void encoderDriveLeft(double speedFront, double speedBack,
                                 double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = robot.leftDriveFront.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightDriveFront.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newRightBackTarget = robot.rightDriveBack.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            robot.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            robot.rightDriveFront.setTargetPosition(newRightFrontTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveFront.setPower(Math.abs(speedFront));
            robot.rightDriveFront.setPower(Math.abs(speedFront));
            robot.leftDriveBack.setPower(Math.abs(speedBack));
            robot.rightDriveBack.setPower(Math.abs(speedBack));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) && robot.rightDriveBack.isBusy() && robot.leftDriveBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveFront.getCurrentPosition(),
                        robot.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    //method that drives the robot to the right
    public void encoderDriveRight(double speedFront, double speedBack,
                                  double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftDriveFront.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightDriveFront.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newRightBackTarget = robot.rightDriveBack.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);

            robot.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            robot.rightDriveFront.setTargetPosition(newRightFrontTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveFront.setPower(Math.abs(speedFront));
            robot.rightDriveFront.setPower(Math.abs(speedFront));
            robot.leftDriveBack.setPower(Math.abs(speedBack));
            robot.rightDriveBack.setPower(Math.abs(speedBack));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) && robot.rightDriveBack.isBusy() && robot.leftDriveBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveFront.getCurrentPosition(),
                        robot.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    //method that turns the robot to the right
    public void encoderDriveTurnRight(double speed,
                                      double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftDriveFront.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightDriveFront.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
            newRightBackTarget = robot.rightDriveBack.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);

            robot.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            robot.rightDriveFront.setTargetPosition(newRightFrontTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveFront.setPower(Math.abs(speed));
            robot.rightDriveFront.setPower(Math.abs(speed));
            robot.leftDriveBack.setPower(Math.abs(speed));
            robot.rightDriveBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) && robot.rightDriveBack.isBusy() && robot.leftDriveBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveFront.getCurrentPosition(),
                        robot.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    //method that turns the robot to the right
    public void encoderDriveTurnLeft(double speed,
                                     double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftDriveFront.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightDriveFront.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
            newRightBackTarget = robot.rightDriveBack.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);

            robot.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            robot.rightDriveFront.setTargetPosition(newRightFrontTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDriveFront.setPower(Math.abs(speed));
            robot.rightDriveFront.setPower(Math.abs(speed));
            robot.leftDriveBack.setPower(Math.abs(speed));
            robot.rightDriveBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()) && robot.rightDriveBack.isBusy() && robot.leftDriveBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDriveFront.getCurrentPosition(),
                        robot.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveBack.setPower(0);
            robot.rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}