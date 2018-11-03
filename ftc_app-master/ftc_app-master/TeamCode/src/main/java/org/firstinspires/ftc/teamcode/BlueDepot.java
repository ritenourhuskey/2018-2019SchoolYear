package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BlueDepot", group="PushBot")
//@Disabled
public class BlueDepot extends LinearOpMode{

    ProjectRoverRuckusMecanumDriveHardware robot = new ProjectRoverRuckusMecanumDriveHardware();
    private ElapsedTime runtime  = new ElapsedTime();

    static final double     FORWARD_SPEED   = 0.6;
    static final double     TURN_SPEED      = 0.55;
    double  stop = 0.0;

    double boneDispenser_up_position = 0.0;
    double boneDispenser_down_position = 0.4;

    long sleepTime = 1000;

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);
        robot.boneDispenser.setPosition(boneDispenser_up_position);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        /* Mineral A is closer to blue crater
         * Mineral B is in the middle
         * Mineral C is closer to red crater
         */

        // Turn left to face mineral C
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3){
            moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.35){
            turnLeft(TURN_SPEED);
            telemetry.addData("Status", "Turning right");
            telemetry.update();
        }
        runtime.reset();
        // Move toward mineral C
        while (opModeIsActive() && runtime.seconds() < 1.0){
            moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }

        // If mineral C is gold - knock it. Then move to depot.


        sleep(sleepTime);
        runtime.reset();
        // Move forward toward wall
        while (opModeIsActive() && runtime.seconds() < 1.503){
            moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }
        runtime.reset();
        // Turn around so marker is facing depot. Move to depot. Claim depot.
        while (opModeIsActive() && runtime.seconds() < 1.58){
            turnLeft(TURN_SPEED);
            telemetry.addData("Status", "Turning around");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.81){
            moveBackwards(FORWARD_SPEED);
            telemetry.addData("Status", "Moving backwards to depot");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.15){
            robot.boneDispenser.setPosition(boneDispenser_down_position);
            telemetry.addData("Status", "Claimed");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.0){
            moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving to crater");
            telemetry.update();
        }
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

    public void moveBackwards(double backwardSpeed){
        robot.leftDriveFront.setPower(-backwardSpeed);
        robot.rightDriveFront.setPower(-backwardSpeed);
        robot.leftDriveBack.setPower(-backwardSpeed);
        robot.rightDriveBack.setPower(-backwardSpeed);
    }
}
