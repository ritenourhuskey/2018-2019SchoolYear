package org.firstinspires.ftc.teamcode;
import android.view.FocusFinder;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.GoldAlignExample;
import org.firstinspires.ftc.teamcode.Competition;
import java.util.FormatFlagsConversionMismatchException;

@Autonomous(name="RedDepotWithSample", group="Pushbot")
//@Disabled
public class RedDepotWithSample extends LinearOpMode{
    // Red Depot Lander
    // NEED TO TEST
    /* Public OpMode members. */
    ProjectRoverRuckusMecanumDriveHardware robot = new ProjectRoverRuckusMecanumDriveHardware();
    private ElapsedTime runtime  = new ElapsedTime();
    GoldAlignDetector detector = new GoldAlignDetector();
    Competition move = new Competition();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark NeveRest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     FORWARD_SPEED   = 0.6;
    static final double     TURN_SPEED      = 0.55;
    static final double     SUPERSONICSPEED      = 1.0;
    double sample_speed = 0.3;

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
            move.moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3){
            move.strafeRight(FORWARD_SPEED);
            telemetry.addData("Status", "Moving right");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5){
            move.moveForward(FORWARD_SPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }

        runtime.reset();
        // Sampling code
        while(detector.getAligned() == false){
            move.encoderDriveLeft(FORWARD_SPEED, FORWARD_SPEED, 20, 4.0);
        }
        if (detector.getAligned() == true){
            // Knock mineral with mineral collector
            move.encoderDriveForward(FORWARD_SPEED, 3, 0.15);
            move.encoderDriveBackward(FORWARD_SPEED, 3, 0.15);
            move.encoderDriveLeft(FORWARD_SPEED, FORWARD_SPEED, 20-robot.leftDriveFront.getCurrentPosition(), 4.0-getRuntime());
        }

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.15){
            move.turnLeft(TURN_SPEED);
            telemetry.addData("Status", "Turning left");
            telemetry.update();
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.15){
            move.strafeRight(FORWARD_SPEED);
            telemetry.addData("Status", "Moving right");
            telemetry.update();
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.15){
            move.moveBackwards(FORWARD_SPEED);
            telemetry.addData("Status", "Moving backwards");
            telemetry.update();
        }
        sleep(sleepTime);
        while(opModeIsActive()){
            robot.boneDispenser.setPosition(boneDispenser_down_position);
            telemetry.addData("Status", "Claimed");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.0){
            move.moveForward(SUPERSONICSPEED);
            telemetry.addData("Status", "Moving forward");
            telemetry.update();
        }
        telemetry.addData("Status", "Parked");
        telemetry.update();
    }
}