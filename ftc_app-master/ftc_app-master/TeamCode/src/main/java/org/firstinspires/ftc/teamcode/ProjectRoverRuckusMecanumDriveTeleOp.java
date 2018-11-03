/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dropbox.SampleFiles.MecanumDriveTeleOp;

@TeleOp(name="ProjectRoverRuckusMecanumDriveTeleOp", group="RR")
//@Disabled
public class ProjectRoverRuckusMecanumDriveTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    ProjectRoverRuckusMecanumDriveHardware robot = new ProjectRoverRuckusMecanumDriveHardware();
    private ElapsedTime runtime  = new ElapsedTime();
    @Override
    public void runOpMode() {

        double gamepad1LeftY;
        double gamepad1LeftX;
        double gamepad1RightX;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Welcome to Rover Ruckus Mecanum.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            gamepad1LeftY = -gamepad1.left_stick_y;
            gamepad1LeftX = -gamepad1.left_stick_x;
            gamepad1RightX = gamepad1.right_stick_x;

            telemetry.addData("LeftY: ", gamepad1LeftY);
            telemetry.addData("LeftX: ", gamepad1LeftX);
            telemetry.addData("RightX: ", gamepad1RightX);

            telemetry.update();

            //Mecanum Formulas
            final double v1 = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
            final double v2 = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            final double v3 = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            final double v4 = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;

            //Write the values to the Motor`
            robot.leftDriveFront.setPower(v1);
            robot.leftDriveBack.setPower(v3);

            robot.rightDriveFront.setPower(v2);
            robot.rightDriveBack.setPower(v4);

            //Latch
            if (gamepad1.dpad_up){
                robot.LatchUp.setPower(-1);
            }
            else if (gamepad1.dpad_down){
                robot.LatchUp.setPower(1);
            }
            else{
                robot.LatchUp.setPower(0);
            }
            //servo latch
            if (gamepad1.left_bumper){
                robot.Latch.setPosition(0);
            }
            else if (gamepad1.right_bumper){
                robot.Latch.setPosition(1);
            }
            else{
                robot.Latch.setPosition(.5);
            }

            //Bone Dispenser
            if (gamepad1.dpad_left){
                robot.boneDispenser.setPosition(0);
            }
            else if (gamepad1.dpad_right) {
                robot.boneDispenser.setPosition(.4);
            }

            //arm
            if (gamepad1.y){
                robot.armLow.setPower(.5);
            }
            else if (gamepad1.x){
                robot.armLow.setPower(-.5);
            }
            else {
                robot.armLow.setPower(0);
            }

            //Kermit Collector
            // a - takes in the minerals
            // b - spits out the minerals
            if (gamepad1.b){
                robot.KermitCollector.setPosition(0);
            }
            else if (gamepad1.a){
                robot.KermitCollector.setPosition(1);
            }
            else{
                robot.KermitCollector.setPosition(.5);
            }

/*            // KermitArm
            if (gamepad1.a && gamepad1.dpad_up){
                robot.KermitExtender.setPower(0.5);
            }
            else if(gamepad1.a && gamepad1.dpad_down){
                robot.KermitExtender.setPower(-0.5);
            }
            else{
                robot.KermitExtender.setPower(0);
            }
*/            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
