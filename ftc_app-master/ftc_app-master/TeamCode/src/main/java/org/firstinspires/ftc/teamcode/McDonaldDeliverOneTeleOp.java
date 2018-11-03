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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="McDonaldDeliverOneTeleOp", group="McDonaldDelivery")
@Disabled
public class McDonaldDeliverOneTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    McDonaldDeliverOne robot = new McDonaldDeliverOne();

    @Override
    public void runOpMode() {
        // These two values are the motor values for both the back and front motors using the joystick
        double leftControlStickY;
        double leftControlStickX;
        double rightControlStickY;
        double rightControlStickX;
        double SPEED_MODIFIER;
        double SPEED_MODIFIER_INTERVAL;


        robot.init(hardwareMap);

        SPEED_MODIFIER = 1.00;
        SPEED_MODIFIER_INTERVAL = .25;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "McDonaldDelivery 1.0 Activated");
        telemetry.addData("Speed Modifier: ", SPEED_MODIFIER);

        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            //Takes the Values from the gamepad and write it to motor
            robot.rightDriveBack.setPower(gamepad1.right_stick_y * SPEED_MODIFIER);
            robot.rightDriveFront.setPower(gamepad1.right_stick_y * SPEED_MODIFIER);
            robot.leftDriveBack.setPower(-gamepad1.left_stick_y * SPEED_MODIFIER);
            robot.leftDriveFront.setPower(-gamepad1.left_stick_y * SPEED_MODIFIER);



            //Set the SPEED_MODIFIER value to control speed more easily
            if (gamepad1.left_bumper){
                if (SPEED_MODIFIER > 0) {
                    SPEED_MODIFIER -= SPEED_MODIFIER_INTERVAL;

                }
            }
            if (gamepad1.right_bumper){
                if (SPEED_MODIFIER < 1) {
                    SPEED_MODIFIER += SPEED_MODIFIER_INTERVAL;

                }
            }
            telemetry.addData("say", gamepad1.left_trigger);
            telemetry.update();

            //Control Elevators
            if (gamepad1.dpad_up){
                robot.leftArm.setPower(.75);
                robot.rightArm.setPower(.75);
            }
            else if (gamepad1.dpad_down){
                robot.leftArm.setPower(-.75);
                robot.rightArm.setPower(-.75);
            }
            else{
                robot.leftArm.setPower(0);
                robot.rightArm.setPower(0);
            }
            if (gamepad1.dpad_left){
                robot.middleArm.setPower(-.75);
            }
            else if (gamepad1.dpad_right){
                robot.middleArm.setPower(.75);
            }
            else{
                robot.middleArm.setPower(0);
            }

            //quick shutoff button for safty
            if (gamepad1.start && gamepad1.x){
                System.exit(1);
            }


            telemetry.addData("Speed Modifier: ", SPEED_MODIFIER);
            telemetry.addData("say", gamepad1.left_trigger);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
