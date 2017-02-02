        /*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

        @Autonomous(name="Corner Push", group="Linear Opmode")
        @Disabled
        public class RedPushBall extends LinearOpMode {
            /* Declare OpMode members. */
            private ElapsedTime runtime = new ElapsedTime();
            private DcMotor leftMotor = null;
            private DcMotor rightMotor = null;

            @Override
            public void runOpMode() {
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                //Initialize the hardware variables. Same as configured on phone
                leftMotor = hardwareMap.dcMotor.get("leftDrive");
                rightMotor = hardwareMap.dcMotor.get("rightDrive");
                //Set the drive motor directions
                rightMotor.setDirection(DcMotor.Direction.REVERSE);
                // Wait for the game to start (driver presses PLAY)
                waitForStart();

                back(1);
                Telemetry("forward", 3);
            }

            public void Telemetry(String event, double time) {
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < time)) {
                    telemetry.addData(event, "S Elapsed " + runtime.seconds());
                    telemetry.addData("Right ", leftMotor.getPower());
                    telemetry.addData("Left ", rightMotor.getPower());
                    telemetry.update();
                }
            }

            public void back(double power) {
                leftMotor.setPower(-power);
                rightMotor.setPower(-power);
            }
        }
