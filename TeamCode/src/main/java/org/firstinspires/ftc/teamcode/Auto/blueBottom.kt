package org.firstinspires.ftc.teamcode.Auto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.pedro.PedroOpMode


@Autonomous(name = "PedroPath")
class blueBottom: PedroOpMode() {
    //starting position



    private fun buildPaths() {
    }





    val secondRoutine: Command
        get() = SequentialGroup(

        )

    override fun onInit() {
       // follower = Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
        //follower.setMaxPower(0.7)
        //follower.setStartingPose(//startPose)
       // buildPaths()
    }

    override fun onStartButtonPressed() {
        secondRoutine()
    }
}