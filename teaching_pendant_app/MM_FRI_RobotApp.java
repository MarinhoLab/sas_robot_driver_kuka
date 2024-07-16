// This app simply connects to the robot, sets it to joint position control, and awaits for FRI joint-position commands. 
package application;

/*
Import KUKA LBR packages
 */
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;


public class MM_FRI_RobotApp extends RoboticsAPIApplication 
{
	private LBR lbr_7_800;
	private Controller lbr_7_800_Ctrl;
	private String FRI_ClientIP;

	@Override
	public void initialize() 
	{
		lbr_7_800_Ctrl = (Controller) getContext().getControllers().toArray()[0];
		lbr_7_800 = (LBR) lbr_7_800_Ctrl.getDevices().toArray()[0];
		
		// **********************************************************************
		// *** change next line to the FRIClient's IP address                 ***
		// **********************************************************************
		FRI_ClientIP = "192.170.10.12";
	}

	@Override
	public void run() throws Exception {

		PositionControlMode ctrMode = new PositionControlMode();
		PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);

		// Configure and start FRI session
		FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lbr_7_800, FRI_ClientIP);
		friConfiguration.setReceiveMultiplier(1);
		friConfiguration.setSendPeriodMilliSec(1);//Important: This number should be smaller than 10 for joint-position based control

		getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
		getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
				+ " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

		FRISession friSession = new FRISession(friConfiguration);
		FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);

		// Await until FRI session is ready to switch to command mode
		try
		{
			friSession.await(10, TimeUnit.SECONDS);

		}
		catch (final TimeoutException e)
		{
			//If there is no connection, then close the FRI session
			getLogger().error(e.getLocalizedMessage());
			friSession.close();
			return;
		}
		getLogger().info("FRI connection established.");

		try
		{
			while (true)
			{
				lbr_7_800.moveAsync(posHold.addMotionOverlay(jointOverlay));
				FRIChannelInformation.FRIConnectionQuality Conn_Quality = friSession.getFRIChannelInformation().getQuality();
				if ((Conn_Quality != FRIChannelInformation.FRIConnectionQuality.EXCELLENT) && (Conn_Quality != FRIChannelInformation.FRIConnectionQuality.GOOD))
				{
					getLogger().info("FRI connection quality is too low.");
					getLogger().info(String.valueOf(friSession.getFRIChannelInformation().getJitter()));
					getLogger().info(String.valueOf(friSession.getFRIChannelInformation().getLatency()));
					//TODO: Investigate if we really need to die here or we can simply retry.
					break;
				}
			}
		}
		catch(Exception run_err)
		{
			friSession.close();
			getLogger().error(run_err.getLocalizedMessage());
			getLogger().info(String.valueOf(friSession.getFRIChannelInformation().getLatency()));
			return;
		}

		//Close the FRI session if the programme is done.
		friSession.close();
		getLogger().info("FRI connection ended.");

	}

	public static void main(final String[] args)
	{
		final MM_FRI_RobotApp app = new MM_FRI_RobotApp();
		app.runApplication();
	}
}
