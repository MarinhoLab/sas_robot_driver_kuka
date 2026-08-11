// This application connects to the robot, enables joint-position control,
// and waits for FRI joint-position commands at 1000 Hz.

package application;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

public class MM_FRI_RobotApp extends RoboticsAPIApplication
{
    private static final int FRI_SEND_PERIOD_MS = 1;
    private static final int FRI_RECEIVE_MULTIPLIER = 1;
    private static final int FRI_CONNECTION_TIMEOUT_SECONDS = 10;
    private static final int MONITORING_PERIOD_MS = 100;

    private LBR lbr_7_800;
    private Controller lbr_7_800_Ctrl;
    private String FRI_ClientIP;

    @Override
    public void initialize()
    {
        lbr_7_800_Ctrl =
                (Controller) getContext().getControllers().toArray()[0];

        lbr_7_800 =
                (LBR) lbr_7_800_Ctrl.getDevices().toArray()[0];

        // FRI client's IP address.
        FRI_ClientIP = "192.170.10.12";
    }

    @Override
    public void run()
    {
        FRISession friSession = null;
        IMotionContainer motionContainer = null;

        try
        {
            /*
             * Configure the robot-side control mode.
             */
            PositionControlMode controlMode =
                    new PositionControlMode();

            PositionHold positionHold =
                    new PositionHold(
                            controlMode,
                            -1,
                            TimeUnit.SECONDS);

            /*
             * Configure the FRI session.
             *
             * A send period of 1 ms and receive multiplier of 1 request
             * one state/command exchange every millisecond.
             */
            FRIConfiguration friConfiguration =
                    FRIConfiguration.createRemoteConfiguration(
                            lbr_7_800,
                            FRI_ClientIP);

            friConfiguration.setSendPeriodMilliSec(
                    FRI_SEND_PERIOD_MS);

            friConfiguration.setReceiveMultiplier(
                    FRI_RECEIVE_MULTIPLIER);

            getLogger().info(
                    "Creating FRI connection to "
                    + friConfiguration.getHostName());

            getLogger().info(
                    "Send period: "
                    + friConfiguration.getSendPeriodMilliSec()
                    + " ms | Receive multiplier: "
                    + friConfiguration.getReceiveMultiplier());

            /*
             * Create the FRI session and joint-position overlay.
             */
            friSession = new FRISession(friConfiguration);

            FRIJointOverlay jointOverlay =
                    new FRIJointOverlay(friSession);

            /*
             * Wait until the client is connected and FRI is ready
             * to enter command mode.
             */
            getLogger().info(
                    "Waiting for the FRI client to connect...");

            friSession.await(
                    FRI_CONNECTION_TIMEOUT_SECONDS,
                    TimeUnit.SECONDS);

            getLogger().info(
                    "FRI connection established.");

            /*
             * Start the indefinite position hold with the FRI overlay.
             *
             * This motion must be started only once. The FRI subsystem
             * handles the cyclic 1 ms exchange independently.
             */
            motionContainer =
                    lbr_7_800.moveAsync(
                            positionHold.addMotionOverlay(
                                    jointOverlay));

            getLogger().info(
                    "FRI joint-position overlay started.");

            /*
             * Monitor the active FRI session.
             *
             * This loop does not generate the 1 kHz command rate.
             * It only monitors the session and active motion.
             */
            while (!motionContainer.isFinished())
            {
                FRIChannelInformation channelInformation =
                        friSession.getFRIChannelInformation();

                FRIChannelInformation.FRIConnectionQuality quality =
                        channelInformation.getQuality();

                if (quality
                        != FRIChannelInformation
                                .FRIConnectionQuality.EXCELLENT
                        && quality
                        != FRIChannelInformation
                                .FRIConnectionQuality.GOOD)
                {
                    getLogger().warn(
                            "FRI connection quality is too low.");

                    getLogger().warn(
                            "Connection quality: "
                            + quality);

                    getLogger().warn(
                            "FRI jitter: "
                            + channelInformation.getJitter());

                    getLogger().warn(
                            "FRI latency: "
                            + channelInformation.getLatency());

                    break;
                }

                /*
                 * Prevent this supervisory loop from consuming a CPU core.
                 * This sleep has no effect on the internal 1 ms FRI cycle.
                 */
                Thread.sleep(MONITORING_PERIOD_MS);
            }
        }
        catch (TimeoutException timeoutException)
        {
            getLogger().error(
                    "Timed out while waiting for the FRI client: "
                    + timeoutException.getLocalizedMessage());
        }
        catch (InterruptedException interruptedException)
        {
            Thread.currentThread().interrupt();

            getLogger().error(
                    "The FRI monitoring thread was interrupted: "
                    + interruptedException.getLocalizedMessage());
        }
        catch (Exception exception)
        {
            getLogger().error(
                    "FRI application error: "
                    + exception.getLocalizedMessage());

            if (friSession != null)
            {
                FRIChannelInformation channelInformation =
                        friSession.getFRIChannelInformation();

                getLogger().error(
                        "FRI connection quality: "
                        + channelInformation.getQuality());

                getLogger().error(
                        "FRI jitter: "
                        + channelInformation.getJitter());

                getLogger().error(
                        "FRI latency: "
                        + channelInformation.getLatency());
            }
        }
        finally
        {
            /*
             * Stop the active robot motion before closing the FRI session.
             */
            if (motionContainer != null
                    && !motionContainer.isFinished())
            {
                getLogger().info(
                        "Cancelling FRI position hold.");

                motionContainer.cancel();
            }

            if (friSession != null)
            {
                getLogger().info(
                        "Closing FRI session.");

                friSession.close();
            }

            getLogger().info(
                    "FRI connection ended.");
        }
    }

    public static void main(final String[] args)
    {
        MM_FRI_RobotApp app =
                new MM_FRI_RobotApp();

        app.runApplication();
    }
}