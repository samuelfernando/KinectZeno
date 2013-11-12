package uk.ac.shef.kinectzeno;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.awt.*;
import java.awt.image.*;

import org.openni.*;
import com.primesense.nite.*;
import java.io.PrintStream;
import java.text.DecimalFormat;
import javax.swing.JLabel;
import org.robokind.api.common.position.NormalizedDouble;
import org.robokind.api.motion.Joint;
import org.robokind.api.motion.Robot.JointId;
import org.robokind.api.motion.Robot.RobotPositionMap;
import org.robokind.api.motion.messaging.RemoteRobot;
import org.robokind.client.basic.Robokind;
import static org.robokind.client.basic.RobotJoints.LEFT_ELBOW_PITCH;
import static org.robokind.client.basic.RobotJoints.LEFT_ELBOW_YAW;
import static org.robokind.client.basic.RobotJoints.LEFT_SHOULDER_ROLL;
import org.robokind.client.basic.UserSettings;

public class UserViewer extends Component 
                        implements UserTracker.NewFrameListener {
    
    float mHistogram[];
    int[] mDepthPixels;
    UserTracker mTracker;
    UserTrackerFrameRef mLastFrame;
    BufferedImage mBufferedImage;
    int[] mColors;
    RemoteRobot myRobot;
    JointId left_elbow_yaw;
    JointId left_elbow_pitch;
    JointId left_shoulder_roll;
    RobotPositionMap myGoalPositions;
    PrintStream out;
    JLabel positionLabel;
    DecimalFormat df;
    
    public UserViewer(UserTracker tracker, JLabel positionLabel) {
        String robotID = "myRobot";
        String robotIP = "192.168.0.54";
        // set respective addresses
        UserSettings.setRobotId(robotID);
        UserSettings.setRobotAddress(robotIP);
    	mTracker = tracker;
        this.positionLabel = positionLabel;
        mTracker.addNewFrameListener(this);
        df = new DecimalFormat("#.##");
       // myRobot = Robokind.connectRobot();
        mColors = new int[] { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFF00, 0xFFFF00FF, 0xFF00FFFF };
/*        left_elbow_yaw = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_ELBOW_YAW)); 
        left_elbow_pitch = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_ELBOW_PITCH));
        left_shoulder_roll = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_SHOULDER_ROLL));
        
        myGoalPositions = new org.robokind.api.motion.Robot.RobotPositionHashMap();
  */
   }
    
    public synchronized void paint(Graphics g) {
        if (mLastFrame == null) {
            return;
        }
        
        int framePosX = 0;
        int framePosY = 0;
        
        VideoFrameRef depthFrame = mLastFrame.getDepthFrame();
        if (depthFrame != null) {
	        int width = depthFrame.getWidth();
	        int height = depthFrame.getHeight();
	        
	        // make sure we have enough room
	        if (mBufferedImage == null || mBufferedImage.getWidth() != width || mBufferedImage.getHeight() != height) {
	            mBufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
	        }
	        
	        mBufferedImage.setRGB(0, 0, width, height, mDepthPixels, 0, width);
	        
	        framePosX = (getWidth() - width) / 2;
	        framePosY = (getHeight() - height) / 2;

	        g.drawImage(mBufferedImage, framePosX, framePosY, null);
        }
        
        for (UserData user : mLastFrame.getUsers()) {
        	if (user.getSkeleton().getState() == SkeletonState.TRACKED) {
        		drawLimb(g, framePosX, framePosY, user, JointType.HEAD, JointType.NECK);
        		
        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_SHOULDER, JointType.LEFT_ELBOW);
     
      
                        drawLimb(g, framePosX, framePosY, user, JointType.LEFT_ELBOW, JointType.LEFT_HAND);

        		drawLimb(g, framePosX, framePosY, user, JointType.RIGHT_SHOULDER, JointType.RIGHT_ELBOW);
        		drawLimb(g, framePosX, framePosY, user, JointType.RIGHT_ELBOW, JointType.RIGHT_HAND);

        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_SHOULDER, JointType.RIGHT_SHOULDER);

        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_SHOULDER, JointType.TORSO);
        		drawLimb(g, framePosX, framePosY, user, JointType.RIGHT_SHOULDER, JointType.TORSO);

        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_HIP, JointType.TORSO);
        		drawLimb(g, framePosX, framePosY, user, JointType.RIGHT_HIP, JointType.TORSO);
        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_HIP, JointType.RIGHT_HIP);

        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_HIP, JointType.LEFT_KNEE);
        		drawLimb(g, framePosX, framePosY, user, JointType.LEFT_KNEE, JointType.LEFT_FOOT);

        		drawLimb(g, framePosX, framePosY, user, JointType.RIGHT_HIP, JointType.RIGHT_KNEE);
        		drawLimb(g, framePosX, framePosY, user, JointType.RIGHT_KNEE, JointType.RIGHT_FOOT);
        	}
        }
    }

    private void drawLimb(Graphics g, int x, int y, UserData user, JointType from, JointType to) {
    	com.primesense.nite.SkeletonJoint fromJoint = user.getSkeleton().getJoint(from);
    	com.primesense.nite.SkeletonJoint toJoint = user.getSkeleton().getJoint(to);
    	
        if (fromJoint.getPositionConfidence() == 0.0 || toJoint.getPositionConfidence() == 0.0) {
            return;
    	}
    	
      
    	
    	com.primesense.nite.Point2D<Float> fromPos = mTracker.convertJointCoordinatesToDepth(fromJoint.getPosition());
    	com.primesense.nite.Point2D<Float> toPos = mTracker.convertJointCoordinatesToDepth(toJoint.getPosition());

        if (from==JointType.LEFT_SHOULDER && to==JointType.LEFT_ELBOW) {
           float dx = toJoint.getPosition().getX() - fromJoint.getPosition().getX();
           float dy = toJoint.getPosition().getY() - fromJoint.getPosition().getY();
           float dz = toJoint.getPosition().getZ() - fromJoint.getPosition().getZ();
           
           
           
           //String text = "dx "+df.format(dx);
           //text += "dy "+df.format(dy);
           //text += "dz "+df.format(dz);
           
           //positionLabel.setText(text);
        }
    	
        
    	// draw it in another color than the use color
    	g.setColor(new Color(mColors[(user.getId() + 1) % mColors.length]));
    	g.drawLine(x + fromPos.getX().intValue(), y + fromPos.getY().intValue(), x + toPos.getX().intValue(), y + toPos.getY().intValue());
    }
    
    public synchronized void onNewFrame(UserTracker tracker) {
        if (mLastFrame != null) {
            mLastFrame.release();
            mLastFrame = null;
        }
        
        mLastFrame = mTracker.readFrame();
        
        // check if any new user detected
        for (UserData user : mLastFrame.getUsers()) {
        	if (user.isNew()) {
        		// start skeleton tracking
        		mTracker.startSkeletonTracking(user.getId());
        	}
        }

        VideoFrameRef depthFrame = mLastFrame.getDepthFrame();
        
        if (depthFrame != null) {
        	ByteBuffer frameData = depthFrame.getData().order(ByteOrder.LITTLE_ENDIAN);
            ByteBuffer usersFrame = mLastFrame.getUserMap().getPixels().order(ByteOrder.LITTLE_ENDIAN);
        
	        // make sure we have enough room
	        if (mDepthPixels == null || mDepthPixels.length < depthFrame.getWidth() * depthFrame.getHeight()) {
	        	mDepthPixels = new int[depthFrame.getWidth() * depthFrame.getHeight()];
	        }
        
            calcHist(frameData);
            frameData.rewind();

            int pos = 0;
            while(frameData.remaining() > 0) {
                short depth = frameData.getShort();
                short userId = usersFrame.getShort();
                short pixel = (short)mHistogram[depth];
                int color = 0xFFFFFFFF;
                if (userId > 0) {
                	color = mColors[userId % mColors.length];
                }
                
                mDepthPixels[pos] = color & (0xFF000000 | (pixel << 16) | (pixel << 8) | pixel);
                pos++;
            }
        }

        repaint();
    }

    private void calcHist(ByteBuffer depthBuffer) {
        // make sure we have enough room
        if (mHistogram == null) {
            mHistogram = new float[10000];
        }
        
        // reset
        for (int i = 0; i < mHistogram.length; ++i)
            mHistogram[i] = 0;

        int points = 0;
        while (depthBuffer.remaining() > 0) {
            int depth = depthBuffer.getShort() & 0xFFFF;
            if (depth != 0) {
                mHistogram[depth]++;
                points++;
            }
        }

        for (int i = 1; i < mHistogram.length; i++) {
            mHistogram[i] += mHistogram[i - 1];
        }

        if (points > 0) {
            for (int i = 1; i < mHistogram.length; i++) {
                mHistogram[i] = (int) (256 * (1.0f - (mHistogram[i] / (float) points)));
            }
        }
    }
}
