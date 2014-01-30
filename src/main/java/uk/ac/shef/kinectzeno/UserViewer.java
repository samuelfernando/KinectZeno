package uk.ac.shef.kinectzeno;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.awt.*;
import java.awt.image.*;
import javax.vecmath.*;

import org.openni.*;
import com.primesense.nite.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.PrintStream;
import java.text.DecimalFormat;
import java.util.HashSet;
import javax.swing.JLabel;
import org.robokind.api.animation.Animation;
import org.robokind.api.animation.messaging.RemoteAnimationPlayerClient;
import org.robokind.api.common.position.NormalizedDouble;
import org.robokind.api.motion.Joint;
import org.robokind.api.motion.Robot.JointId;
import org.robokind.api.motion.Robot.RobotPositionMap;
import org.robokind.api.motion.messaging.RemoteRobot;
import org.robokind.api.speech.messaging.RemoteSpeechServiceClient;
import org.robokind.client.basic.Robokind;
import static org.robokind.client.basic.RobotJoints.LEFT_SHOULDER_PITCH;
import static org.robokind.client.basic.RobotJoints.LEFT_SHOULDER_ROLL;
import static org.robokind.client.basic.RobotJoints.LEFT_ELBOW_PITCH;
import static org.robokind.client.basic.RobotJoints.LEFT_ELBOW_YAW;
import static org.robokind.client.basic.RobotJoints.RIGHT_ELBOW_YAW;
import static org.robokind.client.basic.RobotJoints.RIGHT_SHOULDER_PITCH;
import static org.robokind.client.basic.RobotJoints.RIGHT_SHOULDER_ROLL;
import static org.robokind.client.basic.RobotJoints.RIGHT_ELBOW_PITCH;
import static org.robokind.client.basic.RobotJoints.NECK_YAW;
import static org.robokind.client.basic.RobotJoints.NECK_PITCH;
import static org.robokind.client.basic.RobotJoints.LEFT_SMILE;
import static org.robokind.client.basic.RobotJoints.RIGHT_SMILE;
import static org.robokind.client.basic.RobotJoints.BROWS;
import static org.robokind.client.basic.RobotJoints.EYELIDS;
import static org.robokind.client.basic.RobotJoints.NECK_ROLL;


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
    JointId left_shoulder_pitch;
    JointId left_shoulder_roll;
    JointId left_elbow_pitch;
    JointId right_elbow_yaw;
    JointId left_elbow_yaw;
    JointId right_shoulder_pitch;
    JointId right_shoulder_roll;
    JointId right_elbow_pitch;
    JointId neck_yaw;
     JointId neck_pitch;
     JointId neck_roll;
     JointId smile_left;
     JointId smile_right;
     JointId brows;
     JointId eyelids;
private static RemoteAnimationPlayerClient myPlayer;
    RobotPositionMap myGoalPositions;
    PrintStream out;
    JLabel positionLabel;
    DecimalFormat df;
    long lastUpdateTime = 0;
    long lastSpeak = 0;
    Skeleton lastSkeleton;
    float speed;
  float neck_vel = 0;
  HashSet<JointType> moveSet;
    private static RemoteSpeechServiceClient mySpeaker;
    public UserViewer(UserTracker tracker, JLabel positionLabel) {
        String robotID = "myRobot";
        //String robotIP = "192.168.0.54";
        try {
         BufferedReader br = new BufferedReader(new FileReader("C:\\Users\\zeno\\Documents\\NetBeansProjects\\zeno-ip.txt"));
         String robotIP = br.readLine();
        
        // set respective addresses
        UserSettings.setRobotId(robotID);
        UserSettings.setRobotAddress(robotIP);
         UserSettings.setSpeechAddress(robotIP);
         UserSettings.setAnimationAddress(robotIP);
    	mTracker = tracker;
        this.positionLabel = positionLabel;
        mTracker.addNewFrameListener(this);
        df = new DecimalFormat("#.##");
        mColors = new int[] { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFF00, 0xFFFF00FF, 0xFF00FFFF };
         mySpeaker = Robokind.connectSpeechService();
        myRobot = Robokind.connectRobot();
        myPlayer = Robokind.connectAnimationPlayer();
       
        left_shoulder_pitch = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_SHOULDER_PITCH));
        neck_yaw = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(NECK_YAW));
         neck_pitch = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(NECK_PITCH));
         smile_left = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_SMILE));        
       smile_right = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(RIGHT_SMILE));        
      brows = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(BROWS));        
      eyelids = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(EYELIDS));        
     
        left_elbow_pitch = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_ELBOW_PITCH));        
        left_shoulder_roll = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_SHOULDER_ROLL));
        right_shoulder_pitch = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(RIGHT_SHOULDER_PITCH));        
        right_elbow_pitch = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(RIGHT_ELBOW_PITCH));        
        right_shoulder_roll = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(RIGHT_SHOULDER_ROLL));
       left_elbow_yaw = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(LEFT_ELBOW_YAW));        
       right_elbow_yaw = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(RIGHT_ELBOW_YAW));        
          neck_roll = new org.robokind.api.motion.Robot.JointId(myRobot.getRobotId(), new Joint.Id(NECK_ROLL));        
      
       lastSkeleton = null;
        myGoalPositions = new org.robokind.api.motion.Robot.RobotPositionHashMap();
          myGoalPositions = myRobot.getDefaultPositions();
           myRobot.move(myGoalPositions, 1000);
           //setPosition(right_shoulder_pitch, 0.5f);
           // setPosition(left_shoulder_pitch, 0.5f);
        //myRobot.move(myGoalPositions, 1000);
        speed = 0.0f;
        moveSet = new HashSet<JointType>();
        moveSet.add(JointType.LEFT_ELBOW);
        moveSet.add(JointType.LEFT_HAND); 
        moveSet.add(JointType.LEFT_SHOULDER);
        moveSet.add(JointType.RIGHT_ELBOW);
        moveSet.add(JointType.RIGHT_HAND);
        moveSet.add(JointType.RIGHT_SHOULDER);
        
         }
        catch (Exception e) {
            e.printStackTrace();
        }
        
   }
    Point3f convertPoint(com.primesense.nite.Point3D<Float> p) {
        Point3f point = new Point3f();
        point.x = p.getX();
        point.y = p.getY();
        point.z = p.getZ();
        return point;
    }
    float planeAngle(Point3f a, Point3f b, Point3f c, Point3f d, Point3f e) {
        
       
        Vector3f vec1 =new Vector3f();
        vec1.sub(b, a);
        
        Vector3f vec2 =new Vector3f();
       vec2.sub(c,a);
        
        Vector3f vec3 =new Vector3f();
        vec3.sub(e,d);
        
        Vector3f normal = new Vector3f();
        normal.cross(vec1, vec2);
        
        float angle = vec3.angle(normal);
        
        return angle;
        
    }
    
    float vectorAngle(Point3f a, Point3f b, Point3f c) {
        Vector3f vec1 =new Vector3f();
        vec1.sub(b, a);
        
        Vector3f vec2 =new Vector3f();
       vec2.sub(c,a);
        
       float angle = vec1.angle(vec2);
       return angle;
        
    }
    
    float quatToAngle(Quaternion q) {
        Quat4f qv = new Quat4f();
        qv.x = q.getX();
        qv.y = q.getY();
        qv.w = q.getW();
        qv.z = q.getZ();
        AxisAngle4f a = new AxisAngle4f();
        a.set(qv);
        
        
        return a.angle;
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
                        
                        long timelapse = timeSinceLastUpdate();
                        if (timelapse>200) {
                            //moveRobot();
                            /* Point3f leftShoulder = convertPoint(user.getSkeleton().getJoint(JointType.LEFT_SHOULDER).getPosition());
                            Point3f rightShoulder = convertPoint(user.getSkeleton().getJoint(JointType.RIGHT_SHOULDER).getPosition());
                            Point3f torso = convertPoint(user.getSkeleton().getJoint(JointType.TORSO).getPosition());
                            Point3f leftHip = convertPoint(user.getSkeleton().getJoint(JointType.LEFT_HIP).getPosition());
                            Point3f leftKnee = convertPoint(user.getSkeleton().getJoint(JointType.LEFT_KNEE).getPosition());
                           
                            float leftHipPitch = planeAngle(leftShoulder, rightShoulder, torso, leftHip, leftKnee);

                            positionLabel.setText("left = "+leftHipPitch);
                        
                        */ 
                            
                           /* Point3f head = convertPoint(user.getSkeleton().getJoint(JointType.HEAD).getPosition());
                            //float dist = head.distance(lastPos);
                            
                            Point3f zeno_head = new Point3f(0,-400,0);
                            Point3f zeno_level = new Point3f(100,-400,0);
                            Point3f zeno_level2 = new Point3f(0,-400,100);

                            Point3f orig = new Point3f(0,0,0);
                            
                            Point3f center_head = new Point3f(0,270,2100);
                            Point3f center_feet = new Point3f(0,-270,2100);
                            */
                            //float angle = planeAngle(orig, center_head, center_feet, orig, head);
                            //angle = 0.5f+(float)Math.cos(angle)*0.7f;
                            //angle -= Math.PI/2;
                            //angle = 0.5f-angle;
                            
                            
                            //float angle = planeAngle(zeno_head, zeno_level, zeno_level2, zeno_head, head);
                            
                            //angle=0.7f-(float)Math.cos(angle);
                            //positionLabel.setText("dist "+dist);
                            float armDist = findDist(user, true);
                            float dist = findDist(user, false);
                            lastSkeleton = user.getSkeleton();
                            
                            /*if (speed>0.1) {
                                 moveArms();
                           
                                trackHead();
                               myRobot.move(myGoalPositions, 200);
                                lastUpdateTime = System.currentTimeMillis();

                            }
                            else {
                                moveQuizzical();
                            }*/
    
                            //int spText = 70+(int)((speed-0.5)*100);
                            float armSpeed = armDist / timelapse;
                            speed = dist/timelapse;
                            boolean needMove = false;
                            //if (armSpeed>0.7) {
                                moveArms();
                              //  needMove = true;
                            //}
                            //if (speed>2) {
                                trackHead();
                              //  needMove = true;
                            //}
                            //if (needMove) {
                               myRobot.move(myGoalPositions, 200);
                            //}
                            //else {
                                //setPosition(smile_left, 0.5f);
                                //setPosition(smile_right, 0.5f);
                                //moveQuizzical();
                                //myRobot.move(myGoalPositions, 200);
                            //}
                            
                            
                            lastUpdateTime = System.currentTimeMillis();
                            positionLabel.setText("speed "+df.format(speed)+" armSpeed "+df.format(armSpeed));
                           
                        }
                    
                    
                }
        }
    }

    private float findDist(UserData user, boolean armsOnly) {
        Skeleton skeleton = user.getSkeleton();

        if (lastSkeleton==null) {
            lastSkeleton = skeleton;
            return 1000.0f;
        }
        SkeletonJoint[] joints = skeleton.getJoints();
        float total=0.0f;
        for (SkeletonJoint joint : joints) {
            JointType type = joint.getJointType();
            SkeletonJoint lastJoint = lastSkeleton.getJoint(type);
            Point3f lastPos = convertPoint(lastJoint.getPosition());
            Point3f curPos = convertPoint(joint.getPosition());
            float dist = lastPos.distance(curPos);
            if (armsOnly) {
                if (moveSet.contains(type)) {        
                    total+=dist;
                }
            }
            else {
                total+=dist;
            }
        }
        
        
        return total;
    }
    private long timeSinceLastUpdate() {
        long currentTime = System.currentTimeMillis();
        return currentTime - lastUpdateTime;
    }
     private long timeSinceLastSpeak() {
        long currentTime = System.currentTimeMillis();
        return currentTime - lastSpeak;
    }
    private void setPosition(org.robokind.api.motion.Robot.JointId jointID, float val) {
      if (val<0) val=0.0f;
      if (val>1) val=1.0f;
      myGoalPositions.put(jointID, new NormalizedDouble(val)); 
    }
    
     

    private void moveQuizzical() {
        Animation anim = Robokind.loadAnimation("animations/neck_roll.xml");
       // AnimationJob introJob = myPlayer.playAnimation(introAnim);
        myPlayer.playAnimation(anim);
    
    }
    
    private void moveArms() {
        UserData user = mLastFrame.getUsers().get(0);
        if (user.getSkeleton().getState() == SkeletonState.TRACKED) {
            Point3f leftShoulder = convertPoint(user.getSkeleton().getJoint(JointType.LEFT_SHOULDER).getPosition());
            Point3f rightShoulder = convertPoint(user.getSkeleton().getJoint(JointType.RIGHT_SHOULDER).getPosition());
            Point3f torso = convertPoint(user.getSkeleton().getJoint(JointType.TORSO).getPosition());
            Point3f leftElbow = convertPoint(user.getSkeleton().getJoint(JointType.LEFT_ELBOW).getPosition());
            Point3f rightElbow = convertPoint(user.getSkeleton().getJoint(JointType.RIGHT_ELBOW).getPosition());
             Point3f leftHand = convertPoint(user.getSkeleton().getJoint(JointType.LEFT_HAND).getPosition());
            Point3f neck = convertPoint(user.getSkeleton().getJoint(JointType.NECK).getPosition());
            Point3f rightHand = convertPoint(user.getSkeleton().getJoint(JointType.RIGHT_HAND).getPosition());
            Point3f origin = new Point3f();
            float leftShoulderPitch = planeAngle(leftShoulder, rightShoulder, torso, leftShoulder, leftElbow);
            if (leftElbow.y > leftShoulder.y) {
                leftShoulderPitch = (float)Math.PI/2 - leftShoulderPitch;
            }
            float leftShoulderRoll = planeAngle(neck, torso, origin, leftShoulder, leftElbow);
                    
            float normPitch = (2.3f-leftShoulderPitch)/2.6f;
            float normRoll = (leftShoulderRoll-1.7f)/1.5f;
         
            setPosition(left_shoulder_pitch, normPitch);
            setPosition(left_shoulder_roll, normRoll);
            
            float leftElbowPitch = vectorAngle(leftElbow, leftShoulder, leftHand);
            
            normPitch = (float)(3 - leftElbowPitch) / (float)(Math.PI/2);
            setPosition(left_elbow_pitch, normPitch);
            
            float rightShoulderPitch = planeAngle(leftShoulder, rightShoulder, torso, rightShoulder, rightElbow);
            if (rightElbow.y > rightShoulder.y) {
                rightShoulderPitch = (float)Math.PI/2 - rightShoulderPitch;
            }
            
            normPitch = (2.3f-rightShoulderPitch)/2.6f;
            setPosition(right_shoulder_pitch, normPitch);
            float rightShoulderRoll = planeAngle(neck, torso, origin, rightShoulder, rightElbow);
            normRoll = (1.5f - rightShoulderRoll)/1.5f;
            setPosition(right_shoulder_roll, normRoll);
            
            
            float rightElbowPitch = vectorAngle(rightElbow, rightShoulder, rightHand);
            
            normPitch = (float)(3 - rightElbowPitch) / (float)(Math.PI/2);
            setPosition(right_elbow_pitch, normPitch);
            Quaternion leftElbowOrientation = user.getSkeleton().getJoint(JointType.LEFT_ELBOW).getOrientation();
            float angle = quatToAngle(leftElbowOrientation);
            
            if (angle>Math.PI) {
                 angle = (float)(2 * Math.PI - angle);
             }
            float normAngle = 1-angle/4.0f;
            
            setPosition(left_elbow_yaw, normAngle);
            
             Quaternion rightElbowOrientation = user.getSkeleton().getJoint(JointType.RIGHT_ELBOW).getOrientation();
            angle = quatToAngle(rightElbowOrientation);
               if (angle>Math.PI) {
                 angle = (float)(2 * Math.PI - angle);
             }
            normAngle = 1-angle/4.0f;
            setPosition(right_elbow_yaw, normAngle);
            
        }
    }
    
    private void trackHead() {
        
        UserData user = mLastFrame.getUsers().get(0);
        if (user.getSkeleton().getState() == SkeletonState.TRACKED) {
            

            Point3f head = convertPoint(user.getSkeleton().getJoint(JointType.HEAD).getPosition());
                           
            Point3f orig = new Point3f(0,0,0);
                            
            Point3f center_head = new Point3f(0,270,2100);
            Point3f center_feet = new Point3f(0,-270,2100);
                            
           float angle = planeAngle(orig, center_head, center_feet, orig, head);
            angle = (float)Math.cos(angle)*0.7f+0.6f;
             setPosition(neck_yaw, angle);
            Point3f zeno_head = new Point3f(0,-400,0);
            Point3f zeno_level = new Point3f(100,-400,0);
            Point3f zeno_level2 = new Point3f(0,-400,100);

            angle = planeAngle(zeno_head, zeno_level, zeno_level2, zeno_head, head);
            angle=0.75f-(float)Math.cos(angle);

            setPosition(neck_pitch, angle);
            RobotPositionMap myCurrentPositions = myRobot.getCurrentPositions();
            NormalizedDouble shoulder_pos = myCurrentPositions.get(right_shoulder_pitch);
            //NormalizedDouble neck_pos = myCurrentPositions.get(neck_roll);
            
            //float neck = (float)neck_pos.getValue();
            //float shoulder = (float)shoulder_pos.getValue();
             //float shoulder_dir = speed*12.0f*(0.5f-shoulder);
            //float neck_dir = (1.0f-speed)*12.0f*(0.5f-neck);
            
             
             
            setPosition(smile_left, 0.5f+speed*0.05f);
            setPosition(smile_right, 0.5f+speed*0.05f);
            
            //setPosition(neck_roll, 1.0f-speed);
            
           // setPosition(brows, 0.5f+speed*0.2f);
            //setPosition(eyelids, 0.5f+speed*0.5f);
            //setPosition(right_shoulder_pitch, shoulder+shoulder_dir);
            //setPosition(left_shoulder_pitch, shoulder+shoulder_dir);
          //neck_vel = (1.0f-speed);
          
            //setPosition(neck_roll, neck+neck_vel);
          
              /* if (speed>0.7) {
                mySpeaker.speak("whoopee");
            }*/
            /*if (speed>0.5 && timeSinceLastSpeak()>2000) {
                int spText = 100+(int)((speed-0.5)*60);
                mySpeaker.speak("\\VCT="+spText+"\\ oh");
                lastSpeak = System.currentTimeMillis();
            }*/
            
           
            
            
            
            
           
            
        }
    }
    
    private void drawLimb(Graphics g, int x, int y, UserData user, JointType from, JointType to) {
    	com.primesense.nite.SkeletonJoint fromJoint = user.getSkeleton().getJoint(from);
    	com.primesense.nite.SkeletonJoint toJoint = user.getSkeleton().getJoint(to);
    	
        /*if (fromJoint.getPositionConfidence() == 0.0 || toJoint.getPositionConfidence() == 0.0) {
            return;
    	}*/
    	
      
    	
    	com.primesense.nite.Point2D<Float> fromPos = mTracker.convertJointCoordinatesToDepth(fromJoint.getPosition());
    	com.primesense.nite.Point2D<Float> toPos = mTracker.convertJointCoordinatesToDepth(toJoint.getPosition());

       
    	
        
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
