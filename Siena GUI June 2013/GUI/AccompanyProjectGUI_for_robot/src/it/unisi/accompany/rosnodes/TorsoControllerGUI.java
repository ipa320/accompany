package it.unisi.accompany.rosnodes;

import java.util.Calendar;

import org.ros.exception.RosRuntimeException;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import trajectory_msgs.JointTrajectoryPoint;

import android.util.Log;

public class TorsoControllerGUI implements NodeMain{
	
	public final double[] max_torso_pos={0.0,0.15,0.0,0.25}; //cob 3-2
	public final double[] min_torso_pos={0.0,-0.15,0.0,-0.25}; //cob 3-2
	//public final double[] max_torso_pos={0.10,0.0,0.1499}; //cob 3-3
	//public final double[] min_torso_pos={-0.10,0.0,-0.1499}; //cob 3-3
	protected double[] torso_pos; 
	protected double[] desired_torso_pos;
	protected int my_seq_count;
	protected Publisher<trajectory_msgs.JointTrajectory> p;
	protected Subscriber<pr2_controllers_msgs.JointTrajectoryControllerState> s;
	protected MessageFactory mf;
	
	protected long last_publish;
	protected Calendar calendar;
	
	public double[] getTorsoPos()
	{
		return torso_pos;
	}
	
	public double[] getDesiredTorsoPos()
	{
		return desired_torso_pos;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("AccompanyGUI-Torso controller","Error: Torso controller error!");
	}
	@Override
	public void onShutdown(Node arg0) {
		Log.e("Accompany-Ros","shutdown torso controller...");
		p.shutdown();
	}
	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void onStart(ConnectedNode cn) {
		try{
		calendar= Calendar.getInstance();
		my_seq_count=0;
		p= cn.newPublisher("torso_controller/command",trajectory_msgs.JointTrajectory._TYPE);
		mf=cn.getTopicMessageFactory();
		}catch(Exception e)
		{
			p.shutdown();
			throw new RosRuntimeException(e);
		}
		desired_torso_pos = new double[4];
		torso_pos= new double[4];
		
		for (int h=0;h<4;h++)
			torso_pos[h]=0.;
	}
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("TorsoControllerGUI");
	}

	public void publish(double difference)
	{
		trajectory_msgs.JointTrajectory msg= p.newMessage();
		msg.getJointNames().add("torso_lower_neck_pan_joint");
		msg.getJointNames().add("torso_lower_neck_tilt_joint");
		msg.getJointNames().add("torso_upper_neck_pan_joint");
		msg.getJointNames().add("torso_upper_neck_tilt_joint");
		//try{
			 JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
			 double a[]= new double[4];
			
			 desired_torso_pos[1]+=difference;
			 desired_torso_pos[3]+=difference*(0.1499/0.25);
			 if (desired_torso_pos[1]<min_torso_pos[1])
				 desired_torso_pos[1]=min_torso_pos[1];
			 if(desired_torso_pos[3]<min_torso_pos[3])
				 desired_torso_pos[3]=min_torso_pos[3];
			 if (desired_torso_pos[1]>max_torso_pos[1])
				 desired_torso_pos[1]=max_torso_pos[1];
			 if(desired_torso_pos[3]>max_torso_pos[3])
				 desired_torso_pos[3]=max_torso_pos[3];
			 a[0]=torso_pos[0];
			 a[1]=desired_torso_pos[1];
			 a[2]=torso_pos[2];
			 a[3]=desired_torso_pos[3];
			 pp.setPositions(a);
			 
			 Duration d= new Duration();
			 d.secs=3;
			 d.nsecs=0;
			 pp.setTimeFromStart(d);
			 msg.getPoints().add(pp);
			 msg.getHeader().setSeq(my_seq_count);
			 my_seq_count++;
	    p.publish(msg);
		last_publish= calendar.getTimeInMillis();
	}
	
	public void bringHome()
	{
		trajectory_msgs.JointTrajectory msg= p.newMessage();
		msg.getJointNames().add("torso_lower_neck_pan_joint");
		msg.getJointNames().add("torso_lower_neck_tilt_joint");
		msg.getJointNames().add("torso_upper_neck_pan_joint");
		msg.getJointNames().add("torso_upper_neck_tilt_joint");
		JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
		double a[]= new double[4];
		a[0]=0.; 
		a[1]=0.;
		a[2]=0.;
		a[3]=0.;
		pp.setPositions(a);
		 
		Duration d= new Duration();
		d.secs=3;
		d.nsecs=0;
		pp.setTimeFromStart(d);
		msg.getPoints().add(pp);
		msg.getHeader().setSeq(my_seq_count);
		my_seq_count++;
		p.publish(msg);
		last_publish= calendar.getTimeInMillis();
		
		torso_pos=a;
	}
	
	public int getMyPid()
	{
		return android.os.Process.myPid();
	}
	
}
