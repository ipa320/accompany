package it.unisi.accompany.rosnodes;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.R;

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
	
	protected final String TAG= "AccompanyGUI-TorsoController";
	
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
	
	protected int cob_version;
	protected AccompanyGUIApp myApp;
	
	protected boolean shouldBringHome=false;
	
	/*public TorsoControllerGUI(int v)
	{
		String cv="";
		if (v==AccompanyGUIApp.COB32) cv="Cob 3-2";
		else cv="Cob 3-6";
		Log.v(TAG, "Creating...");
		Log.v(TAG,"Cob Version --> "+cv);
		cob_version=v;
	}*/
	
	public TorsoControllerGUI(AccompanyGUIApp a, int v)
	{
		myApp=a;
		String cv="";
		if (v==AccompanyGUIApp.COB32) cv="Cob 3-2";
		else cv="Cob 3-6";
		Log.v(TAG, "Creating...");
		Log.v(TAG,"Cob Version --> "+cv);
		cob_version=v;
	}
	
	public void setCobVersion(int v)
	{
		//log infos
		String cv="";
		if (v==AccompanyGUIApp.COB32) cv="Cob 3-2";
		else cv="Cob 3-6";
		Log.v(TAG, "setting Cob Version --> "+cv);
		
		//managing different sizes of the vectors for the 2 versions:
		switch(v)
		{
			case AccompanyGUIApp.COB32:
			{
				if (cob_version==AccompanyGUIApp.COB36)
				{
					double[] oldt = torso_pos;
					torso_pos= new double[4];
					torso_pos[0]=oldt[0];
					torso_pos[1]=oldt[1];
					torso_pos[2]=oldt[0];
					torso_pos[3]=oldt[2];
					oldt= desired_torso_pos;
					desired_torso_pos= new double[4];
					desired_torso_pos[0]=oldt[0];
					desired_torso_pos[1]=oldt[1];
					desired_torso_pos[2]=oldt[0];
					desired_torso_pos[3]=oldt[2];
				}
			} break;
			case AccompanyGUIApp.COB36:
			{
				if (cob_version==AccompanyGUIApp.COB32)
				{
					double[] oldt = torso_pos;
					torso_pos= new double[3];
					torso_pos[0]=oldt[0];
					torso_pos[1]=oldt[1];
					torso_pos[2]=oldt[3];
					oldt= desired_torso_pos;
					desired_torso_pos= new double[3];
					desired_torso_pos[0]=oldt[0];
					desired_torso_pos[1]=oldt[1];
					desired_torso_pos[2]=oldt[3];
				}
			} break;
			default: break;
		}
		
		//real update
		cob_version=v;
	}
	
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
		Log.e(TAG,"Error: Torso controller error!");
myApp.closeAppOnError(myApp.getResources().getString(R.string.comunication_error));
		
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
			String cv="";
			if (cob_version== AccompanyGUIApp.COB32) cv="Cob 3-2";
			else cv="Cob 3-6";
			Log.v(TAG,"Starting torso controller, (Cob Version:"+cv+")");
			calendar= Calendar.getInstance();
			my_seq_count=0;
			p= cn.newPublisher("torso_controller/command",trajectory_msgs.JointTrajectory._TYPE);
			mf=cn.getTopicMessageFactory();
		}catch(Exception e)
		{
			p.shutdown();
			myApp.toastMessage("Cannot find TorsoController!");
			myApp.closeAppOnError(myApp.getResources().getString(R.string.registration_error));
			throw new RosRuntimeException(e);
		}
		if (cob_version== AccompanyGUIApp.COB32)
		{
			desired_torso_pos = new double[4];
			torso_pos= new double[4];
		
			for (int h=0;h<4;h++)
				torso_pos[h]=0.;
			
		}
		else
		{
			desired_torso_pos = new double[3];
			torso_pos= new double[3];
		
			for (int h=0;h<3;h++)
				torso_pos[h]=0.;
		}
		
		if (shouldBringHome)
		{
			shouldBringHome=false;
			bringHome();
		}
	}
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("TorsoControllerGUI");
	}

	public void publish(double difference)
	{
		Log.v(TAG,"Publishing: "+difference);
		trajectory_msgs.JointTrajectory msg= p.newMessage();
		if (cob_version==AccompanyGUIApp.COB32)
		{
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
				 //msg.getHeader().setSeq(my_seq_count);
				 my_seq_count++;
		}
		else  //cob 3-6
		{
			
			msg.getJointNames().add("torso_pan_joint");
			msg.getJointNames().add("torso_lower_neck_tilt_joint");
			msg.getJointNames().add("torso_upper_neck_tilt_joint");
			//try{
				 JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
				 double a[]= new double[3];
				
				 desired_torso_pos[1]+=difference;
				 desired_torso_pos[2]+=difference*(0.1499/0.25);
				 if (desired_torso_pos[1]<min_torso_pos[1])
					 desired_torso_pos[1]=min_torso_pos[1];
				 if(desired_torso_pos[2]<min_torso_pos[2])
					 desired_torso_pos[2]=min_torso_pos[2];
				 if (desired_torso_pos[1]>max_torso_pos[1])
					 desired_torso_pos[1]=max_torso_pos[1];
				 if(desired_torso_pos[2]>max_torso_pos[2])
					 desired_torso_pos[2]=max_torso_pos[2];
				 a[0]=torso_pos[0];
				 a[1]=desired_torso_pos[1];
				 a[2]=desired_torso_pos[2];
				 pp.setPositions(a);
				 
				 Duration d= new Duration();
				 d.secs=3;
				 d.nsecs=0;
				 pp.setTimeFromStart(d);
				 msg.getPoints().add(pp);
				 //msg.getHeader().setSeq(my_seq_count);
				 //my_seq_count++;
		}
	    p.publish(msg);
		last_publish= calendar.getTimeInMillis();
	}
	
	public void bringHome()
	{
		Log.v(TAG,"Torso --> Home");
		if (p!=null)
		{
			trajectory_msgs.JointTrajectory msg= p.newMessage();
			if (cob_version==AccompanyGUIApp.COB32)
			{
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
				//msg.getHeader().setSeq(my_seq_count);
				my_seq_count++;
			}
			else //cob 3-6
			{
				msg.getJointNames().add("torso_lower_neck_tilt_joint");
				msg.getJointNames().add("torso_pan_joint");
				msg.getJointNames().add("torso_upper_neck_tilt_joint");
				JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
				double a[]= new double[3];
				a[0]=0.; 
				a[1]=0.;
				a[2]=0.;
				pp.setPositions(a);
				
				Duration d= new Duration();
				d.secs=3;
				d.nsecs=0;
				pp.setTimeFromStart(d);
				msg.getPoints().add(pp);
				//msg.getHeader().setSeq(my_seq_count);
				//my_seq_count++;
			}
			 
			
			p.publish(msg);
			last_publish= calendar.getTimeInMillis();
			
			//torso_pos=a;
		}
		else
		{
			Log.e(TAG,"Cannot bring torso home -  missing publisher");
		}
	}
	
	public int getMyPid()
	{
		return android.os.Process.myPid();
	}
	
	public void shouldBringHome(boolean b)
	{
		shouldBringHome=b;
	}
}
