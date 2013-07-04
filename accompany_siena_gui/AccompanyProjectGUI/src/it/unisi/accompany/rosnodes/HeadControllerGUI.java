package it.unisi.accompany.rosnodes;

import java.security.Timestamp;
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

import pr2_controllers_msgs.JointTrajectoryGoal;

import trajectory_msgs.JointTrajectoryPoint;

import android.util.Log;
public class HeadControllerGUI implements NodeMain{

	protected final String HEAD_CONTROL_DOWNSAMPLED_TOPIC = "head_controller/state_downsampled";
	public final double max_head_pos=0;
	public final double min_head_pos=-Math.PI;
	protected double head_pos; 
	protected double desired_head_pos;
	protected int my_seq_count;
	protected Publisher<pr2_controllers_msgs.JointTrajectoryActionGoal> p;
	protected Subscriber<pr2_controllers_msgs.JointTrajectoryControllerState> s;
	//protected Subscriber<trajectory_msgs.JointTrajectory> s;
	
	protected MessageFactory mf;
	
	protected long last_publish;
	protected Calendar calendar;
	
	protected boolean shouldBringHome=false;
	
	public double getHeadPos()
	{
		return head_pos;
	}
	
	public double getDesiredHeadPos()
	{
		return desired_head_pos;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("Accompany-GUI-HeadController","ERROR");
	}

	@Override
	public void onShutdown(Node arg0) {
		//s.shutdown();
		if (p!=null) p.shutdown();
		Log.e("Accompan-Ros","Shutdown Head controller");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode cn) {
		calendar= Calendar.getInstance();
		try{
			//s= cn.newSubscriber("head_controller/joint_trajectory_action/status",pr2_controllers_msgs.JointTrajectoryControllerState._TYPE);
			s= cn.newSubscriber(HEAD_CONTROL_DOWNSAMPLED_TOPIC,pr2_controllers_msgs.JointTrajectoryControllerState._TYPE);
			s.addMessageListener(new MessageListener<pr2_controllers_msgs.JointTrajectoryControllerState>() {
			      @Override
			      public void onNewMessage(final pr2_controllers_msgs.JointTrajectoryControllerState message) {
			        double[] a =message.getActual().getPositions();
			        head_pos=a[0];
			        if (calendar.getTimeInMillis()-last_publish>3000)
			        	desired_head_pos=head_pos;
			      }
			    });
		
		my_seq_count=0;
		//p= cn.newPublisher("head_controller/command",trajectory_msgs.JointTrajectory._TYPE);
		p= cn.newPublisher("head_controller/joint_trajectory_action/goal",pr2_controllers_msgs.JointTrajectoryActionGoal._TYPE);
		mf=cn.getTopicMessageFactory();
		
		//if already requested bring the camera to front
		if (shouldBringHome==true)
			bringCameraToFront();
		//bringCameraToFront();
		}catch(Exception e)
		{
			s.shutdown();
			p.shutdown();
			throw new RosRuntimeException(e);
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		
		return GraphName.of("AccompanyGUI/HeadController");
	}
	
	public double publish(double value) 
	{
		double diff=0;
		if (p==null)
		{
			Log.i("Accompany-GUI-HeadController"," Head controller not ready!");
			shouldBringHome=true;
			return 0;
		}
		pr2_controllers_msgs.JointTrajectoryActionGoal msg = p.newMessage();
		JointTrajectoryGoal gg=null;
		try{
			Log.i("HEAD","moving of value:"+value);
			 JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
			 gg=mf.newFromType(JointTrajectoryGoal._TYPE);
			 
			 double a[]= new double[1];
			 //desired_head_pos=head_pos;
			 if (head_pos > -(max_head_pos-min_head_pos)/2) desired_head_pos+=value;
			 else  desired_head_pos-=value;
			 //desired_head_pos-=value;
			 if (desired_head_pos > max_head_pos) 
			 {
				 diff=desired_head_pos - max_head_pos;
				 desired_head_pos = max_head_pos;
			 }
			 if (desired_head_pos < min_head_pos) 
			 {
				 diff=desired_head_pos - min_head_pos;
				 desired_head_pos = min_head_pos;
			 }
			 Log.i("HEAD","moving to:"+desired_head_pos);
			 a[0]= desired_head_pos;
			 pp.setPositions(a);
			 Duration d= new Duration();
			 d.secs=3;
			 d.nsecs=0;
			 pp.setTimeFromStart(d);
			 gg.getTrajectory().getPoints().add(pp);
			 gg.getTrajectory().getJointNames().add("head_axis_joint");
			 my_seq_count++;
		}catch(Exception e){
			Log.i("Accompany-GUI-HeadController"," cannot instantiate head message");
		}
		msg.setGoal(gg);
		p.publish(msg);
		last_publish= calendar.getTimeInMillis();
		return diff;
	}
	
	public void bringCameraToFront()
	{
		double diff=0;
		if (p==null)
		{
			Log.i("Accompany-GUI-HeadController"," Head controller not ready!");
			shouldBringHome=true;
			return;
		}
		pr2_controllers_msgs.JointTrajectoryActionGoal msg = p.newMessage();
		JointTrajectoryGoal gg=null;
		try{
			Log.i("HEAD","moving head to front");
			 JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
			 gg=mf.newFromType(JointTrajectoryGoal._TYPE);
			 
			 double a[]= new double[1];
			 //desired_head_pos=head_pos;
			 desired_head_pos=-Math.PI;
			 a[0]= desired_head_pos;
			 pp.setPositions(a);
			 Duration d= new Duration();
			 d.secs=3;
			 d.nsecs=0;
			 pp.setTimeFromStart(d);
			 gg.getTrajectory().getPoints().add(pp);
			 gg.getTrajectory().getJointNames().add("head_axis_joint");
			 my_seq_count++;
		}catch(Exception e){
			Log.i("Accompany-GUI-HeadController"," cannot instantiate head message");
		}
		msg.setGoal(gg);
		p.publish(msg);
		last_publish= calendar.getTimeInMillis();
		return;
	}
	
	public void bringCameraToFrontOld()
	{
		publish(-Math.PI);
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

/************************************************************************************************************************/

/*public class HeadControllerGUI implements NodeMain{

	protected final String TAG="AccompanyGUI-HeadController";
	
	public final double max_head_pos=0;
	public final double min_head_pos=-Math.PI;
	protected double head_pos; 
	protected double desired_head_pos;
	protected int my_seq_count;
	protected Publisher<trajectory_msgs.JointTrajectory> p;
	protected Subscriber<trajectory_msgs.JointTrajectory> s;
	
	protected MessageFactory mf;
	
	protected long last_publish;
	protected Calendar calendar;
	
	protected boolean shouldBringHome=false;
	
	public double getHeadPos()
	{
		return head_pos;
	}
	
	public double getDesiredHeadPos()
	{
		return desired_head_pos;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("Accompany-GUI-HeadController","ERROR");
	}

	@Override
	public void onShutdown(Node arg0) {
		s.shutdown();
		p.shutdown();
		Log.e("Accompan-Ros","Shutdown Head controller");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode cn) {
		calendar= Calendar.getInstance();
		try{
		s= cn.newSubscriber("head_controller/command",trajectory_msgs.JointTrajectory._TYPE);
		s.addMessageListener(new MessageListener<trajectory_msgs.JointTrajectory>() {
		      @Override
		      public void onNewMessage(final trajectory_msgs.JointTrajectory message) {
		        double[] a =message.getPoints().get(0).getPositions();
		        head_pos=a[0];
		        desired_head_pos=head_pos;
		      }
		    });
		
		my_seq_count=0;
		p= cn.newPublisher("head_controller/command",trajectory_msgs.JointTrajectory._TYPE);
		mf=cn.getTopicMessageFactory();
		}catch(Exception e)
		{
			s.shutdown();
			p.shutdown();
			throw new RosRuntimeException(e);
		}
		if (shouldBringHome) this.bringCameraToFront();
	}

	@Override
	public GraphName getDefaultNodeName() {
		
		return GraphName.of("AccompanyGUI/HeadController");
	}
	
	public double publish(double value) 
	{
		double diff=0;
		trajectory_msgs.JointTrajectory msg= p.newMessage();
		msg.getJointNames().add("head_axis_joint");
		try{
			 JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
			 double a[]= new double[1];
			 if (head_pos > -(max_head_pos-min_head_pos)/2) desired_head_pos+=value;
			 else  desired_head_pos-=value;
			 if (desired_head_pos > max_head_pos) 
			 {
				 diff=desired_head_pos - max_head_pos;
				 desired_head_pos = max_head_pos;
			 }
			 if (desired_head_pos < min_head_pos) 
			 {
				 diff=desired_head_pos - min_head_pos;
				 desired_head_pos = min_head_pos;
			 }
			 a[0]= desired_head_pos;
			 pp.setPositions(a);
			 Duration d= new Duration();
			 d.secs=3;
			 d.nsecs=0;
			 pp.setTimeFromStart(d);
			 msg.getPoints().add(pp);
			 msg.getHeader().setSeq(my_seq_count);
			 my_seq_count++;
		}catch(Exception e){
			Log.i("Accompany-GUI-HeadController"," cannot instantiate head message");
		}
		p.publish(msg);
		last_publish= calendar.getTimeInMillis();
		return diff;
	}
	
	public void bringCameraToFront()
	{
		if (p!=null)
		{
			Log.i(TAG,"Camera to fornt...");
			desired_head_pos=-Math.PI;
			trajectory_msgs.JointTrajectory msg= p.newMessage();
			msg.getJointNames().add("head_axis_joint");
			JointTrajectoryPoint pp=mf.newFromType(JointTrajectoryPoint._TYPE);
			double a[]= new double[1];
			a[0]=desired_head_pos;
			pp.setPositions(a);
			Duration d= new Duration();
			d.secs=3;
			d.nsecs=0;
			pp.setTimeFromStart(d);
			msg.getPoints().add(pp);
			msg.getHeader().setSeq(my_seq_count);
			my_seq_count++;
			p.publish(msg);
			Log.i("head msg: ",msg.toString());
			last_publish= calendar.getTimeInMillis();
		}
		else
			Log.e(TAG,"Cannot bring camera to front, missing publisher!");
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
*/