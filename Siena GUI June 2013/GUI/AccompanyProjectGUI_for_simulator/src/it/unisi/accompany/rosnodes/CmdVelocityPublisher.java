package it.unisi.accompany.rosnodes;

import geometry_msgs.Vector3;

import org.ros.RosCore;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.util.Log;

public class CmdVelocityPublisher implements NodeMain{

	protected Publisher<geometry_msgs.Twist> p;
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("Cmd_vel Accompany","Error!!");
	}

	@Override
	public void onShutdown(Node arg0) {
		p.shutdown();
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}
	
	public int getMyPid()
	{
		return android.os.Process.myPid();
	}

	@Override
	public void onStart(ConnectedNode cn) {
		try{
		NameResolver nr= cn.getResolver().newChild("CmdVelPub");
		
		p= cn.newPublisher(nr.resolve("/base_controller/command"),geometry_msgs.Twist._TYPE);
		}catch(Exception e)
		{
			p.shutdown();
		}
	
		
	}

	@Override
	public GraphName getDefaultNodeName() {
		
		return GraphName.of("CmdVelPub");
	}
	
	public void  publish(float value)
	{
		geometry_msgs.Twist cmd= p.newMessage();
		cmd.getAngular().setZ(value);
		Log.i("Accompany-Publisher","published: ("+cmd.getLinear().getX()+","+cmd.getLinear().getY()+","+cmd.getLinear().getZ()+");("+cmd.getAngular().getX()+","+cmd.getAngular().getY()+","+cmd.getAngular().getZ()+")");
		p.publish(cmd);
	}

}
