package it.unisi.accompany.rosnodes;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.R;
import it.unisi.accompany.msgs_and_data.AccompanyAction;
import it.unisi.accompany.msgs_and_data.AccompanyActionRequest;
import it.unisi.accompany.msgs_and_data.AccompanyActionResponse;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;


import android.os.Handler;
import android.util.Log;

public class EmphasisClient implements NodeMain{

	protected final String TAG ="Accompany GUI - Emphasis (i.e. Squeeze-Me) Client";
	protected final double MAX_SPEED=1.0;
	
	protected ServiceClient<AccompanyActionRequest,AccompanyActionResponse> sc;
	protected AccompanyGUIApp myApp;
	
	protected double last_squeeze_speed;
	
	protected final double DEFAULT_SPEED=0.3;
	//values starting more or less from 0.1 and going up (usually 0.8 is very high)
	
	
	public EmphasisClient(AccompanyGUIApp a)
	{
		super();
		myApp=a;
		last_squeeze_speed=0.25;
	}
	
	public int getLastSqueezeSpeed()
	{
		if (last_squeeze_speed<0.18)
			return 1;
		if (last_squeeze_speed<0.26)
			return 2;
		if (last_squeeze_speed<0.34)
			return 3;
		if (last_squeeze_speed<0.42)
			return 4;
		if (last_squeeze_speed<0.50)
			return 5;
		if (last_squeeze_speed<0.58)
			return 6;
		if (last_squeeze_speed<0.66)
			return 7;
		if (last_squeeze_speed<0.74)
			return 8;
		if (last_squeeze_speed<0.82)
			return 9;
		return 10;
		
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e(TAG,"Error!");
	}

	@Override
	public void onShutdown(Node arg0) {
		if (sc!=null) sc.shutdown();
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode cn) {
		try{
			sc=cn.newServiceClient("EmphasisService", AccompanyAction._TYPE);
		}catch(Exception e){  //sarebbe ServiceNotFoundException
			Log.e(TAG,"Error in connecting to Accompany Emphasis Service!");
			if (sc!=null) sc.shutdown();
			myApp.closeAppOnError(myApp.getResources().getString(R.string.registration_error));
			//myApp.toastMessage("Emphasis server not found!");
			//throw new RosRuntimeException(e);
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("AccompanyGUIEmphasisClient");
	}
	
	public void setEmphasis(Double speed)
	{
		//check max
		if (speed>MAX_SPEED) speed=MAX_SPEED;
		//sending request to service
		setEmphasisRequest(speed);
	}
	
	protected void setEmphasisRequest(final Double speed)
	{
		if (sc!=null)
		{
			AccompanyActionRequest req =sc.newMessage();
			req.setAction(Double.toString(speed));
			req.setUid((long)(myApp.getUid()));
			sc.call(req, new ServiceResponseListener<AccompanyActionResponse>() {

				@Override
				public void onFailure(RemoteException arg0) {
					Log.e(TAG,"Richiesta fallita!!");
				}

				@Override
				public void onSuccess(AccompanyActionResponse arg0) {
					Log.v(TAG,"emphasis set result: "+arg0.getResult());
					Log.i(TAG,"squeeze speed setted to: "+speed);	
					last_squeeze_speed=speed;
					myApp.db_client.updateEmphasis(speed);
				}	
			});
		}
	}
	
	public void resetEmphasis()
	{
		//double speed= DEFAULT_SPEED;
		double speed=-1;
		//sending request to service
		if (sc!=null)
		{
			AccompanyActionRequest req =sc.newMessage();
			req.setAction(Double.toString(speed));
			req.setUid((long)(myApp.getUid()));
			sc.call(req, new ServiceResponseListener<AccompanyActionResponse>() {

				@Override
				public void onFailure(RemoteException arg0) {
					Log.e(TAG,"Richiesta fallita!!");
				}

				@Override
				public void onSuccess(AccompanyActionResponse arg0) {
					Log.v(TAG,"emphasis set result: "+arg0.getResult());	
					Log.i(TAG,"squeeze speed resetted to: "+DEFAULT_SPEED);	
					myApp.db_client.updateResetEmphasis(DEFAULT_SPEED);
					last_squeeze_speed=DEFAULT_SPEED;
				}	
			});
		}
	}

}
