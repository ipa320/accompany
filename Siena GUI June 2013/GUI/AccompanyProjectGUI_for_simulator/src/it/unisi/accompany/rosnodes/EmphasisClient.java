package it.unisi.accompany.rosnodes;

import it.unisi.accompany.AccompanyGUIApp;
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
	
	protected ServiceClient<AccompanyActionRequest,AccompanyActionResponse> sc;
	protected AccompanyGUIApp myApp;
	
	public EmphasisClient(AccompanyGUIApp a)
	{
		super();
		myApp=a;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e(TAG,"Error!");
	}

	@Override
	public void onShutdown(Node arg0) {
		sc.shutdown();
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
			myApp.closeAppOnError("Cannoct connect to emphaisis server!");
			throw new RosRuntimeException(e);
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("AccompanyGUIEmphasisClient");
	}
	
	public void setEmphasis(Double speed)
	{
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
					Log.i(TAG,"emphasis set result: "+arg0.getResult());	
				}	
			});
		}
	}

}
