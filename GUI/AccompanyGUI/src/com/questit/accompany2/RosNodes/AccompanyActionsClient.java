package com.questit.accompany2.RosNodes;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import com.questit.accompany.msgs.AccompanyAction;
import com.questit.accompany.msgs.AccompanyActionRequest;
import com.questit.accompany.msgs.AccompanyActionResponse;
import com.questit.accompany2.AccompanyGUI_app2;

import android.os.Handler;
import android.util.Log;

public class AccompanyActionsClient implements NodeMain{

	protected ServiceClient<AccompanyActionRequest,AccompanyActionResponse> sc;
	protected AccompanyGUI_app2 myApp;
	protected Handler h;
	
	public AccompanyActionsClient(AccompanyGUI_app2 a,Handler hh)
	{
		super();
		myApp=a;
		h=hh;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("AccompanyGUI-ActionsClient","Error!");
	}

	@Override
	public void onShutdown(Node arg0) {
		sc.shutdown();
		
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
			sc=cn.newServiceClient("ActionsService", AccompanyAction._TYPE);
		}catch(Exception e){  //sarebbe ServiceNotFoundException
			Log.e("AccompanyGUI-ActionsClient","Error in connecting to Accompany Actions Service!");
			sc.shutdown();
			throw new RosRuntimeException(e);
		}
		//test
		AccompanyActionRequest req =sc.newMessage();
		req.setAction("say_connected");
		sc.call(req, new ServiceResponseListener<AccompanyActionResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				Log.e("AccompanyGUI-ActionClient","Richiesta fallita!!");
			}

			@Override
			public void onSuccess(AccompanyActionResponse arg0) {
				Log.e("AccompanyGUI-ActionClient",""+arg0.getResult());				
			}
			
		});
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("AccompanyGUIActionsClient");
	}

	public void sendRequest(String command)
	{
		//sending request to service
		AccompanyActionRequest req =sc.newMessage();
		req.setAction(command);
		req.setUid((long)(myApp.getUid()));
		sc.call(req, new ServiceResponseListener<AccompanyActionResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				Log.e("AccompanyGUI-ActionClient","Richiesta fallita!!");
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run() {
						myApp.handleFailedActionResponse();
					}
				});
			}

			@Override
			public void onSuccess(AccompanyActionResponse arg0) {
				Log.e("AccompanyGUI-ActionClient",""+arg0.getResult());	
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run() {
						myApp.handleActionResponse();
					}
				});
			}
					
		});
	}
	
	/*public void halt()
	{
		sc.shutdown();
	}*/
}
