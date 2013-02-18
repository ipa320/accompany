package com.questit.accompany2.RosNodes;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import com.questit.accompany.msgs.AccompanyAction;
import com.questit.accompany.msgs.AccompanyActionResponse;
import com.questit.accompany.msgs.AccompanyDBmsg;
import com.questit.accompany.msgs.AccompanyDBmsgRequest;
import com.questit.accompany.msgs.AccompanyDBmsgResponse;
import com.questit.accompany2.AccompanyGUI_app2;

import android.os.Handler;
import android.util.Log;

public class DatabaseClient implements NodeMain{

	protected ServiceClient<AccompanyDBmsgRequest,AccompanyDBmsgResponse> sc;
	protected AccompanyGUI_app2 app;
	protected Handler h;
	
	public DatabaseClient(AccompanyGUI_app2 a,Handler hh)
	{
		super();
		app=a;
		h=hh;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("AccompanyGUI-DBclient","Error!!!!");
		h.postAtFrontOfQueue(new Runnable(){
			@Override
			public void run()
			{
					app.closeAppOnError();
			}});
	}

	@Override
	public void onShutdown(Node arg0) {
		sc.shutdown();
		Log.e("Accompany-problem","shutdown db client");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	public boolean isStarted()
	{
		if (sc==null) return false;
		else return true;
	}
	
	@Override
	public void onStart(ConnectedNode cn) {
		
		try{
			sc=cn.newServiceClient("DatabaseConnectorService", AccompanyDBmsg._TYPE);
		}catch(Exception e){  //sarebbe ServiceNotFoundException
			Log.e("AccompanyGUI-ActionsClient","Error in connecting to Accompany Actions Service!");
			sc.shutdown();
			throw new RosRuntimeException(e);
		}
	}
	
	public void request(int code, String par, String son)
	{
		AccompanyDBmsgRequest msg= sc.newMessage();
		msg.setRequest(code);
		msg.setParam(par);
		msg.setSonReq(Integer.parseInt(son));
		sc.call(msg, new ServiceResponseListener<AccompanyDBmsgResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				Log.e("AccompanyGUI-DBClient","Richiesta fallita!!");
			}

			@Override
			public void onSuccess(final AccompanyDBmsgResponse arg0) {
				Log.e("AccompanyGUI-DBClient",""+arg0.getAnswer());
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run()
					{
						app.handleResponse(arg0.getAnswer(),(int)arg0.getCode(),(int)arg0.getSonRes());
					}
				});
				
			}
			
		});
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("AccompanyGUI/DBclient");
	}
	
	//return the location of the user (get it from the database)
	public String getUserLocation()
	{
		return "502";  //by now, for debugging
	}
	
	//return the location of robot (get it from the database)
	public String getRobotLocation()
	{
		return "502";  //by now, for debug
	}

	//login request
	public void login(String user,String pwd)
	{
		AccompanyDBmsgRequest msg= sc.newMessage();
		msg.setRequest(0);
		msg.setParam(user);
		msg.setSonReq(Integer.parseInt("0"));
		sc.call(msg, new ServiceResponseListener<AccompanyDBmsgResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				Log.e("AccompanyGUI-DBClient","Richiesta fallita!!");
			}

			@Override
			public void onSuccess(final AccompanyDBmsgResponse arg0) {
				Log.e("AccompanyGUI-DBClient",""+arg0.getAnswer());
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run()
					{
						app.handleLoginResponse(arg0.getAnswer(),(int)arg0.getCode(),(int)arg0.getSonRes());
					}
				});
				
			}
			
		});
	}
	
}
