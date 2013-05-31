package it.unisi.accompany.clients;

import it.unisi.accompany.AccompanyGUIApp;

import java.io.InputStream;
import java.util.ArrayList;

import org.apache.http.NameValuePair;
import org.apache.http.message.BasicNameValuePair;

import com.loopj.android.http.AsyncHttpClient;
import com.loopj.android.http.AsyncHttpResponseHandler;
import com.loopj.android.http.RequestParams;


import android.os.Handler;
import android.util.Log;

public class DatabaseClient {//extends Thread{

	protected final String TAG= "AccompanyGUI-DBClient";
	
	protected AccompanyGUIApp app;
	protected Handler h;
	
	protected static AsyncHttpClient client= new AsyncHttpClient();
	//protected static String BASEURL="http://10.0.1.5:9995/";
	
	protected final int CMDCODE=999;
	protected String baseurl;
	
	public DatabaseClient(AccompanyGUIApp a,Handler hh,String ip,String port)
	{
		super();
		app=a;
		h=hh;
		client.setTimeout(1000000);
		createBaseUrl(ip,port);
	}
	
	public void createBaseUrl(String add,String prt)
	{
		baseurl="http://"+add+":"+prt+"/";
		Log.i(TAG,"Create url("+add+","+prt+"): "+baseurl);
	}
	
	public void get(String url, RequestParams pars, AsyncHttpResponseHandler rh)
	{
		client.get(baseurl+url, pars,rh);
	}
	
	//return the location of the user (get it from the database)
	public String getUserId()
	{
		return Integer.toString(app.getUid());  //by now, for debugging
	}
	

	//login request
	public void login(String user,String pwd)
	{
		RequestParams my_par= new RequestParams();
		my_par.put("unick",user);
		
		DatabaseClient.this.get("login", my_par, new AsyncHttpResponseHandler(){
			    @Override
			    public void onSuccess(String response)
			    {
			    	Log.i("Accompany GUI - Login reponse",response);
			    	final String[] log_res=response.split(",");
			    	Log.i("Accompany GUI - Login reponse","uid:"+log_res[0]+", lang: "+log_res[1]);
			    	final int uid=Integer.parseInt(log_res[0]);
			    	final int ulang=Integer.parseInt(log_res[1]);
			    	h.postAtFrontOfQueue(new Runnable(){
			    		@Override
			    		public void run()
			    		{
			    			app.handleLoginResponse(log_res[0],uid,ulang);
			    		}
			    	});
			    }
			    
			    @Override
			    public void onFailure(Throwable e,final String response)
			    {
			    	h.postAtFrontOfQueue(new Runnable() {
						
						@Override
						public void run() {
							Log.e("Accompany GUI - Login reponse","ERROR! - resp: "+response);
					    	app.toastMessage("Cannot connect to db Host! please check Database Ip and port in settings!");
						}
					});
			    	
			    }
		});
				
	}
	
	public void getFullActionList()
	{
		DatabaseClient.this.get("full_action_list", null, new AsyncHttpResponseHandler(){
		    @Override
		    public void onSuccess(final String response)
		    {
		    	Log.i("Accompany GUI - Full actions list request reponse: ",response);
		    	h.postAtFrontOfQueue(new Runnable(){
		    		@Override
		    		public void run()
		    		{
		    			app.handleFullActionsListResponse(response);
		    		}
		    	});
		    }
		    
		    @Override
		    public void onFailure(Throwable e,String response)
		    {
		    	Log.e("Accompany GUI - Fulla action list request reponse","ERROR! - resp: "+response);
		    	app.closeAppOnError("Cannot connect to db! Closing...");
		    }
		});
	}
	
	public void thread_request(final int code, String uid, String ulang)
	{
		Log.i("Accompany db client - actions request","starting "+code);
		RequestParams my_par= new RequestParams();
		if (code==app.USER_ACTIONS_REQUEST_CODE) my_par.put("uid",uid);
		my_par.put("ulang", ulang);
		String url="";
		if (code==app.USER_ACTIONS_REQUEST_CODE) url="user_actions";
		else if (code==app.ROBOT_ACTIONS_REQUEST_CODE)
				url="robot_actions";
			else return;
		
		DatabaseClient.this.get(url, my_par, new AsyncHttpResponseHandler(){
		    @Override
		    public void onSuccess(final String response)
		    {
		    	Log.i("Accompany GUI - Actions request reponse: ",response);
		    	h.postAtFrontOfQueue(new Runnable(){
		    		@Override
		    		public void run()
		    		{
		    			app.handleResponse(response,code);
		    		}
		    	});
		    }
		    
		    @Override
		    public void onFailure(Throwable e,String response)
		    {
		    	Log.e("Accompany GUI - Actions request reponse","ERROR! - resp: "+response);
		    	app.closeAppOnError("Cannot connect to db! Closing...");
		    }
		});
	}
	
	public void request(final int code, String uid, String ulang)
	{
		Log.i("Accompany db client - actions request","starting "+code);
		RequestParams my_par= new RequestParams();
		if (code==app.USER_ACTIONS_REQUEST_CODE) my_par.put("uid",uid);
		my_par.put("ulang", ulang);
		String url="";
		if (code==app.USER_ACTIONS_REQUEST_CODE) url="user_actions";
		else if (code==app.ROBOT_ACTIONS_REQUEST_CODE)
				url="robot_actions";
			else return;
		
		DatabaseClient.this.get(url, my_par, new AsyncHttpResponseHandler(){
		    @Override
		    public void onSuccess(final String response)
		    {
		    	Log.i("Accompany GUI - Actions request reponse: ",response);
		    	h.postAtFrontOfQueue(new Runnable(){
		    		@Override
		    		public void run()
		    		{
		    			app.handleResponse(response,code);
		    		}
		    	});
		    }
		    
		    @Override
		    public void onFailure(Throwable e,String response)
		    {
		    	Log.e("Accompany GUI - Actions request reponse","ERROR! - resp: "+response);
		    	app.closeAppOnError("Cannot connect to db! Closing...");
		    }
		});
	}
	
	/*public void requestSons(final int id, String ulang)
	{
		RequestParams my_par= new RequestParams();
		my_par.put("pid",Integer.toString(id));
		my_par.put("ulang", ulang);
		String url="";
		
		DatabaseClient.get("sons_actions", my_par, new AsyncHttpResponseHandler(){
		    @Override
		    public void onSuccess(final String response)
		    {
		    	Log.i("Accompany GUI - Actions request reponse: ",response);
		    	h.postAtFrontOfQueue(new Runnable(){
		    		@Override
		    		public void run()
		    		{
		    			app.handleSonsResponse(response,id);
		    		}
		    	});
		    }
		    
		    @Override
		    public void onFailure(Throwable e,String response)
		    {
		    	Log.e("Accompany GUI - Actions request reponse","ERROR! - resp: "+response);
		    }
		});
	}*/
	
	public void requestOptions(final int id, String ulang)
	{
		RequestParams my_par= new RequestParams();
		my_par.put("pid",Integer.toString(id));
		my_par.put("ulang", ulang);
		String url="";
		DatabaseClient.this.get("options", my_par, new AsyncHttpResponseHandler(){
		    @Override
		    public void onSuccess(final String response)
		    {
		    	Log.i("Accompany GUI - Options request reponse: ",response);
		    	h.postAtFrontOfQueue(new Runnable(){
		    		@Override
		    		public void run()
		    		{
		    			app.handleOptionsResponse(response,id);
		    		}
		    	});
		    }
		    
		    @Override
		    public void onFailure(Throwable e,String response)
		    {
		    	Log.e("Accompany GUI - Options request reponse","ERROR! - resp: "+response);
		    	app.closeAppOnError("Cannot connect to db! Closing...");
		    }
		});
		
	}
	
	public void getExpression()
	{
		Log.i("AccompanyGUI-DBClient - expression","starting request");
		DatabaseClient.this.get("expression_request", null, new AsyncHttpResponseHandler(){

			@Override
			public void onSuccess(final String response) {
				Log.i("AccompanyGUI-DBClient - expression response",response);
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run()
					{
						app.handleExpressionResponse(response);
					}
				});
			}
			
			 @Override
			    public void onFailure(Throwable e,String response)
			    {
			    	Log.e("Accompany GUI - Expression reponse","ERROR! - resp: "+response);
			    	app.closeAppOnError("Cannot connect to db! Closing...");
			    }
		});

	}
	
	
	public void sendCommand(int id)
	{
		RequestParams my_par= new RequestParams();
		my_par.put("cmd_id",Integer.toString(id));
		Log.i("Accompany GUI db client-","send request - "+id);
		DatabaseClient.this.get("command", my_par, new AsyncHttpResponseHandler(){
			@Override
			public void onFailure(Throwable e, String response) {
				Log.e("AccompanyGUI-DBClient","Command - failed request!!");
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run() {
						app.handleFailedActionResponse();
					}
				});
			}

			@Override
			public void onSuccess(final String response) {
				Log.e("AccompanyGUI-DBClient command","response: "+response);
				h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run()
					{
						h.postAtFrontOfQueue(new Runnable(){
							@Override
							public void run() {
								//app.handleActionResponse();
								Log.i("AccompanyGUI-DBclient","Command - successfull request!!");
							}
						});
					}
				});
				
			}
		});
	}
	
	public void requestParameterSet(int id,String value)
	{
		RequestParams my_par= new RequestParams();
		my_par.put("opt_id",Integer.toString(id));
		my_par.put("val", value);
		Log.i("Accompany GUI db client-","send parameter id-> "+id+" value-> "+value);
		DatabaseClient.this.get("setparameter", my_par, new AsyncHttpResponseHandler(){
			@Override
			public void onFailure(Throwable e, String response) {
				Log.e("AccompanyGUI-DBClient","Param - failed request!!");
				/*h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run() {
						app.handleFailedActionResponse();
					}
				});*/
			}

			@Override
			public void onSuccess(final String response) {
				Log.e("AccompanyGUI-DBClient","parameter response: "+response);
				/*h.postAtFrontOfQueue(new Runnable(){
					@Override
					public void run()
					{
						h.postAtFrontOfQueue(new Runnable(){
							@Override
							public void run() {
								app.handleActionResponse();
								Log.i("AccompanyGUI-DBclient","Command - successfull request!!");
							}
						});
					}
				});*/
				
			}
		});
	}
}
