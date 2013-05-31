package it.unisi.accompany.threads;

import it.unisi.accompany.AccompanyGUIApp;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.params.BasicHttpParams;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;

import android.os.Handler;
import android.util.Log;

import com.loopj.android.http.AsyncHttpClient;

public class RobotStatusThread extends Thread {
	
	protected final String TAG="AccompanyGUI-RobotStatusControllerThread";
	protected String url="http://10.0.1.5:9996/";
	
	protected boolean interrupt;
	protected AccompanyGUIApp app;
	protected Handler h;
	
	protected int retry_count;
	
	protected String old_result;
	protected String current_status;
	protected String current_task;
	//protected int current_status;
	protected ArrayList<String> next_tasks;
	
	public RobotStatusThread(AccompanyGUIApp a, Handler hh,String ip, String port)
	{
		this.app=a;
		this.interrupt=false;
		this.h=hh;
		this.current_status="-1.0";
		this.current_task=null;
		//client
		this.next_tasks= new ArrayList<String>();
		
		this.retry_count=0;
		
		createBaseUrl(ip,port);
	}
	
	public void createBaseUrl(String ip, String prt)
	{
		url="http://"+ip+":"+prt+"/";
	}

	protected void getCurrentStatus()
	{
		InputStream is=null;
		String result="";
		
		try{
			HttpParams httpParameters = new BasicHttpParams();
	    	int timeoutConnection = 5000;  //the proper connection timeout
	    	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
	    	int timeoutSocket = 5000;      //the operation timeout
	    	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
	    	
	    	HttpClient hc= new DefaultHttpClient(httpParameters); 
	    	HttpPost hp= new HttpPost(url+"current");
	    	Log.i(TAG,"starting connection for current status...");
	    	HttpResponse response = hc.execute(hp);
	    	Log.i(TAG,"Executed the query");
	    	HttpEntity entity = response.getEntity();
	    	is = entity.getContent();
	    	
	    	if (is!=null)
			{
				try{
					 BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
			         StringBuilder sb = new StringBuilder();
			         String line = null;
			         while ((line = reader.readLine()) != null) {
			        	 sb.append(line + "\n");
			         }
			         is.close();
		
			         result=sb.toString();
				}catch(Exception e)
				{
					Log.e(TAG,"error parisng string!");
				}
			}
	    	
	    	Log.e(TAG,"current status result "+result);
	    	if (!(result.equals("")||result.equals("error")||result.equals("-1")))
	    	{
	    		old_result=result;
	    		String[] splitted=result.split(",");
	    		current_status=splitted[0];
	    		current_task=splitted[1];
	    		
	    		if (current_status.equals("1.0")||current_status.equals("1"))
	    		{
	    			h.postAtFrontOfQueue(new Runnable()
	    			{

						@Override
						public void run() {
							app.showRobotExecutingCommandView();
						}
	    			});
	    		}
	    		else if (current_status.equals("0.0")||current_status.equals("0"))
	    		{
	    			//do nothing you are already in user view!
	    		}
	    	}
	    	Log.e(TAG,"current status: "+current_status+", current task: "+current_task);
	    	
	    	
		}catch(Exception e)
		{
			Log.e(TAG,"Http connection error... retrying in 10 sec...");
			try {
				Thread.sleep(10000);
			} catch (InterruptedException e1) {
				Log.e(TAG,"Thread sleeping problem!");
			}
			retry_count++;
			if (retry_count<10) this.getCurrentStatus();
		}
		retry_count=0;
	}
	
	protected void getNextStatus()
	{
		InputStream is=null;
		String result="";
		
		ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
	    nameValuePairs.add(new BasicNameValuePair("current",current_status));
	    nameValuePairs.add(new BasicNameValuePair("task",current_task));
		
		try{
			HttpParams httpParameters = new BasicHttpParams();
	    	int timeoutConnection = 100000;  //the proper connection timeout
	    	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
	    	int timeoutSocket = 100000;      //the operation timeout
	    	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
	    	
	    	HttpClient hc= new DefaultHttpClient(httpParameters); 
	    	HttpPost hp= new HttpPost(url+"next");
	    	//HttpGet hp= new HttpGet(URL+"next");
	    	Log.i(TAG,"starting connection for next status...");
	    	
	    	hp.setEntity(new UrlEncodedFormEntity(nameValuePairs));
	    	HttpResponse response = hc.execute(hp);
	    	Log.i(TAG,"Executed the query");
	    	HttpEntity entity = response.getEntity();
	    	is = entity.getContent();
	    	
	    	if (is!=null)
			{
				try{
					 BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
			         StringBuilder sb = new StringBuilder();
			         String line = null;
			         while ((line = reader.readLine()) != null) {
			        	 sb.append(line + "\n");
			         }
			         is.close();
		
			         result=sb.toString();
				}catch(Exception e)
				{
					Log.e(TAG,"error parisng string!");
				}
			}
	    	
	    	Log.e(TAG,"next status result: "+result);
	    	if (!(result.equals("")||result.equals("error")||result.equals("-1")))
	    		if (!result.equals(old_result)) 
	    		{
	    			old_result=result;
	    			String[] splitted=result.split(",");
	    			current_status=splitted[0];
	    			current_task=splitted[1];
	    			Log.e(TAG,"splitted");
	    			
	    			if (current_status.equals("1.0")||current_status.equals("1"))
		    		{
		    			h.postAtFrontOfQueue(new Runnable()
		    			{
							@Override
							public void run() {
								Log.i("123","thrad: go working");
								app.showRobotExecutingCommandView();
							}
		    			});
		    		}
		    		else if (current_status.equals("0.0")||current_status.equals("0"))
		    		{
		    			//switch back to old view
		    			h.postAtFrontOfQueue(new Runnable()
		    			{
							@Override
							public void run() {
								Log.i("123","thrad: go back from working");
								app.handleActionResponse();
							}
		    			});
		    		}
	    			
	    		}
	    	Log.e(TAG,"next status: "+current_status+", next task: "+current_task);
	    	
		}catch(Exception e)
		{
			Log.e(TAG,"Http connection error... retrying in 10 sec...");
			try {
				Thread.sleep(10000);
			} catch (InterruptedException e1) {
				Log.e(TAG,"Thread sleeping problem!");
			}
		}
	}
	
	public String getCurrentTask()
	{
		return current_task.replace("Siena", "").trim();
	}
	
	public ArrayList<String> getNextActions()
	{
		return next_tasks;
	}
	
	@Override
	public void start()
	{
		super.start();
	}
	
	public void halt()
	{
		this.interrupt=true;
		//interrupt();
	}
	
	@Override
	public void run()
	{
		getCurrentStatus();
		while(!interrupt)
		{
			getNextStatus();
		}
	}

}
