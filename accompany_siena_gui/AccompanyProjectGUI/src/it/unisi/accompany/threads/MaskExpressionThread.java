package it.unisi.accompany.threads;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.clients.DatabaseClient;

import java.util.Hashtable;

import android.graphics.drawable.Drawable;
import android.os.Handler;
import android.util.Log;

public class MaskExpressionThread extends Thread{
	
	protected String TAG = "AccompanyGUI-MaskExpressionThread";
	protected int DEFAULT_SAMPLINGRATE = 1000;
	
	public static final int BASIC=0;
	
	protected int samplingRate;
	
	DatabaseClient myDb;
	AccompanyGUIApp app;
	boolean interrupt;
	//protected MaskAnimationThread mat;
	protected Handler hand;
	
	public String expression;
	public Hashtable<String,Integer> expressionTable;
	
	public MaskExpressionThread(Handler h, DatabaseClient db,AccompanyGUIApp ma)
	{
		this.myDb=db;
		this.hand=h;
		this.app=ma;
		this.interrupt=false;
		this.expression="basic";
		this.expressionTable= new Hashtable<String, Integer>();
		this.expressionTable.put("basic",0); //fear
		this.expressionTable.put("sadness",1);
		this.expressionTable.put("fear",2);
		this.expressionTable.put("disgust", 3);
		this.expressionTable.put("surprise", 4);
		this.expressionTable.put("anger",5);
		this.expressionTable.put("joy",6);
		this.expressionTable.put("low_batteries",7);
		//need to handle the squeeze factor in a special way
		//this.expressionTable.put("squeeze",-1);                  //RIMETTI CON SQUEEZE
		
		this.samplingRate= DEFAULT_SAMPLINGRATE;
	}
	
	public MaskExpressionThread(Handler h, DatabaseClient db,AccompanyGUIApp ma, int sr)
	{
		this.myDb=db;
		this.hand=h;
		this.app=ma;
		this.interrupt=false;
		this.expression="basic";
		this.expressionTable= new Hashtable<String, Integer>();
		this.expressionTable.put("basic",0); //fear
		this.expressionTable.put("sadness",1);
		this.expressionTable.put("fear",2);
		this.expressionTable.put("disgust", 3);
		this.expressionTable.put("surprise", 4);
		this.expressionTable.put("anger",5);
		this.expressionTable.put("joy",6);
		this.expressionTable.put("low_batteries",7);
		//need to handle the squeeze factor in a special way
		//this.expressionTable.put("squeeze",-1);                  //RIMETTI CON SQUEEZE
		
		this.samplingRate= sr;
	}
	
	@Override
	public void start()
	{
		super.start();
		Log.v(TAG,"Starting...");
	}
	
	public void halt()
	{
		this.interrupt=true;
	}
	
	public void run()
	{
		while(!interrupt)
		{
			//Interroga il db 1 volta ogni n millisecondi
			try {
				Log.v(TAG,"Sleeping... "+ samplingRate);
				Thread.sleep(samplingRate);
				myDb.getExpression();
				System.gc();
			} catch (InterruptedException e) {
				Log.e("Accompany Expressions Thread","Error: cannot sleep!");
				interrupt=true;
			}
		}
		Log.i(TAG,"Closing...");
	}
	
	public void handleResponse(String response)
	{
		if (response.equals("error"))
		{
			Log.e("Accompany Expression Thread","Error retriving expression from db...");
			return;
		}
		if (!expression.equals(response))
		{
			/*if (expressionTable.get(expression)==0) //sei in basic
			{
				app.switchMask(expressionTable.get(expression),expressionTable.get(response));
				expression=response;
			}
			else //go in basic
			{
				app.switchMask(expressionTable.get(expression),expressionTable.get("basic"));
				expression="basic";
			}*/
			try{
				Log.v(TAG,"Received new expression: "+response);
				int new_ex = expressionTable.get(response);
				int ans = app.switchMask(expressionTable.get(expression),new_ex);
				if (ans==1) expression=response;
			}
			catch(Exception e)
			{
				Log.e(TAG,"Recived request for unknow expression.");
				Log.e(TAG,"Going to basic...");
				int ans = app.switchMask(expressionTable.get(expression),expressionTable.get("basic"));
				if (ans==1) expression="basic";
			}
		}
		else
		{
			//to handle possible changes in squeeze value
			if (response.equals("squeeze"))
				app.switchMask(this.expressionTable.get(response),this.expressionTable.get(response));
		}
	}
	
	public int getCurrentExpression()
	{
		return this.expressionTable.get(expression);
	}
	
	public void setSamplingRate(int sr)
	{
		this.samplingRate = sr;
	}
}
