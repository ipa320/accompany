package it.unisi.accompany.threads;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.clients.DatabaseClient;

import java.util.Hashtable;

import android.graphics.drawable.Drawable;
import android.os.Handler;
import android.util.Log;

public class MaskExpressionThread extends Thread{
	protected final int samplingRate=2500;
	
	DatabaseClient myDb;
	AccompanyGUIApp app;
	boolean interrupt;
	//protected MaskAnimationThread mat;
	protected Handler hand;
	
	public String expression;
	public Hashtable<String,Integer> expressionTable;
	
	public MaskExpressionThread(Handler h, DatabaseClient db,AccompanyGUIApp ma)
	{
		myDb=db;
		hand=h;
		app=ma;
		interrupt=false;
		expression="basic";
		expressionTable= new Hashtable<String, Integer>();
		expressionTable.put("basic",0); //fear
		expressionTable.put("sadness",1);
		expressionTable.put("fear",2);
		expressionTable.put("disgust", 3);
	}
	
	@Override
	public void start()
	{
		super.start();
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
				Thread.sleep(samplingRate);
				myDb.getExpression();
				System.gc();
			} catch (InterruptedException e) {
				Log.e("Accompany Expressions Thread","Error: cannot sleep!");
			}
		}
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
			if (expressionTable.get(expression)==0) //sei in basic
			{
				app.switchMask(expressionTable.get(expression),expressionTable.get(response));
				expression=response;
			}
			else //go in basic
			{
				app.switchMask(expressionTable.get(expression),expressionTable.get("basic"));
				expression="basic";
			}
			/*mat = new MaskAnimationThread(hand,app,expression,response);
			mat.start();
			expression=response;*/
		}
	}
	
	public int getCurrentExpression()
	{
		return expressionTable.get(expression);
	}
	

}
