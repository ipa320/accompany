package it.unisi.accompany.threads;

import java.util.ArrayList;

import android.os.Handler;
import android.util.Log;
import it.unisi.accompany.AccompanyGUIApp;

public class ActionPossibilitiesUpdateThread extends Thread{
	
	protected final String TAG="AccompanyGUI-ApUpdateThread";
	
	protected AccompanyGUIApp myApp;
	protected boolean stop;
	
	protected Handler myHandler;
	
	protected final int sleeptime=5000;
	
	protected ArrayList<Integer> rob_id,usr_id;
	
	public ActionPossibilitiesUpdateThread(AccompanyGUIApp ma,Handler h)
	{
		super();
		this.myApp=ma;
		this.myHandler=h;
	}
	
	@Override
	public void run()
	{
		//fai prima acquisizione senza confronto
		
		while(!stop)
		{
			try {
				Thread.sleep(sleeptime);
			} catch (InterruptedException e) {
				Log.e(TAG,"Sleeping problem!");
			}
			// fai direttamente qua hhtpp client sincorno! e confronta con valori vecchi.
			
			//se c'è novità allora manda richiesta di refresh alla GUI - > che aggiunge le nuove in coda, mette la flag di rimozione alle altre 
			//(i.e. toglie se sono in normal, le toglie al prossimo reset se sono selezionate)
		}
	}
	
	public void halt()
	{
		Log.v(TAG,"Closing...");
		stop=true;
	}

}
