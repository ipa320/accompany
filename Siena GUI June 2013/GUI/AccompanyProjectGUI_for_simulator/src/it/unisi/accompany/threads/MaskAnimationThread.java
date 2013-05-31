package it.unisi.accompany.threads;

import it.unisi.accompany.activities.RobotView;
import it.unisi.accompany.activities.RobotWorkingView;
import android.os.Handler;
import android.util.Log;
import android.view.View;

public class MaskAnimationThread extends Thread{

	Handler hh;
	RobotView act;
	boolean sshot=false;
	boolean v;
	boolean interrupt=false;
	boolean pause=false;
	
	public MaskAnimationThread(Handler h,RobotView ma)
	{
		hh=h;
		act=ma;
	}
	
	@Override
	public void run()
	{
		while (!interrupt)
		{
			while (!sshot&&!interrupt)
			{
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					Log.e("sleep","sleep");
				}
			}
			if (!interrupt&&!pause)
			{
				if (v)
				{
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.maskF.setVisibility(View.INVISIBLE);
							act.mask7.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask7.setVisibility(View.INVISIBLE);
							act.mask6.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask6.setVisibility(View.INVISIBLE);
							act.mask5.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask5.setVisibility(View.INVISIBLE);
							act.mask4.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask4.setVisibility(View.INVISIBLE);
							act.mask3.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask3.setVisibility(View.INVISIBLE);
							act.mask2.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask2.setVisibility(View.INVISIBLE);
							act.mask1.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask1.setVisibility(View.INVISIBLE);
							act.mask.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
				}
				else
				{
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask.setVisibility(View.INVISIBLE);
							act.mask1.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask1.setVisibility(View.INVISIBLE);
							act.mask2.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask2.setVisibility(View.INVISIBLE);
							act.mask3.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask3.setVisibility(View.INVISIBLE);
							act.mask4.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask4.setVisibility(View.INVISIBLE);
							act.mask5.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask5.setVisibility(View.INVISIBLE);
							act.mask6.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask6.setVisibility(View.INVISIBLE);
							act.mask7.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask7.setVisibility(View.INVISIBLE);
							act.maskF.setVisibility(View.VISIBLE);
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","sleep");
					}
				}
				sshot=false;
			}
		}
	}
	
	public void terminate()
	{
		interrupt=true;
	}
	
	public void shot(boolean verso)
	{
		sshot=true;
		v=verso;
	}
	
	public void pause()
	{
		pause=true;
	}
	
	public void continueRun()
	{
		pause=false;
	}
	
	public void shot2(boolean verso)
	{
		sshot=true;
		v=verso;
	}
}
