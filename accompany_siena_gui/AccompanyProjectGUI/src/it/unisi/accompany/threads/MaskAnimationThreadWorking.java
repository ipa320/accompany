package it.unisi.accompany.threads;

import it.unisi.accompany.activities.RobotWorkingView;
import android.os.Handler;
import android.util.Log;
import android.view.View;

public class MaskAnimationThreadWorking extends Thread{

	protected final String TAG = "AccompanyGUI-MaskAnimationThread(W)";
	
	Handler hh;
	RobotWorkingView act;
	boolean sshot=false;
	boolean v;
	boolean interrupt=false;
	boolean pause=false;
	int after;
	
	boolean im1,im2,im3,im4,im5,im6,im7,imF;
	
	public MaskAnimationThreadWorking(Handler h,RobotWorkingView ma)
	{
		hh=h;
		act=ma;
	}
	
	@Override
	public void run()
	{
		im1=true;
		im2=true;
		im3=true;
		im4=true;
		im5=true;
		im6=true;
		im7=true;
		imF=true;
		while (!interrupt)
		{
			while (!sshot&&!interrupt)
			{
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					Log.e("sleep","interrupted!");
					interrupt=true;
				}
			}
			//wait to have all the images loaded
			while (!im1||!im2||!im3||!im4||!im5||!im6||!im7||!imF)
			{
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					Log.e("sleep","interrupted!");
					interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
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
						Log.e("sleep","interrupted!");
						interrupt=true;
					}
					
					resetLoad();
					
					sshot=false;
					hh.post(new Runnable() {
						
						@Override
						public void run() {
							act.mask1.setVisibility(View.INVISIBLE);
							act.mask.setVisibility(View.VISIBLE);
							if (after!=0)
							{
								act.switchMask(0, after);
							}
						}
					});
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						Log.e("sleep","interrupted!");
						interrupt=true;
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
					sshot=false;
				}
			}
		}
	}
	
	public void resetLoad()
	{
		im1=false;
		im2=false;
		im3=false;
		im4=false;
		im5=false;
		im6=false;
		im7=false;
		imF=false;
	}
	
	public void imageLoaded(int idx)
	{
		switch (idx)
		{
			case 1:
				im1=true;
				break;
			case 2:
				im2=true;
				break;
			case 3:
				im3= true;
				break;
			case 4:
				im4= true;
				break;
			case 5:
				im5= true;
				break;
			case 6:
				im6= true;
				break;
			case 7:
				im7= true;
				break;
			case 8:
				imF= true;
				break;
			default:
				break;
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
	
	public void shot2(boolean verso,int a)
	{
		after=a;
		sshot=true;
		v=verso;
	}
}
