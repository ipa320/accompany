package it.unisi.accompany.threads;

import org.jboss.netty.buffer.ChannelBuffer;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.util.Log;
import android.view.SurfaceHolder;

public class DrawingThread extends Thread{
	protected static final String TAG="AccompanyGUI-DrawingThread"; 
	
	protected boolean stop_draw,image_changed,paused;
	protected SurfaceHolder sh;
	
	protected sensor_msgs.CompressedImage last_image;
	
	protected BitmapFactory.Options options_bounds;
	
	public DrawingThread(SurfaceHolder hh)
	{
		super();
		this.stop_draw=false;
		this.sh=hh;
		this.image_changed=false;
		this.paused=false;
	}
	
	// constructor with last image
	public DrawingThread(SurfaceHolder hh, sensor_msgs.CompressedImage msg)
	{
		super();
		this.stop_draw=false;
		this.sh=hh;
		this.image_changed=false;
		this.last_image=msg;
		this.paused=false;
	}
	
	@Override 
	public void run()
	{
		/*if (last_image!=null)
		{
			options_bounds= new BitmapFactory.Options();
			options_bounds.inPurgeable=true;

			ChannelBuffer buffer = last_image.getData();
			byte[] data = buffer.array();
			options_bounds.inJustDecodeBounds=false;

			Bitmap b = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(), buffer.readableBytes(),options_bounds);
			   
		    System.gc();
		    
		    Canvas c = sh.lockCanvas();
		    
		    c.drawBitmap(b,0,0,null);
		    
		    sh.unlockCanvasAndPost(c);
		    image_changed=false;
		}*/
		while (!this.stop_draw)
		{
			while (this.paused)
			{
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					Log.e(TAG,"cannot sleep!");
				}
			}
			while(!this.stop_draw && !this.image_changed)
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				Log.e(TAG,"cannot sleep!");
			}
			if (!this.stop_draw && !this.paused)
			{ // in that case a new image is arrived
				options_bounds= new BitmapFactory.Options();
				options_bounds.inPurgeable=true;

				ChannelBuffer buffer = last_image.getData();
				byte[] data = buffer.array();
				options_bounds.inJustDecodeBounds=false;

				Bitmap b = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(), buffer.readableBytes(),options_bounds);
				   
			    //System.gc();
			    
			    Canvas c = sh.lockCanvas();
			    
			    c.drawBitmap(b,0,0,null);
			    
			    sh.unlockCanvasAndPost(c);
			    image_changed=false;
			}
		}
		Log.i(TAG,"Closing...");
	}
	
	public void stopDraw()
	{
		this.stop_draw=true;
	}
	
	public void setImage(sensor_msgs.CompressedImage img)
	{
		if (!image_changed) 
		{
			last_image=img;
			image_changed=true;
		}
	}
	
	public void pause()
	{
		paused=true;
	}
	
	public void restart()
	{
		paused=false;
	}
}
