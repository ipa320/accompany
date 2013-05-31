package it.unisi.accompany.runnable;

import it.unisi.accompany.AccompanyGUIApp;

import org.jboss.netty.buffer.ChannelBuffer;

import sensor_msgs.CompressedImage;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;

public class CompressedImageAsyncTask extends AsyncTask<sensor_msgs.CompressedImage,Void,Bitmap>{

	protected AccompanyGUIApp myApp;
	
	protected BitmapFactory.Options options_bounds;
	
	public CompressedImageAsyncTask(AccompanyGUIApp my)
	{
		super();
		myApp = my;
	}
	
	@Override
	protected Bitmap doInBackground(CompressedImage... params) {
		for (CompressedImage par : params)
		{
			   options_bounds= new BitmapFactory.Options();
			   options_bounds.inPurgeable=true;

			   ChannelBuffer buffer = par.getData();
			   byte[] data = buffer.array();
			   options_bounds.inJustDecodeBounds=false;

			   Bitmap b = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(), buffer.readableBytes(),options_bounds);
			   
			   System.gc();
			   return b;
		}
		return null;
	}

	@Override
	protected void onPostExecute(Bitmap b)
	{
		myApp.setBitmap(b);//,m);
	}
	
}

