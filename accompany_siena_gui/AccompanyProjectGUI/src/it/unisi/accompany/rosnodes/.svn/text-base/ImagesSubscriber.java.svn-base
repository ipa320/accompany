package it.unisi.accompany.rosnodes;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.R;
import it.unisi.accompany.runnable.CompressedImageAsyncTask;
import it.unisi.accompany.threads.DrawingThread;

import org.ros.android.AccompanyBitmapFromCompressedImage;
import org.ros.android.AccompanyBitmapFromImage;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.exception.RosRuntimeException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;


import android.graphics.Bitmap;
import android.os.Handler;
import android.util.Log;

public class ImagesSubscriber<T> implements NodeMain{

	protected final String TAG = "AccompanyGUI-ImagesSubscriber";
	
	private String topicName;
	private String messageType;
	private AccompanyBitmapFromCompressedImage callable;
	private Handler handler;
	protected AccompanyGUIApp myApp;
	
	protected ConnectedNode cn;
	protected Subscriber<sensor_msgs.CompressedImage> subscriber;

	protected boolean shouldStart=false;
	
	protected DrawingThread dt; 
	protected sensor_msgs.CompressedImage last_message;
	
	//count to avoid the block of the device due to too much bitmaps to be processed
	int num=0;
	
	public ImagesSubscriber(Handler h,AccompanyGUIApp ma)
	{
		super();
		myApp=ma;
		handler=h;	
		
		shouldStart=false;
		last_message=null;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e(TAG,"Error: Image subscriber error!");
		myApp.closeAppOnError(myApp.getResources().getString(R.string.comunication_error));
	}

	@Override
	public void onShutdown(Node arg0) {
		if (subscriber!=null) subscriber.shutdown();
		Log.e(TAG,"Shutdown...");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		cn=connectedNode;
		if (shouldStart) 
		{
			this.startSubscribing();
			shouldStart=false;
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("GuiImageSubscriber");
	}

	public void setTopicName(String topicName) {
	    this.topicName = topicName;
	}

	public void setMessageType(String messageType) {
	    this.messageType = messageType;
	}

	public void setMessageToBitmapCallable(AccompanyBitmapFromCompressedImage callable) {
	  this.callable = callable;
	}
	  
	public void startSubscribing()
	{
		if (cn!=null)
		{
			try{
				subscriber = cn.newSubscriber(topicName, messageType);
				num=0;
			    subscriber.addMessageListener(new MessageListener<sensor_msgs.CompressedImage>() {
			      @Override
			      public void onNewMessage(final sensor_msgs.CompressedImage message) { 
			    	  if (dt==null)
			    	  {
			    		  CompressedImageAsyncTask iat= new CompressedImageAsyncTask(myApp);
			    		  iat.execute(message);
			    		  //last_message=message;
			    	  }
			    	  else
			    	  {
			    		  dt.setImage(message);
			    		  //last_message= message;
			    	  }
			      }
			    });
			}catch(Exception e)
			{
				Log.e(TAG,"Cannot create subscriber!");
				if (subscriber!=null) subscriber.shutdown();
				if (cn!=null) cn.shutdown();
				myApp.toastMessage("Missing Images Publisher!");
				throw new RosRuntimeException(e);
			}
		}
		else
			shouldStart=true;
	}
	
	public void stopSubscribing()
	{
		if (subscriber!=null)
			subscriber.shutdown();
		else
			Log.e(TAG,"reciving stop command before start!");
	}
	  
	public int getMyPid()
	{
	    return android.os.Process.myPid();
	}
	
	public void shouldStart(boolean b)
	{
		shouldStart=b;
	}
	
	public void setDrawingThread(DrawingThread d)
	{
		dt=d;
	}
	
	public sensor_msgs.CompressedImage getLastMsg()
	{
		return last_message;
	}
}
