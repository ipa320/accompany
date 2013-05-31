package it.unisi.accompany.rosnodes;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.runnable.CompressedImageAsyncTask;

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

	protected final String TAG = "Accompany GUI - Images subscriber";
	
	private String topicName;
	private String messageType;
	private AccompanyBitmapFromCompressedImage callable;
	private Handler handler;
	protected AccompanyGUIApp myApp;
	
	protected ConnectedNode cn;
	protected Subscriber<sensor_msgs.CompressedImage> subscriber;

	
	//count to avoid the block of the device due to too much bitmaps to be processed
	int num=0;
	
	public ImagesSubscriber(Handler h,AccompanyGUIApp ma)
	{
		super();
		myApp=ma;
		handler=h;		
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("AccompanyGUI","Error: Image subscriber error!");
	}

	@Override
	public void onShutdown(Node arg0) {
		if (subscriber!=null) subscriber.shutdown();
		Log.e("Accompany-Ros","shtdown images subscriber...");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		cn=connectedNode;
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
		try{
			subscriber = cn.newSubscriber(topicName, messageType);
			num=0;
		    subscriber.addMessageListener(new MessageListener<sensor_msgs.CompressedImage>() {
		      @Override
		      public void onNewMessage(final sensor_msgs.CompressedImage message) { 
		    		  CompressedImageAsyncTask iat= new CompressedImageAsyncTask(myApp);
		    		  iat.execute(message);
		      }
		    });
		}catch(Exception e)
		{
			Log.e(TAG,"Cannot create subscriber!");
			subscriber.shutdown();
			cn.shutdown();
			throw new RosRuntimeException(e);
		}
	}
	
	public void stopSubscribing()
	{
		subscriber.shutdown();
	}
	  
	public int getMyPid()
	{
	    return android.os.Process.myPid();
	}
}
