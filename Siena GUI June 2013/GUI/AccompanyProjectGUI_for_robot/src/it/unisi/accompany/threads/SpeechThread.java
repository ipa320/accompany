package it.unisi.accompany.threads;


import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.speech.MicrophoneInput;
import it.unisi.accompany.speech.MicrophoneInputListener;
import android.content.Context;
import android.media.MediaRecorder;
import android.os.Handler;
import android.util.Log;

public class SpeechThread extends Thread implements MicrophoneInputListener{
	 MicrophoneInput micInput;  // The micInput object provides real time audio.
	 double width=0;
	  double mean=0;
	  double speed=0;
	  double mOffsetdB = 10;
	  double mGain = 2500.0 / Math.pow(10.0, 90.0 / 20.0);
	  // For displaying error in calibration.
	  double mDifferenceFromNominal = 0.0;
	  double mRmsSmoothed;  // Temporally filtered version of RMS.
	  double mAlpha = 0.9;  // Coefficient of IIR smoothing filter for RMS.
	  private final int mSampleRate=8000;  // The audio sampling rate to use.
	  private final int mAudioSource=MediaRecorder.AudioSource.VOICE_RECOGNITION;  // The audio source to use.
	  
	  // Variables to monitor UI update and check for slow updates.
	  private volatile boolean mDrawing;
	  private volatile int mDrawingCollided;
	  double max=0;
	  int waiting=0;
	  
	  protected int DB_TRESHOLD=57; //originale 42, minimi risultati 52(con rumore tagliato)
	  protected boolean state;
	  
	  protected AccompanyGUIApp myApp;
	  private static final String TAG = "AccompanyGUI-Speech";

	  protected Handler h;
	  
	  double normalizedValue;
	  protected int NORMALIZED_TRS=300;
	  
	  public SpeechThread(AccompanyGUIApp a,boolean b,Handler hh)
	  {
		  super();
		  myApp=a;
		  h=hh;
		  state=b;
	  }
	  
	  public void setMode(boolean b)
	  {
		  state=b;
	  }
	  
	  @Override
	  public void start()
	  {
		  micInput = new MicrophoneInput(this);
		  micInput.setSampleRate(mSampleRate);
          micInput.setAudioSource(mAudioSource);
          micInput.start();
		  super.start();
	  }
	  
	  public void halt()
	  {
		  micInput.stop();
		  interrupt();
	  }	  

	@Override
	public void processAudioFrame(short[] audioFrame) {
		// Compute the RMS value. (Note that this does not remove DC).
        double rms = 0;
        for (int i = 0; i < audioFrame.length; i++) {
          rms += audioFrame[i]*audioFrame[i];
        }
        rms = Math.sqrt(rms/audioFrame.length);
     // Compute a smoothed version for less flickering of the display.
        mRmsSmoothed = mRmsSmoothed * mAlpha + (1 - mAlpha) * rms;
        final double rmsdB = 20.0 * Math.log10(mGain * mRmsSmoothed);
        
        double d=20 + rmsdB;
        if(d>DB_TRESHOLD){
      	 mean+=d;
      	 width++;
      	 if(d>max){
      		 max=d;
      		 speed=width;
      	 }
      		 
        }
        else
        {	  
      	  if(mean!=0){
      		  if(waiting>10){
      			  waiting=0;
      		  //try {
      			  normalizedValue=(mean/width);
      			  normalizedValue=(normalizedValue-DB_TRESHOLD)*DB_TRESHOLD;//*35;
            		String msg=width+" "+normalizedValue+" "+speed;
            		
					Log.i(TAG,"Hearing: "+msg);
					//Log.i(TAG,"Speed: "+speed);
					Log.i(TAG,"Emphasis: "+(normalizedValue-NORMALIZED_TRS)/1000);
					if (state)
					{
						h.post(new Runnable(){
					

							@Override
							public void run() {
								//if (speed>25&&state)
								//	myApp.toastMessage("I'm coming");
								//else
									if (normalizedValue>NORMALIZED_TRS&&state) 
										myApp.toastMessage("I'm coming");
							}
							
						});
						//if (speed>25&&state)
						//{ 
						//	myApp.emp_client.setEmphasis((speed-10)/100);
						//	myApp.sendActionRequest("come_here","I'm coming");
						//	Log.e("AccompanyGUI-speech","state "+state);
						//}
						//else
							if (normalizedValue>NORMALIZED_TRS&&state) 
							{
								myApp.emp_client.setEmphasis((normalizedValue-NORMALIZED_TRS)/1000);
								//myApp.sendActionRequest("come_here","I'm coming");
								myApp.sendActionRequest(myApp.COMEHEREID);
								Log.e("AccompanyGUI-speech","state "+state);
							}
            		
					}
            		//onEndEmphasis();
            		try {
						Thread.sleep(5000);
					} catch (InterruptedException e) {
						Log.e(TAG,"Error sleeping!");
					}

      		  max=0;
      		  mean=0;
      		  width=0;
      		  speed=0;
      	  }
      	  }
      	  waiting++;
      		  
        }
	}
}
