package it.unisi.accompany.msgs_and_data;

public abstract interface AccompanyActionRequest extends org.ros.internal.message.Message {
		  
		  // Field descriptor #5 Ljava/lang/String;
		  public static final java.lang.String _TYPE = "it/unisi/accompany/msgs_and_data/AccompanyActionRequest";
		  
		  // Field descriptor #5 Ljava/lang/String;
		  public static final java.lang.String _DEFINITION = "string action\nint64 uid\n";
		  
		  // Method descriptor #11 ()J
		  public abstract String getAction();
		  
		  // Method descriptor #13 (J)V
		  public abstract void setAction(String arg0);
		  
		  // Method descriptor #11 ()J
		  public abstract long getUid();
		  
		  // Method descriptor #13 (J)V
		  public abstract void setUid(long arg0);
		  
}
