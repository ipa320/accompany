package it.unisi.accompany.msgs_and_data;

public abstract interface AccompanyActionResponse extends org.ros.internal.message.Message {
	
	 // Field descriptor #5 Ljava/lang/String;
	  public static final java.lang.String _TYPE = "it/unisi/accompany/msgs_and_data/AccompanyActionResponse";
	  
	  // Field descriptor #5 Ljava/lang/String;
	  public static final java.lang.String _DEFINITION = "int64 result\n";
	  
	  // Method descriptor #11 ()J
	  public abstract long getResult();
	  
	  // Method descriptor #13 (J)V
	  public abstract void setAction(long result);
}
