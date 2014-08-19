#include "accompany_karaoke/karaoke_publisher.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <SFML/Audio.hpp>

/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <cppconn/metadata.h>
#include <cppconn/resultset_metadata.h>
#include <cppconn/exception.h>
#include <cppconn/warning.h>

using namespace std;
using namespace sql;

KaraokePublisher::KaraokePublisher(ros::NodeHandle nh)
{
	node_ = nh;
}


void KaraokePublisher::init()
{
	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The first parameter defines the data type of the messages that become published.
	* Here it is a string. Find all data types and instructions how to create
	* your own messages at ros.org.
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/

	string_publisher_ = node_.advertise<std_msgs::String>("karaoke_publisher", 10);
}


void KaraokePublisher::do_publish()
{
	// publishing rate
	ros::Rate loop_rate(10);

  //	string DBHOST="tcp://127.0.0.1:3306";
  //	string USER="accompanyUser";
  //	string PASSWORD="accompany";
  //	string DATABASE="AccompanyTroyes";


	string DBHOST="tcp://10.0.1.181:3306";
  	string USER="accompanyUser";
  	string PASSWORD="accompany";
  	string DATABASE="AccompanyTroyes";

	
	/**
	* A count of how many messages we have sent. This is used to create
	* a unique string for each message.
	*/

	// send messages with publishing rate
	while (ros::ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		// search my.cnf in the server PC for example in one of this folder
		//    /etc/my.cnf
		//    /etc/mysql/my.cnf
		//    $MYSQL_HOME/my.cnf
		//    [datadir]/my.cnf
		//    ~/.my.cnf
		// and comment skip-networking and bind-address = 127.0.0.1 by using #
		// restart your database as explain below

		//to have mysql working do 
		//grant mysql> CREATE USER 'monty'@'%' IDENTIFIED BY 'some_pass';
        //mysql> GRANT ALL PRIVILEGES ON *.* TO 'monty'@'%'
        //->     WITH GRANT OPTION;
		//restart the mysqlserver : sudo service mysql restart
	
		// Load a music to play
		
		sf::Music music;
		if (!music.OpenFromFile("nice_music.wav"))
		{
			cerr << "unable to read the music !" << endl;
		}


		// Play the music
		music.Play();
		//sleep(5);
		//music.Stop();

		std_msgs::String msg;
		//FILE * fichier;
		//fichier = fopen ("test.txt","rb");

		ifstream fichier("test.txt", ios::in);  // on ouvre le fichier en lecture
	 	
		if(fichier)  // si l'ouverture a réussi
        	{
			//double seconds = 0.;
			//while (fichier.eof() == false)
			//{
			//	sleep(1);
			//	std::string text = "";
			//	fichier >> seconds;
			//	fichier >> text;
			//	std::cout << "[" << seconds << "] " << text << std::endl;

			//	std::stringstream ss;
			//	if (text != "")
			//	{
			//		ss << text;
			//		msg.data = ss.str();
			//		string_publisher_.publish(msg);
					//sleep(seconds);
			//	}
			//}

            		// instructions
			string ligne;
        		while(getline(fichier, ligne))  // tant que l'on peut mettre la ligne dans "contenu"
        		{
				sleep(1);
				std::stringstream ss;
               		 	ss << ligne;  // on l'affiche
				double seconds = 0.;
				ss >> seconds;
				std::size_t found = ss.str().find(" ");
				string ligne2;
				ligne2 = ss.str().substr(found+1);
				cout << "line :" << ligne2 << endl;
				msg.data = ligne2;
				string_publisher_.publish(msg);
				sleep(seconds);
				//if database not do break
				
				int i;
				//for(i=0; i<seconds; i=i+1)
				//{
				Driver *driver;
				Connection *con;
				Statement *stmt;
				ResultSet *result;
				string sql;

				driver = get_driver_instance();
 				con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
  				con->setAutoCommit(0); // turn off the autocommit
  				con->setSchema(DATABASE); // select appropriate database schema
  				stmt = con->createStatement(); // create a statement object


				sql  = "SELECT value FROM Sensors WHERE sensorId = 302";
  				result = stmt->executeQuery(sql);
				 if (result->next())
  				{
    					int value = result->getInt("value");
					cout << "value in database :" << value << endl;
					if (value == 1)
					{
						music.Stop();
					}
  				}
  				else
  				{
    
    				cout<<"SQL_error!!!, no user's locationId found."<<endl;

    				delete result;
    				delete stmt;
    				con->close();
    				delete con;

  				}

   				delete result;
   				delete stmt;
    			con->close();
    			delete con;
				//sleep(5);

        		}
        		//}
                	fichier.close();  // on ferme le fichier
              
        	}
        	else  // sinon
                	cerr << "unable to read the file !" << endl;
 

		//ROS_INFO("%s", msg.data.c_str());

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */


		ros::spinOnce();







	}
}
