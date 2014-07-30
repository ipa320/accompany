/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include<vector>
#include<string>
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

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

//the song array of lines
//char* song_basic[] = {"","[start]","Dashing through the snow","In a one-horse open sleigh","O'er the fields we go","Laughing all the way","Bells on bob tails ring","Making spirits bright","What fun it is to laugh and sing","A sleighing song tonight","[clear]","[clear]","Oh, Jingle bells, jingle bells","Jingle all the way","Oh, what fun","it is to ride","In a one-horse open sleigh.","Jingle bells, jingle bells","Jingle all the way","Oh, what fun","it is to ride","In a one-horse open sleigh.","[clear]","[stop]"};
//std::vector<std::string> song(song_basic,end(song_basic));
int playing=0;
ros::Publisher chatter_pub;

void callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (strcmp(msg->data.c_str(),"[START]")==0 && playing==0){
 	playing=1;
  	ROS_INFO("Starting karaoke music!");
        //karaoke part
	int count = 0;
	ros::Rate loop_rate(0.5);

  	string DBHOST="tcp://127.0.0.1:3306";
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
				chatter_pub.publish(msg);
				sleep(seconds);
				//if database not do break

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


				//sql  = "SELECT value FROM Sensors WHERE sensorId = 302";
				sql = "SELECT * FROM Sensors WHERE sensorId = 302 AND lastStatus = \"On\" and lastUpdate+INTERVAL 60 SECOND >= NOW()";
  				result = stmt->executeQuery(sql);
				cout << result  << endl;
				if (result->next())
  				{
				
					music.Stop();
					msg.data = "[stop]";
					chatter_pub.publish(msg);
					
  				}
  				else
  				{
    
    				cout<<"Ringbell still off"<<endl;

    				//delete result;
    				//delete stmt;
    				//con->close();
    				//delete con;

  				}

   				delete result;
   				delete stmt;
    				con->close();
    				delete con;
				

        		}
                	fichier.close();  // on ferme le fichier
			break;
        	}
        	else  // sinon
                	cerr << "unable to read the file !" << endl;


		ros::spinOnce();


	}

        playing=0;
  }
}


/**
 * Main: subscriber that when started triggers the publisher
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "karaoke_talker");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("accompany_karaokeControl", 1000, callback);
  chatter_pub = n.advertise<std_msgs::String>("accompany_karaoke", 1000);
  
  ros::spin();


  return 0;
}

