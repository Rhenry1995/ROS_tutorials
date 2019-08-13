// ROS Includes
#include <ros/ros.h>

// ROS Message includes. For any messages that you are sending via ROS you need
// to include the message header file which stores the data structure.
#include <std_msgs/String.h>

// Also remember to include all other files as you normally would (eg math.h)
#include <math.h>

/*
==========================================================================
If there are any stuctures that will be used make sure to create them here
before you construct the class
==========================================================================
*/


/*
==========================================================================
Create the ROS class. This can be done by creating a seperate .h and .cpp file
like you would with a normal file. However this is not nessecary becasue of the
short structure of the nodes even if it opposes traditional programming
ethiquette.

To demonstrate the process of ROS's communication standing this node will
simply publish a string and then subscribe to it and print it in a loop.
==========================================================================
*/

class RosTemplate{
  // There are 3 different ROS varibles that need to be initiliated. These
  // should be private
  // 1) Node handlers: There are 2 seperate handlers used here.
  ros::NodeHandle nh_; // This is used for handling the subscribing and publishing of topics
  ros::NodeHandle pnh_; // This is for the handling of the paramters that are set when running the node

  // 2) ROS Subscribers: These will subscribe to the topics that are avalible
  ros::Subscriber sub_;

  // 3) Publishers: These publish the messages to the set topics
  ros::Publisher pub_;

  public:
    // Place any member varibles here as you normally would.
    std::string message = "hello world";
    std_msgs::String ros_message;


    // Class consturctor
    RosTemplate(){

      // Load any paramters. For this example the only parameter will be a string
      // Set the node handler to "~" so it looks for paramters.
      pnh_ = ros::NodeHandle("~");
      // Structure of .getParam() is ("PARAMETER_NAME", varible_name). If there
      // is no parameter set the varible will not change.
      pnh_.getParam("message", message);

      // Set Publishers: The set up is as follows
      // pub_var_ = nh_.advertise<message_class>(topic, buffer);
      // NOTE: I normally just have a buffer as 1
      pub_ = nh_.advertise<std_msgs::String>("hello_world_topic", 1);


      // Set Subscribers: These will set a listener for a topic then run a
      // callback function when it is activated. The structure for the set up
      // as follows
      // sub_var_ = nh_.subscribe("topic", buffer, &class::callback_function, this)
      sub_ = nh_.subscribe("hello_world_topic", 1, &RosTemplate::string_Callback, this);

      // IMPORTANT: Set the publisher to run before any subscribers as this
      // has the possibility of causing a runtime error if not.


      // This section is just to get the publish subscribe loop running
      ros_message.data = message; // Sets message data to be the message
      pub_.publish(ros_message);
    }

    // Function which is called when a topic is discribed to. The arg class for
    // the function is a constant of the message class
    void string_Callback(const std_msgs::String msg){
      std::string msg_std = msg.data;

      // ROS INFO is a tool used to display something on the terminal window
      // like std::cout. Structure ROS_INFO("string  %i %f", var1, var2);
      ROS_INFO("%s", msg_std.c_str());

      // Publish the message again for the function run in a loop.
      pub_.publish(ros_message);
    }

};


int main(int argc, char *argv[]) {
  // Initilise ROS
  ros::init(argc, argv, "base_file_structure");

  // Initilise the ROS class
  RosTemplate rt;

  // Ros spin. This is only required if you are subscribing to a topic in your
  // node. Otherwise it isnt nessecary. But what it does is prevents the node
  // from stopping so you are able to continue subscribing to topics. The
  // detailed description is here. http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
  ros::spin();
  return 0;
}
