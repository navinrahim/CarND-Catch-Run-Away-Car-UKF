#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
long long time_pred = 0.0;
double heading_to_target;
double heading_difference;
double distance_difference;
double target_x = 0.0;
double target_y = 0.0;
int step=0;


int main()
{
  uWS::Hub h;

  // Create a UKF instance
  UKF ukf;

  h.onMessage([&ukf](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
      	
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());
          
          string lidar_measurment = j[1]["lidar_measurement"];
          
          MeasurementPackage meas_package_L;
          istringstream iss_L(lidar_measurment);
    	  long long timestamp_L;

    	  // reads first element from the current line
    	  string sensor_type_L;
    	  iss_L >> sensor_type_L;

      	  // read measurements at this timestamp
      	  meas_package_L.sensor_type_ = MeasurementPackage::LASER;
          meas_package_L.raw_measurements_ = VectorXd(2);
          float px;
      	  float py;
          iss_L >> px;
          iss_L >> py;
          meas_package_L.raw_measurements_ << px, py;
          iss_L >> timestamp_L;
          meas_package_L.timestamp_ = timestamp_L;
          
    	  ukf.ProcessMeasurement(meas_package_L);
		 
    	  string radar_measurment = j[1]["radar_measurement"];
          
          MeasurementPackage meas_package_R;
          istringstream iss_R(radar_measurment);
    	  long long timestamp_R;

    	  // reads first element from the current line
    	  string sensor_type_R;
    	  iss_R >> sensor_type_R;

      	  // read measurements at this timestamp
      	  meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
          meas_package_R.raw_measurements_ = VectorXd(3);
          float ro;
      	  float theta;
      	  float ro_dot;
          iss_R >> ro;
          iss_R >> theta;
          iss_R >> ro_dot;
          meas_package_R.raw_measurements_ << ro,theta, ro_dot;
          iss_R >> timestamp_R;
          meas_package_R.timestamp_ = timestamp_R;
          
    	  ukf.ProcessMeasurement(meas_package_R);
		step+=1;
		int flag=0;
		if((time_pred<=meas_package_R.timestamp_) || (time_pred==0.0)) {
		  
		  time_pred = meas_package_R.timestamp_+2000000.0;	
		  UKF ukf_predict = ukf;
		  ukf_predict.Prediction(2);
		  target_x = ukf_predict.x_[0];
		  target_y = ukf_predict.x_[1];	
		  heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
		  heading_to_target = atan2(sin(heading_to_target) , cos(heading_to_target));
    	  //turn towards the target
    	  heading_difference = heading_to_target - hunter_heading;
    	  heading_difference = atan2(sin(heading_difference) , cos(heading_difference));
		
    	  distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));
		   
		  flag=1;
		  step = 0;
		  cout<<"first if \n ------------------------------------"<<endl;
		  cout<<"timestamp_:"<<meas_package_R.timestamp_<<endl;
		  cout<<"Time_Pred:"<<time_pred<<endl;
		  cout<<"ukf_predict:"<<ukf_predict.x_<<endl;
		}
		else if(step == 80) {
			double dt = (time_pred - meas_package_R.timestamp_) / 1000000.0 ;
			UKF ukf_predict = ukf;
			ukf_predict.Prediction(dt);
			target_x = ukf_predict.x_[0];
			target_y = ukf_predict.x_[1];	
			heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
			heading_to_target = atan2(sin(heading_to_target) , cos(heading_to_target));
			//turn towards the target
			heading_difference = heading_to_target - hunter_heading;
			heading_difference = atan2(sin(heading_difference) , cos(heading_difference));
		
			distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));
			step= 0;
			cout<<"else if \n ------------------------------------"<<endl;
		    cout<<"timestamp_:"<<meas_package_R.timestamp_<<endl;
		    cout<<"Time_Pred:"<<time_pred<<endl;
			cout<<"step:"<<step<<endl;
			cout<<"ukf_predict:"<<ukf_predict.x_<<endl;
		}
		
		if(distance_difference >10 ) {
			time_pred -= 1000000.0;
			cout<<"distance_difference"<<distance_difference<<endl;;
		 }
		
		cout<<"timestamp_:"<<meas_package_R.timestamp_<<endl;
		cout<<"Time_Pred:"<<time_pred<<endl;
		cout<<"step:"<<step<<endl;
		// cout<<"heading_to_target:"<<heading_to_target<<endl;
		// cout<<"heading_difference:"<<heading_difference<<endl;
		// cout<<"distance_difference:"<<distance_difference<<endl;
		// cout<<"target_x"<<target_x<<endl;
		// cout<<"target_y"<<target_y<<endl;
		
		
		json msgJson;
		
		if(flag==1) {
		  msgJson["turn"] = heading_difference;  
		  msgJson["dist"] = distance_difference;		  
		}
		else {
		  msgJson["turn"] = 0;
		  msgJson["dist"] = distance_difference;
		}
        
		
		
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	    
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































