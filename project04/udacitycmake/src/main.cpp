#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


bool run_twiddle = true;

int main(int argc, char** argv)
{

  int twiddle_flag = 0;
  if(argc==2){
	twiddle_flag = atoi(argv[1]);
  }

  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double Kp_initial, Ki_initial, Kd_initial;
  unsigned long timesteps = 0;
  // Use a Gain factor for the steering PID loop for initial tuning
  // To be refined using twiddle
  // Same steering gain is also used to calculate max target speed relation to steering angle
  // Classic Ziegler-Nichols 
  /*
    Set all gains to 0.
    Increase Kd until the system oscillates.
    Reduce Kd by a factor of 2-4.
    Set Kp to about 1% of Kd.
    Increase Kp until oscillations start.
    Decrease Kp by a factor of 2-4.
    Set Ki to about 1% of Kp.
    Increase Ki until oscillations start.
    Decrease Ki by a factor of 2-4.
  */
  double Ku = 3;
  Kd_initial = Ku;//3.0;
  Kp_initial = 0.01*Ku*5;//0.2;
  Ki_initial = 0.01*Kp_initial;//0.004;
  pid.Init(Kp_initial, Ki_initial, Kd_initial);
	        std::ofstream myfile;

  h.onMessage([&pid, &timesteps, &twiddle_flag, &myfile](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
	  if(twiddle_flag==2){
	        myfile.open ("data.txt", std::ofstream::out | std::ofstream::app);
  		myfile << cte << "\t" << speed << "\t" << angle << "\n";
  	 	myfile.close();
 	  	timesteps ++;
		if(timesteps>1000){
			exit(0);
		}
	  }


          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.Controller();

          //std::cout << timesteps << std::endl;
 	  timesteps ++;
          //if (timesteps > pid.twiddle_counter_threshold) {
          if (timesteps > (rand() % pid.twiddle_counter_threshold)) {
	std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
            if (!pid.is_twiddle_done && twiddle_flag==1) {
	      std::cout << "twiddle!" << std::endl;
              pid.Twiddle();
              timesteps = 0;
              //pid.Restart(ws);
              //return;
            }
	std::cout << " CTE: " << cte << " Steering Value: " << steer_value
		  << std::endl;
	std::cout << " Kp:" << pid.Kp
		  << " Ki:" << pid.Ki
		  << " Kd:" << pid.Kd
		  << std::endl;
	std::cout << " p[0]:" << pid.p[0]
		  << " p[1]:" << pid.p[1]
		  << " p[2]:" << pid.p[2]
		  << std::endl;
	std::cout << " dp[0]:" << pid.dp[0]
		  << " dp[1]:" << pid.dp[1]
		  << " dp[2]:" << pid.dp[2]
		  << std::endl;
	std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n\n" << std::endl;
	  }
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
