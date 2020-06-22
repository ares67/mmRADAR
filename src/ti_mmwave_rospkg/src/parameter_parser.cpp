#include "parameter_parser.h"

namespace ti_mmwave_rospkg {

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::parameter_parser, nodelet::Nodelet);

parameter_parser::parameter_parser() {}

void parameter_parser::onInit() {}

void parameter_parser::params_parser(ti_mmwave_rospkg::mmWaveCLI &srv, ros::NodeHandle &nh) 
{
  //   ROS_ERROR("%s",srv.request.comm.c_str());
  //   ROS_ERROR("%s",srv.response.resp.c_str());
  std::vector <std::string> v;
  std::string s = srv.request.comm.c_str(); 
  std::istringstream ss(s);
  std::string token;
  std::string req;
  int i = 0;
  while (std::getline(ss, token, ' ')) {
    v.push_back(token);
    if (i > 0) {
      if (!req.compare("profileCfg")) {
        switch (i) {
          case 2:
            nh.setParam("/mmWave_Manager/startFreq", std::stof(token)); break;
          case 3:
            nh.setParam("/mmWave_Manager/idleTime", std::stof(token)); break;
          case 4:
            nh.setParam("/mmWave_Manager/adcStartTime", std::stof(token)); break;
          case 5:
            nh.setParam("/mmWave_Manager/rampEndTime", std::stof(token)); break;
          case 8:
            nh.setParam("/mmWave_Manager/freqSlopeConst", std::stof(token)); break;
          case 10:
            nh.setParam("/mmWave_Manager/numAdcSamples", std::stoi(token)); break;
          case 11:
            nh.setParam("/mmWave_Manager/digOutSampleRate", std::stof(token)); break;
          case 14:
            nh.setParam("/mmWave_Manager/rxGain", std::stof(token)); break;
        }
      } else if (!req.compare("frameCfg")) {
        switch (i) {
          case 1:
            nh.setParam("/mmWave_Manager/chirpStartIdx", std::stoi(token)); break;
          case 2:
            nh.setParam("/mmWave_Manager/chirpEndIdx", std::stoi(token)); break;
          case 3:
            nh.setParam("/mmWave_Manager/numLoops", std::stoi(token)); break;
          case 4:
            nh.setParam("/mmWave_Manager/numFrames", std::stoi(token)); break;
          case 5:
            nh.setParam("/mmWave_Manager/framePeriodicity", std::stof(token)); break;
        }
      }
    } else req = token;
    i++;
  }
}

void parameter_parser::cal_params(ros::NodeHandle &nh) {
  float c0 = 299792458;
  int chirpStartIdx;
  int chirpEndIdx;
  int numLoops;
  float framePeriodicity;
  float startFreq;
  float idleTime;
  float adcStartTime;
  float rampEndTime;
  float digOutSampleRate;
  float freqSlopeConst;
  float numAdcSamples;

  // Edited for IWR6843
  float const1 =1.25;

  nh.getParam("/mmWave_Manager/startFreq", startFreq);
  nh.getParam("/mmWave_Manager/idleTime", idleTime);
  nh.getParam("/mmWave_Manager/adcStartTime", adcStartTime);
  nh.getParam("/mmWave_Manager/rampEndTime", rampEndTime);
  nh.getParam("/mmWave_Manager/digOutSampleRate", digOutSampleRate);
  nh.getParam("/mmWave_Manager/freqSlopeConst", freqSlopeConst);
  nh.getParam("/mmWave_Manager/numAdcSamples", numAdcSamples);

  nh.getParam("/mmWave_Manager/chirpStartIdx", chirpStartIdx);
  nh.getParam("/mmWave_Manager/chirpEndIdx", chirpEndIdx);
  nh.getParam("/mmWave_Manager/numLoops", numLoops);
  nh.getParam("/mmWave_Manager/framePeriodicity", framePeriodicity);

  int ntx = chirpEndIdx - chirpStartIdx + 1;
  int nd = numLoops;
  int nr = numAdcSamples;
  float tfr = framePeriodicity * 1e-3;
  float fs = digOutSampleRate * 1e3;
  float kf = freqSlopeConst * 1e12;
  float adc_duration = nr / fs;
  float BW = adc_duration * kf;
  float PRI = (idleTime + rampEndTime) * 1e-6;
  float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2); 
  float fc_chirp = startFreq * 1e9 + BW / 2; 

  float vrange = c0 / (2 * BW);
  float max_range = nr * vrange /const1;
  float max_vel = c0 / (2 * fc * PRI) / ntx;      // Here is the absolute velocity discrepency between two direction. For one direction, divide it by 2 
  float vvel = max_vel / nd;

  nh.setParam("/mmWave_Manager/num_TX", ntx);
  nh.setParam("/mmWave_Manager/f_s", fs);
  nh.setParam("/mmWave_Manager/f_c", fc);
  nh.setParam("/mmWave_Manager/fc_chirp", fc_chirp);
  nh.setParam("/mmWave_Manager/BW", BW);
  nh.setParam("/mmWave_Manager/PRI", PRI);
  nh.setParam("/mmWave_Manager/t_fr", tfr);
  nh.setParam("/mmWave_Manager/max_range", max_range);
  nh.setParam("/mmWave_Manager/range_resolution", vrange);
  nh.setParam("/mmWave_Manager/max_doppler_vel", max_vel);
  nh.setParam("/mmWave_Manager/doppler_vel_resolution", vvel);


  ROS_INFO("parameter_parser: range_max = %f", max_range);
  ROS_INFO("parameter_parser: range_res = %f", vrange);
  ROS_INFO("parameter_parser: vel_max = %f", max_vel);
  ROS_INFO("parameter_parser: vel_res = %f", vvel);

}

}