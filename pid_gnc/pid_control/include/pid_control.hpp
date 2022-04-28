#pragma once

#include "rocket_types/rocket_types.h"
#include "rocket_model.hpp"
#include "rocket_types/eigen_conversions.h"
#include "pid_template.hpp"
#include "yaml-cpp/yaml.h"
#include "Eigen/Geometry"

#include <memory>
#include <vector>
#include <iostream>

using namespace rocket;

#define INNER_ITER 2
#define INNER_INNER_ITER 4
#define INNER_INNER_INNER_ITER 8
#define G 9.8067


class PidController {

public:
    // Constructor using default config
    //PidController(RocketProps rocket_props) : PidController(rocket_props) {}
    struct ControlCascades{
        std::vector<std::vector<PID>> controllers;
        std::vector<std::vector<double>> errors, error_rates, outputs;
        double alpha_limit, beta_limit;
    };


    PidController(RocketProps &rocket_props, std::shared_ptr<PidController::ControlCascades>& cc, std::shared_ptr<std::pair<rocket::RocketGimbalControl, rocket::RocketControlMomentGyro>>& control_output) :
            
            rocket_model(rocket_props) {
                this->cc = cc;
                this->control_output.reset(new std::pair<rocket::RocketGimbalControl, rocket::RocketControlMomentGyro>());
                this->control_output->first.inner_angle = 0.0;
                this->control_output->first.outer_angle = 0.0;
                this->control_output->first.thrust = 0.0;
                this->control_output->second.inner_angle = 0.0;
                this->control_output->second.outer_angle = 0.0;
                this->control_output->second.torque = 0.0;
                control_output = this->control_output;
                prev_time = 0.0;
                Fz = rocket_model.props.dry_mass * 9.81;
                alpha = 0.0;
                beta = 0.0;
                gamma = 0.0;
    }

    
    static bool loadControllersFromFile(
            std::string filename,
            std::shared_ptr<PidController::ControlCascades>& cc,
            std::vector<std::string> cascade_names,
            std::vector<std::string> dimension_names
        ){
            //while(1);
            YAML::Node node;
            try {
                node = YAML::LoadFile(filename);
            }
            catch(std::exception ex){
                std::cout << "Cannot parse YAML file with filename '" << filename << "'\nError message:\n" << ex.what();
            }
            
            cc->controllers.resize(cascade_names.size());
            cc->errors.resize(cascade_names.size());
            cc->error_rates.resize(cascade_names.size());
            cc->outputs.resize(cascade_names.size());
            
            for(size_t cascade_iter = 0; cascade_iter < cc->controllers.size(); cascade_iter++){
                cc->errors.at(cascade_iter).resize(dimension_names.size());
                cc->error_rates.at(cascade_iter).resize(dimension_names.size());
                cc->outputs.at(cascade_iter).resize(dimension_names.size());
                cc->controllers.at(cascade_iter).clear();
                for(size_t dimension_iter = 0; dimension_iter < dimension_names.size(); dimension_iter++){    
                    try
                    {
                        PID ctrl(
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["gains"][0].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["gains"][1].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["gains"][2].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["limits"][0].as<double>(),
                            node[cascade_names.at(cascade_iter)][dimension_names.at(dimension_iter)]["limits"][1].as<double>(),
                            &(cc->errors.at(cascade_iter).at(dimension_iter)),
                            &(cc->error_rates.at(cascade_iter).at(dimension_iter)),
                            &(cc->outputs.at(cascade_iter).at(dimension_iter))
                        );
                        cc->controllers.at(cascade_iter).push_back(ctrl);
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr 
                            << "Error loading element '" 
                            << cascade_names.at(cascade_iter) 
                            << " - " << dimension_names.at(dimension_iter) 
                            << "'. Can not find 3-element-double-list for key 'gains' and 2-element-double-list for key 'limits'.\nError Message:\n" 
                            << e.what() << std::endl;
                        return false;
                    }
                }
            }
            try{
                cc->alpha_limit = node["additional_limits"]["alpha"].as<double>();
                cc->beta_limit = node["additional_limits"]["beta"].as<double>();
            }
            catch(const std::exception& e)
            {
                std::cerr 
                    << "Error loading 'additional_limits'. Can not find double value for either 'alpha' or 'beta' under this key.\nError Message:\n" 
                    << e.what() << std::endl;
                return false;
            }
            return true;  
        }
    void updateControlLaw(RocketState current, RocketState set_point, double time_s){
        double dt = 1. / 20;
        prev_time = time_s;
        if(inner_inner_inner_iter == INNER_INNER_INNER_ITER){
          inner_inner_inner_iter = 0;
          cc->errors.at(0).at(0) = set_point.position.x - current.position.x;
          cc->errors.at(0).at(1) = set_point.position.y - current.position.y;
          cc->errors.at(0).at(2) = set_point.position.z - current.position.z;
          cc->error_rates.at(0).at(0) = set_point.velocity.x - current.velocity.x;
          cc->error_rates.at(0).at(1) = set_point.velocity.y - current.velocity.y;
          cc->error_rates.at(0).at(2) = set_point.velocity.z - current.velocity.z;
          
          cc->controllers.at(0).at(0).update(dt*INNER_INNER_INNER_ITER);
          cc->controllers.at(0).at(1).update(dt*INNER_INNER_INNER_ITER);
          cc->controllers.at(0).at(2).update(dt*INNER_INNER_INNER_ITER);
        }
        
        if(inner_inner_iter == INNER_INNER_ITER){
            inner_inner_iter = 0;
            cc->errors.at(1).at(0) = set_point.velocity.x + cc->outputs.at(0).at(0) - current.velocity.x;
            cc->errors.at(1).at(1) = set_point.velocity.y + cc->outputs.at(0).at(1) - current.velocity.y;
            cc->errors.at(1).at(2) = set_point.velocity.z + cc->outputs.at(0).at(2) - current.velocity.z;
            
            cc->controllers.at(1).at(0).update(dt*INNER_INNER_ITER);
            cc->controllers.at(1).at(1).update(dt*INNER_INNER_ITER);
            cc->controllers.at(1).at(2).update(dt*INNER_INNER_ITER);
            convertAccelerationToAngle(current.propeller_mass + rocket_model.props.dry_mass);
          }        
        if(inner_iter == INNER_ITER){
          inner_iter = 0;
          updateAnglesFromRocketState(current);
          
          
          cc->errors.at(2).at(0) = cc->outputs.at(1).at(0) - alpha;
          cc->errors.at(2).at(1) = cc->outputs.at(1).at(1) - beta;
          cc->errors.at(2).at(2) = cc->outputs.at(1).at(2) - gamma;

          cc->error_rates.at(2).at(0) = set_point.angular_velocity.x-1 * current.angular_velocity.x;
          cc->error_rates.at(2).at(1) = set_point.angular_velocity.y-1 * current.angular_velocity.y;
          cc->error_rates.at(2).at(2) = set_point.angular_velocity.z-1 * current.angular_velocity.z;
          
          cc->controllers.at(2).at(0).update(dt*INNER_ITER);
          cc->controllers.at(2).at(1).update(dt*INNER_ITER);
          cc->controllers.at(2).at(2).update(dt*INNER_ITER);
          
          
        }
        
        
        cc->errors.at(3).at(0) = set_point.angular_velocity.x + cc->outputs.at(2).at(0) - current.angular_velocity.x;
        cc->errors.at(3).at(1) = set_point.angular_velocity.y + cc->outputs.at(2).at(1) - current.angular_velocity.y;
        cc->errors.at(3).at(2) = set_point.angular_velocity.z + cc->outputs.at(2).at(2) - current.angular_velocity.z;
        
        cc->controllers.at(3).at(0).update(dt, true);
        cc->controllers.at(3).at(1).update(dt, true);
        cc->controllers.at(3).at(2).update(dt, true);
        
        inner_iter++;
        inner_inner_iter++;
        inner_inner_inner_iter++;
        controlAllocation();
        
        
        
    }
private:
    RocketModel rocket_model;
    std::shared_ptr<PidController::ControlCascades> cc;
    std::shared_ptr<std::pair<RocketGimbalControl, RocketControlMomentGyro>> control_output;

    double alpha, beta, gamma, Fz, prev_time;
    int inner_iter = 0;
    int inner_inner_iter = 0;
    int inner_inner_inner_iter = 0;

    static double getContinuousAngle(double newAngle, double prevAngle){
        double d = newAngle - prevAngle;
        d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        return prevAngle + d;
      }
    void updateAnglesFromRocketState(RocketState s){
        Eigen::Matrix<double,3,3> R = Eigen::Quaternion<double>(
            s.orientation.w,
            s.orientation.x,
            s.orientation.y,
            s.orientation.z
            ).toRotationMatrix();
          alpha = getContinuousAngle(atan2(-R(1,2),R(2,2)), alpha);
          beta = getContinuousAngle(atan2(R(0,2),sqrt(R(0,0) * R(0,0) + R(0,1) * R(0,1))), beta);
          gamma = getContinuousAngle(atan2(-R(0,1),R(0,0)), gamma);
    }
    void convertAccelerationToAngle(double mass){
        
        double tx(mass * cc->outputs.at(1).at(0));
        double ty(mass * cc->outputs.at(1).at(1));     
        double tz(mass * (G + cc->outputs.at(1).at(2)));
        //ROS_ERROR_STREAM("\nx" << tx << "\ny" << ty <<"\nz" << tz);
        cc->outputs.at(1).at(0) = std::max(std::min(-1 * atan2(ty, tz), cc->alpha_limit), -cc->alpha_limit);
        ty = -1*tan(cc->outputs.at(1).at(0)) * tz;
        cc->outputs.at(1).at(1) = std::max(std::min(cos(cc->outputs.at(1).at(0)) * atan2(tx, tz), cc->beta_limit), -cc->beta_limit);
        tx = tan(cc->outputs.at(1).at(1)) / cos(cc->outputs.at(1).at(0)) * tz;
        Fz = tz / cos(atan2(sqrt(tx*tx+ty*ty), tz));
      }
    
    void controlAllocation(){
        
        double Fy = cc->outputs.back().at(0) / rocket_model.props.dry_CM;
        double Fx = -cc->outputs.back().at(1) / rocket_model.props.dry_CM;
        
        double theta_1 = atan2(-Fy,Fz);
        double theta_2 = atan2(cos(theta_1) * Fx, Fz);
        double thrust = std::max(std::min(
          sqrt(Fz * Fz + Fy * Fy + Fx * Fx), (double)rocket_model.props.maxThrust.at(2)
        ), (double)rocket_model.props.minThrust.at(2));
        // Convert to gimbal_angle
        RocketGimbalControl gimbal_ctrl;
        gimbal_ctrl.inner_angle = theta_2;  // Rotation around body-y
        gimbal_ctrl.outer_angle = theta_1; // Rotation around body-x
        gimbal_ctrl.thrust = thrust;
        std::cout << gimbal_ctrl.thrust<<std::endl;
        this->control_output->first = gimbal_ctrl;
        RocketControlMomentGyro gyro_ctrl;
        gyro_ctrl.torque = 0.0;
        this->control_output->second = gyro_ctrl;
        
    }

    
    
};