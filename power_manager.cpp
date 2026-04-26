//
// Created by 3545 on 25-11-1.
//

#include "power_manager.h"

#include <vector>

//在结构体里填入电机模型参数，然后填入结构体初始化class
MotorPower::MotorPower(const motor_power_init_t &motor_power_init_data)
    :K0(motor_power_init_data.k0),
     K1(motor_power_init_data.k1),
     K2(motor_power_init_data.k2),
     K3(motor_power_init_data.k3),
     K4(motor_power_init_data.k4),
     K5(motor_power_init_data.k5),
     Current_Conversion(motor_power_init_data.real_current_conversion) {}

//内部参数来源于初始化时最后一个参数
double MotorPower::getMotorRealCurrent(const double current) const {
    const double real_current = current/Current_Conversion;
    return real_current;
}

//预测类型默认为关闭预测(指你要填入电机反馈电流，来获取反馈的功率)
//使用时请根据你填入的电流类型来填入对应的枚举
double MotorPower::update(double current ,double speed,const E_Predict_Status_Type Predict_status,const E_CalMotorPower_Negative_Status_Type Negative_Status) {
    const double product = current*speed;
    double power_sign = 1;

    if (Negative_Status == E_enable_negative) {
        if(product < 0 ) {
            power_sign = -1;
        }
    }

    current = std::abs(getMotorRealCurrent(current));
    speed = std::abs(speed);

    const double power =  (K0 +
                   K1 * current +
                   K2 * speed +
                   K3 * current * speed +
                   K4 * current * current +
                   K5 * speed * speed)*power_sign;

    if(Predict_status == E_enable_predict) {
        predict_power = power;
    }else if(Predict_status == E_disabled_predict) {
        feedback_power = power;
    }else if(Predict_status == E_enable_not_limit_predict) {
        predict_not_limit_power = power;
    }

    return power;
}

//会直接改变最终发给电机的输出值，同时也会return一个衰减系数
double MotorPower::limiter(double *desired_current,const double current_speed, const double motor_power_limit ) {

    if (desired_current == nullptr) {
        return 0.0;
    }

    power_limit = motor_power_limit;

    if(motor_power_limit < 0) {
        *desired_current = 0;
        update(*desired_current,current_speed,E_enable_predict);
        return 0.0;
    }

    const double desired_current_ = *desired_current;

    const double real_desired_current = std::abs(getMotorRealCurrent(desired_current_));
    const double real_current_speed = std::abs(current_speed);

    if (const double predicted_power = update(desired_current_, current_speed,E_enable_not_limit_predict);
       predicted_power <= motor_power_limit) {
        update(*desired_current,current_speed,E_enable_predict);
        return 1.0;
       }

    const double a = K4 * real_desired_current * real_desired_current;
    const double b = (K1 + K3 * real_current_speed) * real_desired_current;
    const double c = K0 + K2 * real_current_speed + K5 * real_current_speed * real_current_speed - motor_power_limit;
    const double discriminant = b * b - 4 * a * c;

    if (std::abs(a) < 1e-9) {
        if (std::abs(b) < 1e-9) {
            if(c <= 1e-9) {
                update(*desired_current,current_speed,E_enable_predict);
                return 1.0;
            }else {
                *desired_current = 0;
                update(*desired_current,current_speed,E_enable_predict);
                return 0.0;
            }
        } else {
            double k = -c / b;
            if (k < 0.0) {
                *desired_current = 0;
                update(*desired_current,current_speed,E_enable_predict);
                return 0.0;
            }
            if (k > 1.0) {
                update(*desired_current,current_speed,E_enable_predict);
                return 1.0;
            }
            update(*desired_current,current_speed,E_enable_predict);
            return k;
        }
    }

    if (discriminant < 0) {
        *desired_current = 0;
        update(*desired_current,current_speed,E_enable_predict);
        return 0.0;
    }

    if(std::abs(discriminant) < 1e-9) {
        double k = -1.0 * b / (2 * a);
        if(k < 0.0) {
            *desired_current = 0;
            update(*desired_current,current_speed,E_enable_predict);
            return 0.0;
        }
        if(k > 1.0) {
            update(*desired_current,current_speed,E_enable_predict);
            return 1.0;
        }
        *desired_current *= k;
        update(*desired_current,current_speed,E_enable_predict);
        return k;
    }

    const double k1 = (-b - std::sqrt(discriminant)) / (2 * a);
    const double k2 = (-b + std::sqrt(discriminant)) / (2 * a);

    if ( (k1 > 0.0 and k1 < 1.0 ) and (k2 > 0.0 and k2 < 1.0) ) {
        *desired_current *=  std::max(k1, k2);
        update(*desired_current,current_speed,E_enable_predict);
        return std::max(k1, k2);
    }else if ( (k1 > 0.0 and k1 < 1.0) and(k2 > 1.0 or k2 < 0.0) ) {
        *desired_current *= k1;
        update(*desired_current,current_speed,E_enable_predict);
        return k1;
    }else if ( (k1 < 0.0 or k1 > 1.0 ) and(k2 > 0.0 and k2 < 1.0) ) {
        *desired_current *= k2;
        update(*desired_current,current_speed,E_enable_predict);
        return k2;
    }else {
        *desired_current = 0;
        update(*desired_current,current_speed,E_enable_predict);
        return 0.0;
    }

}

//一些相关的宏定义见.h,默认参数为测试比较靠谱的值
//使用时效果不好请自行更改
std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit,const double buffer_power_attenuation) {

#ifdef M_Enable_PowerCompensation
    total_power_limit *= (1-M_Power_Compensation_Alpha);
#endif

    total_power_limit *= buffer_power_attenuation;

    if (motor_errors_vector.size() != 4) {
        return {0.0, 0.0, 0.0, 0.0} ;
    }

    for (double& error : motor_errors_vector) {
        error = std::abs(error);
    }

    if (total_power_limit <= 1e-9) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    const double total_error = motor_errors_vector[0]+motor_errors_vector[1]+motor_errors_vector[2]+motor_errors_vector[3];

    if (total_error <= M_Too_Small_AllErrors) {
        double equal_share = total_power_limit / motor_errors_vector.size();
        return {equal_share, equal_share, equal_share, equal_share};
    }

    std::vector<double> motor_power_limits_vector(4);
    if(total_power_limit < M_Motor_ReservedPower_Border) {
        for (int i = 0; i < 4; ++i) {
            const double ratio = motor_errors_vector[i] / total_error;
            motor_power_limits_vector[i] = ratio * total_power_limit;
        }
    }else {
        for (int j = 0; j < 4; ++j) {
            const double ratio = motor_errors_vector[j] / (total_error);
            motor_power_limits_vector[j] = ratio * (total_power_limit-4 * M_PerMotor_ReservedPower) + M_PerMotor_ReservedPower;
        }
    }

    return motor_power_limits_vector;
}

//整个底盘的class
//默认四个电机为一组，如果你用特殊构型请自行修改
//如果你用舵轮的话，建议对舵组和轮组建立两个class，然后使用下面分配舵轮功率函数来分配总功率
ChassisPowerManager::ChassisPowerManager(MotorPower* motor1, MotorPower* motor2, MotorPower* motor3, MotorPower* motor4)
: motors_(),motor_errors_() {
    motors_[0] = motor1;
    motors_[1] = motor2;
    motors_[2] = motor3;
    motors_[3] = motor4;
    motor_errors_.fill(0.0);
}

void ChassisPowerManager::updateMotorError(const size_t index, const double error) {
    if (index >= 4) return;
    motor_errors_[index] = std::abs(error);
}

void ChassisPowerManager::allocatePower(const double total_power_limit, const double buffer_power_attenuation) {
    std::vector<double> errors_vec(motor_errors_.begin(), motor_errors_.end());
    const std::vector<double> allocated_power = power_allocation_by_error(errors_vec, total_power_limit, buffer_power_attenuation);
    if (allocated_power.size() == 4) {
        for (size_t i = 0; i < 4; ++i) {
            motors_[i]->power_limit = allocated_power[i];
        }
    }
}

double ChassisPowerManager::getTotalPredictNotLimitPower() const {
    double total = 0.0;
    for (const auto* motor : motors_) {
        total += motor->predict_not_limit_power;
    }
    return total;
}

double ChassisPowerManager::getTotalPowerLimit() const {
    double total = 0.0;
    for (const auto* motor : motors_) {
        total += motor->power_limit;
    }
    return total;
}

double ChassisPowerManager::getTotalPredictPower() const {
    double total = 0.0;
    for (const auto* motor : motors_) {
        total += motor->predict_power;
    }
    return total;
}

//todo:未测试
std::vector<double> allocate_SW_power(const double total_power, const double servo_rate, const double servo_predict_want_power) {

    if( servo_rate > 1 or servo_rate <= 0) {
        //不正常，禁止分配
        return { 0.0,0.0 };
    }

    //[0]为舵的功率，[1]为轮的功率
    std::vector<double> servo_wheel_power_vector(2);
    double servo_max_allocate_power = total_power * servo_rate ;

    if(servo_predict_want_power >= servo_max_allocate_power) {
        servo_wheel_power_vector[0] = servo_max_allocate_power;
    }else if (servo_predict_want_power < servo_max_allocate_power) {
        servo_wheel_power_vector[0] = servo_predict_want_power;
    }
    servo_wheel_power_vector[1] = total_power - servo_wheel_power_vector[0];

    return servo_wheel_power_vector;
}

double rotate_speed_allocation(const int16_t vx, const int16_t vy, const int16_t rotate, const double alpha) {
    //简易分配，还可以加入最大功率限制的因素,效果不好就简单改下
    //实际比赛(底盘功率限制随等级增加的情况下)建议把速度和等级(最大功率)联系起来
    const double translation = sqrt(vx*vx + vy*vy);
    double adjusted_rotate = std::abs(rotate) - alpha * translation;
    if(std::abs(translation) <= 1e-9) {
        return rotate;
    }

    if(adjusted_rotate <0) {
        adjusted_rotate = 0;
        return adjusted_rotate;
    }

    if(rotate < 0) {
        return -adjusted_rotate;
    }
    return adjusted_rotate;

}

void rotate_theta_forwardfeed(double* theta,double rotate,double translation,double kp) {
    //简易补偿
    if(translation != 0) {
        //其实补偿的大小还跟旋转速度占比平移速度有关，甚至还可以跟最大功率有关，如果效果不好就简单改下
        //你也可以加权上面说的这个系数，比如乘以rotate / translation
        *theta = *theta - kp  * rotate;
    }
}

MovingAverageFilter::MovingAverageFilter(const size_t size)
    : size(size), index(0), count(0), sum(0.0) {
    buffer.resize(size, 0.0);
}
double MovingAverageFilter::update(const double new_value) {
    sum -= buffer[index];
    buffer[index] = new_value;
    sum += new_value;
    index = (index + 1) % size;
    if (count < size) count++;
    return sum / count;
}