//
// Created by 3545 on 25-11-1.
//

#ifndef POWERCTRL_FORFRAMEWORK_H
#define POWERCTRL_FORFRAMEWORK_H

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#define M_Too_Small_AllErrors 500.0
#define M_Enable_PowerCompensation
#define M_Power_Compensation_Alpha 0.02
#define M_Motor_ReservedPower_Border 54.0
#define M_PerMotor_ReservedPower 8.0

typedef enum {
    E_disabled_negative,E_enable_negative
}E_CalMotorPower_Negative_Status_Type;
typedef enum {
    E_disabled_predict,E_enable_predict,E_enable_not_limit_predict
}E_Predict_Status_Type;

typedef struct{
    double k0,k1,k2,k3,k4,k5;
    double real_current_conversion;
}motor_power_init_t;

class MotorPower{
protected:
    const double K0,K1,K2,K3,K4,K5,Current_Conversion;
public:
    double feedback_power = 0;
    double predict_not_limit_power = 0;
    double predict_power = 0;
    double power_limit = 0;
    explicit MotorPower(const motor_power_init_t &motor_power_init_data);
    [[nodiscard]] double getMotorRealCurrent(double current) const;
    double update(double current, double speed,E_Predict_Status_Type Predict_status = E_disabled_predict, E_CalMotorPower_Negative_Status_Type Negative_Status = E_disabled_negative);
    double limiter(double *desired_current, double current_speed, double motor_power_limit);
};

class ChassisPowerManager {
protected:
    std::array<MotorPower*, 4> motors_;
    std::array<double, 4> motor_errors_;
public:
    ChassisPowerManager(MotorPower* motor1, MotorPower* motor2, MotorPower* motor3, MotorPower* motor4);
    void updateMotorError(size_t index, double error);
    void allocatePower(double total_power_limit, double buffer_power_attenuation = 1.0);
    [[nodiscard]] double getTotalPredictNotLimitPower() const;
    [[nodiscard]] double getTotalPowerLimit() const;
    [[nodiscard]] double getTotalPredictPower() const;
};

std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit,double buffer_power_attenuation = 1.0);
std::vector<double> allocate_SW_power( double total_power, double servo_rate, double servo_predict_want_power);
double rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, double alpha);
void rotate_theta_forwardfeed(double* theta,double rotate,double translation,double kp);

class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t size);
    double update(double new_value);

private:
    std::vector<double> buffer;
    size_t size;
    size_t index;
    size_t count;
    double sum;
};

#endif //POWERCTRL_FORFRAMEWORK_H
