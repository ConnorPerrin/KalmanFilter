# pragma once

class KalmanFilter{
public:
    KalmanFilter(float inital_error_in_measurement, float inital_error_in_estimate, float inital_estimate){
        _error_in_measurement = inital_error_in_measurement;
        _error_in_estimate = inital_error_in_estimate;
        _previous_estimate = inital_estimate;
    }

    float predict(float measurement) {
        _current_measurement = measurement;

        calculateGain();
        calculateEstimation();
        calculateErrorEstimate();

        return _current_estimate;
    }

private:
    void calculateGain() {
        _gain = _error_in_estimate / (_error_in_estimate + _error_in_measurement);
    }

    void calculateEstimation(){
        _current_estimate = _previous_estimate + (_gain * (_current_measurement - _previous_estimate));
        _previous_estimate = _current_estimate;
    }

    void calculateErrorEstimate(){
        _error_in_estimate = (1 - _gain) * _error_in_estimate;
    }

private:
    float _gain;

    float _error_in_estimate;
    float _error_in_measurement;

    float _current_estimate;
    float _previous_estimate;

    float _current_measurement;
};