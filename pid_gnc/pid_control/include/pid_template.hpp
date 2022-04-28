class PID{
    public:
    PID(double P_gain, double I_gain, double D_gain, double lower_limit, double upper_limit, double* error, double* error_rate, double* output){
        this->P_gain = P_gain;
        this->D_gain = D_gain;
        this->I_gain = I_gain;
        this->error = error;
        this->error_rate = error_rate;
        this->output = output;
        this->lower_limit = lower_limit;
        this->upper_limit = upper_limit;
    }
    inline bool update(double dt, bool compute_error_rate_internally = false){
        if(dt <= 0) return false;
        this->integrator += *error * dt;
        // anti-windup:
        if(this->integrator * I_gain > upper_limit) this->integrator = upper_limit / I_gain;
        if(this->integrator * I_gain < lower_limit) this->integrator = lower_limit / I_gain;
        // Use simple derivative approximation if no better information is provided externally; should not be used (approximation formula should be improved)
        if(compute_error_rate_internally) *error_rate = (*error - previous_error) / dt;
        else if(this->integrator * I_gain < lower_limit) this->integrator = lower_limit / I_gain;
        *this->output = std::min(std::max(P_gain * *error + I_gain * integrator + D_gain * (*error_rate), lower_limit), upper_limit);
        this->previous_error = *error;
        return true;
    }
    private:
    double P_gain, D_gain, I_gain, lower_limit, upper_limit;
    double* error, *error_rate, *output, previous_error;
    double integrator=0; 

};