
//#include <iostream>
//#include <stdio.h>

#include <math.h>
//#include <vector>

//using namespace std;

#ifndef MOVEMENTHANDLER_H
#define MOVEMENTHANDLER_H

#define ENABLE_STEP_ERROR_COMPENSATION true


int custom_round(double x){
    if(x >= 0.0){
        return (int)(x+0.5);
    }else{
        return (int)(x-0.5);
    }
}

struct stepperSetup{
    int microsteps;
    int gearTeeth;
    int teethPitch;
    int fullStepsPerRevolution;
    double velocityMax;
    double accelerationMax;
};

class DualMovementValue{
    private:

    int last_ms = 0;

    double mm_value = 0.0;
    int steps_value = 0.0;

    double m2s = 0.0;
    double s2m = 0.0;

    double step_error = 0.0;
    double step_error_accumulated = 0.0;

    stepperSetup *s; 

    public:

    DualMovementValue(stepperSetup *ss){
        this->s = ss;
        this->updateFactors();
    }

    void checkForUpdateMicrosteps(){
        if(this->last_ms != this->s->microsteps){
            //printf("[MH-DMV-checkForUpdateMicrosteps] Updating Microsteps: old:%d, new:%d\n", this->last_ms, this->s->microsteps);
            this->updateFactors();            
        }
        this->last_ms = this->s->microsteps;
    }


    //Update Factors including the needed calulations based on setup
    void updateFactors(){
        this->last_ms = this->s->microsteps;

        double one_rev_step  = this->s->fullStepsPerRevolution * this->s->microsteps;
        double one_rev_mm    = this->s->teethPitch             * this->s->gearTeeth;

        this->m2s = one_rev_step / one_rev_mm;
        this->s2m = one_rev_mm / one_rev_step;
    }

    double mm(){
        return this->mm_value;
    }

    void mm(double value){        
        this->checkForUpdateMicrosteps();
        this->mm_value = value;
        this->mm_to_steps(value);
    }

    int steps(){
        return this->steps_value;
    }

    void steps(int value){
        this->checkForUpdateMicrosteps();
        this->steps_value = value;
        this->steps_to_mm(value);
    }

    /*
        Converting MM to Steps
        Because of the step size we can accumulate error in the conversion, trying to compensate for that
        should be carefully evaluated if that is working correctly or not. 
        Perhaps making it actually worse!
    */
    void mm_to_steps(double n_mm){
        if(ENABLE_STEP_ERROR_COMPENSATION){
            double mm_and_error_compensation = n_mm + this->step_error_accumulated;
            this->step_error_accumulated = 0.0;
            double t_step = mm_and_error_compensation * this->m2s;
            double t_step2 = custom_round(t_step);    
            this->step_error = t_step - t_step2;
            this->step_error *= this->s2m;
            this->step_error_accumulated += this->step_error;
            
            this->steps_value = t_step2;
        }else{
            this->steps_value = n_mm * this->m2s;
        }
        
    }

    void steps_to_mm(double n_steps){
        this->mm_value = n_steps * this->s2m;
    }

    double get_error(){
        return this->step_error_accumulated;
    }
};

class Acceleration{
    public:

    Acceleration(){

    }

};

class Velocity{
    public: 

    Velocity(){

    }
};

/* MovementHandler v-1
class MovementHandler{
    private:

    stepperSetup *s;

    //s.microsteps = 32;
    //s.gearTeeth = 20;
    //s.teethPitch = 2;
    //s.fullStepsPerRevolution = 200;
    //s.velocityMax = 800; // max vel. mm/sec
    //s.accelerationMax = 3200; // max acc. mm/sec²    

    const static int data_count = 3; //3 is min otherwise crash with vc,vp and vf
    DualMovementValue *v[data_count];

    int vc_index = (custom_round(((double)data_count)/2.0)-1);

    DualMovementValue *vc = v[vc_index];    //current
    DualMovementValue *vp = v[vc_index-1];  //past
    DualMovementValue *vf = v[vc_index+1];  //future
    

    public:
    MovementHandler(stepperSetup *ss){
        this->s = ss;

        for(int i=0; i<data_count;i++){
            this->v[i] = new DualMovementValue(ss);
        }        
    }

    double velocity(int time){
        double accel = this->s->accelerationMax;

        this->vc->mm(this->vp->mm() + (accel * time));

        // Look ahead
        // end_velocity^2 = start_velocity^2 + 2*accel*move_distance
        return 0.0;
    }
    
};
*/

class MovementHandler{
    public:

    const static int position_buffer_size = 3;

    double max_a = 0.0;
    double max_v = 0.0;

    //c_ = current
    double c_v = 0.0;       // velocity    mm/sec
    double c_a = 0.0;       // accel       mm/sec^2
    double c_dt = 0.03;     // time        sec

    //n_ = next
    double n_v = 0.0;
    double n_a = 0.0;
    double n_dt = 0.03; 

    double c_dist = 0.0;

    //vector<double> next_positions;
    double c_pos = 0.0;
    //vector<double> past_positions;


    

    //Constant Acceleration
    //velocity(time) = start_velocity + accel*time;
    double v_const_a(double v, double a, double dt){
        return (v + a*dt);
    }

    double dist_cruise_v(double dt){
         return dist(c_v, 0, dt);
    }

    //Distance that would be moved
    double dist(double v, double a, double dt){
        if(a != 0.0){
            return (v + .5 * a * dt) * dt;
        }else{
            return (v * dt);
        }        
    }

    double t_from_v_nv_a(double v, double nv, double a){
        return ((nv - v) / a); 
    }

    double dist_const_a_no_time(double v, double nv, double a){
        double t = t_from_v_nv_a(v,nv,a);

        return ((v + 0.5 * a * t) * t);
    }

    double dist_break(double v, double a){
        return dist_const_a_no_time(v,0,a);
    }

    bool check_safe_to_break_distance(double dist, double safetyFactor = 1.0){
        double d = dist_break(c_v, -max_a) * safetyFactor; //Distance needed to break, including safety factor

        if(dist > d){
            return true;
            //we don't have to break yet
        }else{
            return false;
            //we should start breaking, check how hard.
        }
    }
            
    double a_to_pos_in_dt(double v, double dist, double dt){
        return ((dist/dt) - v) / (.5 * dt);
    }

    double look_ahead(double v, double a, double dist){
        //Look ahead key formular 
        //end_velocity^2 = start_velocity^2 + 2*accel*move_distance
        double end_v_sqrt = (v*v) + 2 * a * dist;
        return sqrt(end_v_sqrt);
    }

    double max_nv_from_cv(double v){    
        return (v*v);
    }

    void init_sample(){
        max_a = 3200;
        max_v = 800;
    }
            
    void run_sample(){
        double last_position = 150; //mm
        double next_position = 170;    
        double c_dist = next_position - last_position;
        
        double d_cr_v = dist_cruise_v(c_dt);

        if(d_cr_v == c_dist){
            //do nothing because we are spot on
            //printf("We are having the correct speed!");
        }else{
            //calculate acceleration needed to arrive good
            n_a = a_to_pos_in_dt(c_v, c_dist, c_dt);

            //printf("n_a to pos in dt: %.2f mm/sec2\n\n", n_a);


        }
        //printf("c_dist: %.2f mm \nn_dist at cr_v: %.2f \n\n", c_dist, d_cr_v);



    }

    

    void eval_move_options(){
        
    }

    MovementHandler(){

    }
};

#endif