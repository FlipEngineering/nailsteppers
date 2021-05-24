
#ifndef TICKHANDLER_H
#define TICKHANDLER_H

class TickHandler{
    public:

    //Debugging multiplier, if everything is okay bump it up to 1.0
    double be_careful = 0.2;
    
    double max_v = 200.0;
    double max_a = 800.0;

    double v_fac = 1.0/max_v;

    //Old
    double op = 0.0;
    double ov = 0.0;
    double oa = 0.0;

    //Current
    double cp = 0.0;
    double cv = 0.0;
    double ca = 0.0;

    //Target 1
    double tp1 = 0.0;
    double tv1 = 0.0;
    double ta1 = 0.0;

    //Target 2
    double tp2 = 0.0;
    double tv2 = 0.0;
    double ta2 = 0.0;

    //Tick loop delta time
    double ldt = 0.0;
    double oldt = 0.0; //old loop delta time
    double ldt_diff = 0.0;  //instability check

    //Target update loop delta time
    double tdt = 0.0;
    double otdt = 0.0;
    double tdt_diff = 0.0;  //instability check

    //Next velocity for stepper
    double nv = 0.0;
    //Resulting Accel of (nv-cv)/tdt
    double na = 0.0; 

    unsigned long c = 0;   //counter tick calls
    unsigned long ct = 0;  //counter target inputs
    unsigned long oct = 0; //old counter target
    int ct_diff = 0;

    double tick(double c_v, double c_p, double t_p1, double t_p2, double t_dt, double l_dt, long c_t){        
        c++;        //c  = counter tick   calls
        oct = ct;   //ct = counter target inputs
        ct = c_t;
        ct_diff = ct - oct;

        //old
        op = cp;
        ov = cv;
        oa = ca;

        //current
        cv = c_v;
        cp = c_p;
        //ca = (cp - op); //

        //target
        tp1 = t_p1;
        tp2 = t_p2;

        //target update loop delta time
        otdt = tdt;
        tdt = t_dt;
        tdt_diff = tdt - otdt;
        
        //tick loop delta time
        oldt = ldt;
        ldt = l_dt;
        ldt_diff = ldt - oldt;

        
        //Current Pos correct and target pos not changing
        if(cp == tp1 && tp1 == tp2){
            if(check_if_slow_enough_to_stop_moving()){
                nv = 0.0;
                return nv;
            }
        }
        
        nv = update_motion();


        return conv_v_to_fac(nv);
    }

    bool check_if_slow_enough_to_stop_moving(){
        if(cv == 0.0){
            return true;
        }
        return false;
    }

    double update_motion(){

        nv = (tp1 - cp) / tdt;
        if(nv > (cv*cv)){
            nv = cv*cv;
        }

        return nv;
    }

    double check_max(){
        //check if v and a are within max settings
        
        return 0.0;
    }

    double derive(double y1, double y2, double x1, double x2){
        double dy = y2 - y1;
        double dx = x2 - x1;

        return (dy / dx);

    }

    //Distance that would be moved
    double dist(double v, double a, double dt){
        if(a != 0.0){
            return (v + .5 * a * dt) * dt;
        }else{
            return (v * dt);
        }        

        return 0.0;
    }

    //Klipper attempt of keeping it easy in the beginning 
    double max_nv_from_cv(double v){    
        return (v*v);
    }

    TickHandler(double maxv, double maxa){
        max_v = maxv * be_careful;
        max_a = maxa * be_careful;
    }

    double conv_v_to_fac(double v){
        return v_fac * v;
    }

};


#endif