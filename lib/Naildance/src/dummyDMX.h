class dummyDMX{
    private:
    int freq = 30; //try to send 30 values per sec
    
    double loop_time = 1.0/freq;


    double pos_min = 0.0;   //mm
    double pos_max = 200.0; //mm

    public:
    dummyDMX(double frequency = 0.0, double position_max = 0.0){

        if (frequency != 0.0){
            freq = frequency;
            loop_time = 1.0/freq;
        }

        if(position_max != 0.0){
            pos_max = position_max;
        }

        

    }
};
