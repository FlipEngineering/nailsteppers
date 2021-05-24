#include <MovementHandler.h>
#include <unity.h>

#include <iostream>
#include <stdio.h>
using namespace std;



void test_DualMovementValue(void){
    double error = 0.0;
    //printf("testX %d\n", 4);

    stepperSetup mySetup;

    mySetup.microsteps = 16;
    mySetup.gearTeeth = 20;
    mySetup.teethPitch = 2;
    mySetup.fullStepsPerRevolution = 200;
    

    DualMovementValue testValue = DualMovementValue(&mySetup);

    double distance_mm = 0;
    int distance_steps = 0;
    
    //printf("MM Test:\n");
    testValue.mm(0.09);
    
    distance_steps = testValue.steps();
    distance_mm = testValue.mm();
    //printf("Steps: %d - mm: %f\n\n\n", distance_steps, distance_mm);
    
    error =  testValue.get_error();
    printf("Accumulated Step Error: %f\n", error);
    
    TEST_ASSERT_EQUAL_INT(7, distance_steps);


        //printf("MM Test:\n");
    testValue.mm(0.09);
    
    distance_steps = testValue.steps();
    distance_mm = testValue.mm();
    //printf("Steps: %d - mm: %f\n\n\n", distance_steps, distance_mm);
    
    error =  testValue.get_error();
    printf("Accumulated Step Error: %f\n", error);
    
    TEST_ASSERT_EQUAL_INT(7, distance_steps);


        //printf("MM Test:\n");
    testValue.mm(0.09);
    
    distance_steps = testValue.steps();
    distance_mm = testValue.mm();
    //printf("Steps: %d - mm: %f\n\n\n", distance_steps, distance_mm);
    
    error =  testValue.get_error();
    printf("Accumulated Step Error: %f\n", error);
    
    TEST_ASSERT_EQUAL_INT(8, distance_steps);

        //printf("MM Test:\n");
    testValue.mm(0.09);
    
    distance_steps = testValue.steps();
    distance_mm = testValue.mm();
    //printf("Steps: %d - mm: %f\n\n\n", distance_steps, distance_mm);
    
    error =  testValue.get_error();
    printf("Accumulated Step Error: %f\n", error);
    
    TEST_ASSERT_EQUAL_INT(7, distance_steps);

    mySetup.microsteps = 32;
    //printf("Changing Microsteps to: %d\n", mySetup.microsteps);
    

    //printf("STEPS Test:\n");
    testValue.steps(1);
    distance_steps = testValue.steps();
    distance_mm = testValue.mm();
    double should_mm = 0.006250;
    TEST_ASSERT_EQUAL_DOUBLE(should_mm, distance_mm);


    //printf("Steps: %d - mm: %f\n\n\n", distance_steps, distance_mm);
    //TEST_ASSERT_EQUAL_INT(expected, actual)
    //TEST_ASSERT_EQUAL_INT(1, 1);
    
}

void speedTesting(){
    MovementHandler mh = MovementHandler();
    
    double travel_dist;
    bool check_safe_to_break_distance_test;
    double dist_break_test;
    double look_ahead_test;

    mh.init_sample();
    mh.c_v = 800;
    mh.c_a = 500;
    mh.run_sample();
    
    look_ahead_test = mh.look_ahead(100, -100, 49.99);
    //TEST_ASSERT_EQUAL_DOUBLE(1.41, look_ahead_test);
    printf("Look_ahead_test: %.12f\n", look_ahead_test);

    dist_break_test = mh.dist_break(800, -3200);
    TEST_ASSERT_EQUAL_DOUBLE(100.00, dist_break_test);
    printf("dist_break_test: %.2f\n", dist_break_test);


    travel_dist = 110;    
    mh.c_v = 800;
    check_safe_to_break_distance_test = mh.check_safe_to_break_distance(travel_dist);
    TEST_ASSERT_EQUAL_INT(1, check_safe_to_break_distance_test);

    travel_dist = 90;    
    mh.c_v = 800;
    check_safe_to_break_distance_test = mh.check_safe_to_break_distance(travel_dist);
    TEST_ASSERT_EQUAL_INT(0, check_safe_to_break_distance_test);
    
    printf("check_safe_to_break_distance_test: %d\n", check_safe_to_break_distance_test);

}

int main(int argc, char **argv){
    //cout << "-TEST-BEGIN-\n";
    //cout << "\n\n\n";
    

    UNITY_BEGIN();

    RUN_TEST(test_DualMovementValue);

    cout << "\n\n\n";
    speedTesting();

    UNITY_END();
    
    //cout << "\n\n\n";
    //cout << "-TEST-END-\n";

    return 0;
}