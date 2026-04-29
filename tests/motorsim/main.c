#include "TinyFOC.h"
#include "stdio.h"
#include "communication/TinyFOCDebug.h"
#include "common/time_utils.h"
#include "math.h"
#define M_PI 3.14159265358979323846f
#define SQRT3 1.73205080756887729352f

void nl(){
    printf("\r\n");
}

void p(char * msg){
    printf(msg);
}

void p_f(float val, int digits){
    printf("%.*f", digits, val);
}

int r(char *val, int len){
    return 0;
}

void w(char c){
    printf("%c", c);
}

typedef struct {
    float R;          // Ohms (Phase resistance)
    float L;          // Henries (Phase inductance)
    float kv;         // RPM/V (Velocity constant)
    float ke;         // V/(rad/s) (Back-EMF constant, usually 60 / (sqrt(3) * pi * kv))
    float J;          // kg*m^2 (Inertia)
    int pole_pairs;   // Number of permanent magnet pole pairs
    float damping;         // Nms/rad (Viscous friction)
    float static_friction;  // Nm (Static friction torque)
} BLDCConfig;

typedef struct {
    float angle;      // Mechanical radians
    float velocity;   // rad/s
    float torque;     // Nm
    float current_a;  // Amps
    float current_b;
    float current_c;
    float voltage_a;  // Volts
    float voltage_b;  // Volts
    float voltage_c;  // Volts
    uint32_t last_update; // Timestamp of the last state update
} BLDCState;

BLDCState mybldc = {
    .angle = 0.0f,
    .velocity = 0.0f,
    .current_a = 0.0f,
    .current_b = 0.0f,
    .current_c = 0.0f,
    .voltage_a = 0.0f,
    .voltage_b = 0.0f,
    .voltage_c = 0.0f,
    .last_update = 0
};

BLDCConfig mybldc_config = {
    .R = 0.3f,
    .L = 0.0005f,
    .kv = 20.0f,
    .ke = .4,
    .J = 1e-3f,
    .pole_pairs = 15,
    .damping = 1e-2f,
    .static_friction = 0.01f
};

HallSensor mysensor;
FOCMotor mymotor;
CurrentSense mycs;
FOCDriver mydriver;

uint32_t cmicros = 0;
uint32_t lastprint = 0;

FILE *log_file;
FILE *log_file_sns;

typedef struct{
    uint32_t last_update;
    float values[5];
} LogEntry;

void update_motor(BLDCConfig* cfg, BLDCState* state, float v_a, float v_b, float v_c, float dt) {
    // 1. Calculate Electrical Angle
    float ele_angle = state->angle * cfg->pole_pairs;

    // 2. Simple Back-EMF calculation (Sinusoidal approximation)
    float bemf_a = cfg->ke * state->velocity * sinf(ele_angle);
    float bemf_b = cfg->ke * state->velocity * sinf(ele_angle - 2.0f * M_PI / 3.0f);
    float bemf_c = cfg->ke * state->velocity * sinf(ele_angle + 2.0f * M_PI / 3.0f);

    float v_neutral = (v_a + v_b + v_c) / 3.0f; // Neutral point voltage for wye connection

    // 3. Update Currents (V = I*R + L*di/dt + Bemf) -> di = (V - I*R - Bemf) * dt / L
    state->current_a += ((v_a - v_neutral) - state->current_a * cfg->R - bemf_a) * dt / cfg->L;
    state->current_b += ((v_b - v_neutral) - state->current_b * cfg->R - bemf_b) * dt / cfg->L;
    state->current_c += ((v_c - v_neutral) - state->current_c * cfg->R - bemf_c) * dt / cfg->L;

    // 4. Calculate Torque (T = Kt * I)
    float torque = cfg->ke * (state->current_a * sinf(ele_angle) + 
                             state->current_b * sinf(ele_angle - 2.0f * M_PI / 3.0f) + 
                             state->current_c * sinf(ele_angle + 2.0f * M_PI / 3.0f));

    
    float damping_torque = cfg->damping * state->velocity;
    torque -= damping_torque;

    float friction_torque = (fabsf(state->velocity) > 0.1f) ? (state->velocity < 0) ? -cfg->static_friction : cfg->static_friction : 0.0f;
    torque -= friction_torque;

    // 5. Physics Integration (Acceleration = Torque / Inertia)
    float accel = torque / cfg->J;
    state->velocity += accel * dt;
    state->angle += state->velocity * dt;
    state->torque = torque + damping_torque + friction_torque; // Store total torque including damping and friction for logging

    // Keep angle in 0 to 2PI
    if (state->angle > 2.0f * M_PI) state->angle -= 2.0f * M_PI;
    if (state->angle < 0) state->angle += 2.0f * M_PI;

    if(cmicros - lastprint > 1000){
        lastprint = cmicros;
        if (log_file) {
            fwrite(&mybldc, sizeof(BLDCState), 1, log_file);
        }

        LogEntry myentry = {
            .last_update = cmicros,
            .values = {FIX_TO_FLOAT(mysensor.angle_cache) + mymotor.sensor_offset, mysensor.hall_state, mysensor.direction, mysensor.electric_sector, mysensor.electric_rotations}
            // .values = {mysensor.angle_cache, mysensor.hall_state, mysensor.direction, mysensor.electric_sector, mysensor.electric_rotations}
        };
        if (log_file_sns) {
            fwrite(&myentry, sizeof(LogEntry), 1, log_file_sns);
        }
    }
}


uint32_t mymicros()
{
    cmicros ++;

    if(cmicros - mybldc.last_update > 10){
        float dt = (cmicros - mybldc.last_update) * 1e-6f;
        mybldc.last_update = cmicros;
        update_motor(&mybldc_config, &mybldc, mybldc.voltage_a, mybldc.voltage_b, mybldc.voltage_c, dt);
    }
    return cmicros;
}

void setPWM(FOCDriver_ll *driver, FIXP dcA, FIXP dcB, FIXP dcC) {
    BLDCState* state = (BLDCState*)driver->param;
    state->voltage_a = FIX_TO_FLOAT(dcA) *DEF_POWER_SUPPLY;
    state->voltage_b = FIX_TO_FLOAT(dcB) *DEF_POWER_SUPPLY;
    state->voltage_c = FIX_TO_FLOAT(dcC) *DEF_POWER_SUPPLY;

    _micros();
}

uint8_t get_hall_state(HallSensor_ll *ll) {
    BLDCState* state = (BLDCState*)ll->param;
    BLDCConfig* cfg = &mybldc_config;

    float ele_angle = fmodf(state->angle * cfg->pole_pairs, 2.0f * M_PI);
    if (ele_angle < 0) ele_angle += 2.0f * M_PI;

    int h1 = (ele_angle < M_PI) ? 1 : 0;
    int h2 = (ele_angle > 2.0f * M_PI / 3.0f && ele_angle < 5.0f * M_PI / 3.0f) ? 1 : 0;
    int h3 = (ele_angle > 4.0f * M_PI / 3.0f || ele_angle < M_PI / 3.0f) ? 1 : 0;

    return (h1 << 2) | (h2 << 1) | h3;
}

void readCurrents(CurrentSense_ll *ll, float* i_a, float* i_b, float* i_c) {
    BLDCState* state = (BLDCState*)ll->param;
    *i_a = state->current_a;
    *i_b = state->current_b;
    *i_c = state->current_c;
}

void noop(void *np){

}

HallSensor_ll hall_ll = {
    .init = noop,
    .read = get_hall_state,
    .initOK = true,
    .param = &mybldc
};

FOCDriver_ll foc_ll = {
    .init = noop,
    .enable = noop,
    .disable = noop,
    .dcA = 0.0f,
    .dcB = 0.0f,
    .dcC = 0.0f,
    .initOK = true,
    .param = &mybldc,
    .setpwm = setPWM
};

CurrentSense_ll cs_ll = {
    .init = noop,
    .readcurrents = readCurrents,
    .initOK = true,
    .param = &mybldc
};


int main(void)
{
    _micros = mymicros;
    Print myprint = {
        .newline = nl,
        .print = p,
        .print_f = p_f,
        .read = r,
        .write = w
    };

    log_file = fopen("motor_data.bin", "wb"); // 'wb' = Write Binary
    log_file_sns = fopen("sensor_data.bin", "wb"); // 'wb' = Write Binary

    TinyFOCDebug_enable(&myprint);

    printf("MyMicros: %lu\r\n", _micros());
    printf("MyMicros: %lu\r\n", _micros());

    
    BLDCMotor_load_default(&mymotor);
    mymotor.pole_pairs = mybldc_config.pole_pairs;
    mymotor.KV_rating = mybldc_config.kv;
    mymotor.axis_inductance.d = mybldc_config.L;
    mymotor.axis_inductance.q = mybldc_config.L;
    mymotor.phase_resistance = mybldc_config.R;

    FOCDriver_load_default(&mydriver);
    mydriver.ll = &foc_ll;
    
    CurrentSense_load_default(&mycs);
    mycs.ll = &cs_ll;
    CurrentSense_linkDriver(&mycs, &mydriver);
    

    
    HallSensor_load_default(&mysensor);
    mysensor.ll = &hall_ll;
    mysensor.pp = mymotor.pole_pairs;

    mymotor.linkCurrentSense(&mymotor, &mycs);
    mymotor.linkDriver(&mymotor, &mydriver);
    mymotor.linkSensor(&mymotor, &mysensor.sensor);
    mymotor.sensor_direction = Direction_UNKNOWN;
    mymotor.zero_electric_angle = NOT_SET;
    mymotor.controller = MotionControlType_velocity;
    mymotor.torque_controller = TorqueControlType_voltage;

    mysensor.sensor.init(&mysensor.sensor);
    mydriver.init(&mydriver);
    mymotor.init(&mymotor);
    mycs.init(&mycs);

    mymotor.initFOC(&mymotor);

    mymotor.move(&mymotor, 5); // Move to 5 rad/s

    for(int i = 0; i < 2000; i++){
        mymotor.loopFOC(&mymotor);
        _delay(1);
    }

    fclose(log_file);
    fclose(log_file_sns);
    return 0;
}