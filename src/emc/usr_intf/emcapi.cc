//
// Created by alex on 04.01.24.
//

#include "emcapi.hh"

//
// Created by alex on 04.01.24.
//

#include <iostream>


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

#include "hal.h"		/* access to HAL functions/definitions */
#include "rtapi.h"		/* rtapi_print_msg */
#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "nml_oi.hh"
#include "timer.hh"
#include <rtapi_string.h>
#include "tooldata.hh"

/* Using halui: see the man page */

static int axis_mask = 0;
#define JOGJOINT  1
#define JOGTELEOP 0

#define MDI_MAX 64

#pragma GCC diagnostic push
#if defined(__GNUC__) && (__GNUC__ > 4)
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif

static enum {
    LINEAR_UNITS_CUSTOM = 1,
    LINEAR_UNITS_AUTO,
    LINEAR_UNITS_MM,
    LINEAR_UNITS_INCH,
    LINEAR_UNITS_CM
} linearUnitConversion = LINEAR_UNITS_AUTO;

static enum {
    ANGULAR_UNITS_CUSTOM = 1,
    ANGULAR_UNITS_AUTO,
    ANGULAR_UNITS_DEG,
    ANGULAR_UNITS_RAD,
    ANGULAR_UNITS_GRAD
} angularUnitConversion = ANGULAR_UNITS_AUTO;

struct PTR {
    template<class T>
    struct field { typedef T *type; };
};

template<class T> struct NATIVE {};
template<> struct NATIVE<hal_bit_t> { typedef bool type; };
template<> struct NATIVE<hal_s32_t> { typedef rtapi_s32 type; };
template<> struct NATIVE<hal_u32_t> { typedef rtapi_u32 type; };
template<> struct NATIVE<hal_float_t> { typedef double type; };
struct VALUE {
    template<class T> struct field { typedef typename NATIVE<T>::type type; };
};

template<class T>
struct halui_str_base
{
//#define FIELD(t,f) typename T::template field<t>::type f;
//#define ARRAY(t,f,n) typename T::template field<t>::type f[n];
//    HAL_FIELDS
//#undef FIELD
//#undef ARRAY
};

typedef halui_str_base<PTR> halui_str;
typedef halui_str_base<VALUE> local_halui_str;
#pragma GCC diagnostic pop

static halui_str *halui_data;
static local_halui_str old_halui_data;

static char *mdi_commands[MDI_MAX];
static int num_mdi_commands=0;
static int have_home_all = 0;

static int comp_id, done;				/* component ID, main while loop */

static int num_axes = 0; //number of axes, taken from the INI [TRAJ] section
static int num_joints = 3; //number of joints, taken from the INI [KINS] section
static int num_spindles = 1; // number of spindles, [TRAJ]SPINDLES

static double maxFeedOverride=1;
static double maxMaxVelocity=1;
static double minSpindleOverride=0.0;
static double maxSpindleOverride=1.0;
static EMC_TASK_MODE_ENUM halui_old_mode = EMC_TASK_MODE_MANUAL;
static int halui_sent_mdi = 0;

// the NML channels to the EMC task
static RCS_CMD_CHANNEL *emcCommandBuffer = 0;
static RCS_STAT_CHANNEL *emcStatusBuffer = 0;
EMC_STAT *emcStatus = 0;

// the NML channel for errors
static NML *emcErrorBuffer = 0;

// the serial number to use.
static int emcCommandSerialNumber = 0;

// how long to wait for Task to report that it has received our command
static double receiveTimeout = 10.0;

// how long to wait for Task to finish running our command
static double doneTimeout = 60.;

static void quit(int sig)
{
    done = 1;
}

static int iniLoad(const char *filename)
{
    IniFile inifile;
    const char *inistring;
    double d;
    int i;

    // open it
    if (inifile.Open(filename) == false) {
        return -1;
    }

    if (NULL != (inistring = inifile.Find("DEBUG", "EMC"))) {
        // copy to global
        if (1 != sscanf(inistring, "%i", &emc_debug)) {
            emc_debug = 0;
        }
    } else {
        // not found, use default
        emc_debug = 0;
    }

    if (NULL != (inistring = inifile.Find("NML_FILE", "EMC"))) {
        // copy to global
        rtapi_strxcpy(emc_nmlfile, inistring);
    } else {
        // not found, use default
    }

    if (NULL != (inistring = inifile.Find("MAX_FEED_OVERRIDE", "DISPLAY"))) {
        if (1 == sscanf(inistring, "%lf", &d) && d > 0.0) {
            maxFeedOverride =  d;
        }
    }

    if(inifile.Find(&maxMaxVelocity, "MAX_LINEAR_VELOCITY", "TRAJ") &&
       inifile.Find(&maxMaxVelocity, "MAX_VELOCITY", "AXIS_X"))
        maxMaxVelocity = 1.0;

    if (NULL != (inistring = inifile.Find("MIN_SPINDLE_OVERRIDE", "DISPLAY"))) {
        if (1 == sscanf(inistring, "%lf", &d) && d > 0.0) {
            minSpindleOverride =  d;
        }
    }

    if (NULL != (inistring = inifile.Find("MAX_SPINDLE_OVERRIDE", "DISPLAY"))) {
        if (1 == sscanf(inistring, "%lf", &d) && d > 0.0) {
            maxSpindleOverride =  d;
        }
    }

    inistring = inifile.Find("COORDINATES", "TRAJ");
    num_axes = 0;
    if (inistring) {
        if(strchr(inistring, 'x') || strchr(inistring, 'X')) { axis_mask |= 0x0001; num_axes++; }
        if(strchr(inistring, 'y') || strchr(inistring, 'Y')) { axis_mask |= 0x0002; num_axes++; }
        if(strchr(inistring, 'z') || strchr(inistring, 'Z')) { axis_mask |= 0x0004; num_axes++; }
        if(strchr(inistring, 'a') || strchr(inistring, 'A')) { axis_mask |= 0x0008; num_axes++; }
        if(strchr(inistring, 'b') || strchr(inistring, 'B')) { axis_mask |= 0x0010; num_axes++; }
        if(strchr(inistring, 'c') || strchr(inistring, 'C')) { axis_mask |= 0x0020; num_axes++; }
        if(strchr(inistring, 'u') || strchr(inistring, 'U')) { axis_mask |= 0x0040; num_axes++; }
        if(strchr(inistring, 'v') || strchr(inistring, 'V')) { axis_mask |= 0x0080; num_axes++; }
        if(strchr(inistring, 'w') || strchr(inistring, 'W')) { axis_mask |= 0x0100; num_axes++; }
    }
    if (num_axes ==0) {
        rcs_print("halui: no [TRAJ]COORDINATES specified, enabling all axes\n");
        num_axes = EMCMOT_MAX_AXIS;
        axis_mask = 0xFFFF;
    }

    if (NULL != (inistring = inifile.Find("JOINTS", "KINS"))) {
        if (1 == sscanf(inistring, "%d", &i) && i > 0) {
            num_joints =  i;
        }
    }

    if (NULL != (inistring = inifile.Find("SPINDLES", "TRAJ"))) {
        if (1 == sscanf(inistring, "%d", &i) && i > 0) {
            num_spindles =  i;
        }
    }

    if (NULL != inifile.Find("HOME_SEQUENCE", "JOINT_0")) {
        have_home_all = 1;
    }

    if (NULL != (inistring = inifile.Find("LINEAR_UNITS", "DISPLAY"))) {
        if (!strcmp(inistring, "AUTO")) {
            linearUnitConversion = LINEAR_UNITS_AUTO;
        } else if (!strcmp(inistring, "INCH")) {
            linearUnitConversion = LINEAR_UNITS_INCH;
        } else if (!strcmp(inistring, "MM")) {
            linearUnitConversion = LINEAR_UNITS_MM;
        } else if (!strcmp(inistring, "CM")) {
            linearUnitConversion = LINEAR_UNITS_CM;
        }
    }

    if (NULL != (inistring = inifile.Find("ANGULAR_UNITS", "DISPLAY"))) {
        if (!strcmp(inistring, "AUTO")) {
            angularUnitConversion = ANGULAR_UNITS_AUTO;
        } else if (!strcmp(inistring, "DEG")) {
            angularUnitConversion = ANGULAR_UNITS_DEG;
        } else if (!strcmp(inistring, "RAD")) {
            angularUnitConversion = ANGULAR_UNITS_RAD;
        } else if (!strcmp(inistring, "GRAD")) {
            angularUnitConversion = ANGULAR_UNITS_GRAD;
        }
    }

    const char *mc;
    while(num_mdi_commands < MDI_MAX && (mc = inifile.Find("MDI_COMMAND", "HALUI", num_mdi_commands+1))) {
        mdi_commands[num_mdi_commands++] = strdup(mc);
    }

    // close it
    inifile.Close();

    return 0;
}

static int emcTaskNmlGet()
{
    int retval = 0;

    // try to connect to EMC cmd
    if (emcCommandBuffer == 0) {
        emcCommandBuffer =
                new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "xemc",
                                    emc_nmlfile);
        if (!emcCommandBuffer->valid()) {
            delete emcCommandBuffer;
            emcCommandBuffer = 0;
            retval = -1;
        }
    }
    // try to connect to EMC status
    if (emcStatusBuffer == 0) {
        emcStatusBuffer =
                new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "xemc",
                                     emc_nmlfile);
        if (!emcStatusBuffer->valid()) {
            delete emcStatusBuffer;
            emcStatusBuffer = 0;
            emcStatus = 0;
            retval = -1;
        } else {
            emcStatus = (EMC_STAT *) emcStatusBuffer->get_address();
        }
    }

    return retval;
}

static int emcErrorNmlGet()
{
    int retval = 0;

    if (emcErrorBuffer == 0) {
        emcErrorBuffer =
                new NML(nmlErrorFormat, "emcError", "xemc", emc_nmlfile);
        if (!emcErrorBuffer->valid()) {
            delete emcErrorBuffer;
            emcErrorBuffer = 0;
            retval = -1;
        }
    }

    return retval;
}

static int tryNml()
{
    double end;
    int good;
    const auto RETRY_TIME = 10.0;
    const auto RETRY_INTERVAL = 1.0;	// seconds between wait tries for a subsystem

    if ((emc_debug & EMC_DEBUG_NML) == 0) {
        set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
        // messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
        if (0 == emcTaskNmlGet()) {
            good = 1;
            break;
        }
        esleep(RETRY_INTERVAL);
        end -= RETRY_INTERVAL;
    } while (end > 0.0);
    if ((emc_debug & EMC_DEBUG_NML) == 0) {
        set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// inhibit diag
        // messages
    }
    if (!good) {
        return -1;
    }

    if ((emc_debug & EMC_DEBUG_NML) == 0) {
        set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
        // messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
        if (0 == emcErrorNmlGet()) {
            good = 1;
            break;
        }
        esleep(RETRY_INTERVAL);
        end -= RETRY_INTERVAL;
    } while (end > 0.0);
    if ((emc_debug & EMC_DEBUG_NML) == 0) {
        set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// inhibit diag
        // messages
    }
    if (!good) {
        return -1;
    }

    return 0;
}

static int updateStatus()
{
    NMLTYPE type;

    if (0 == emcStatus || 0 == emcStatusBuffer) {
        rtapi_print("halui: %s: no status buffer\n", __func__);
        return -1;
    }

    if (!emcStatusBuffer->valid()) {
        rtapi_print("halui: %s: status buffer is not valid\n", __func__);
        return -1;
    }

    switch (type = emcStatusBuffer->peek()) {
        case -1:
            // error on CMS channel
            rtapi_print("halui: %s: error peeking status buffer\n", __func__);
            return -1;
            break;

        case 0:			// no new data
        case EMC_STAT_TYPE:	// new data
            break;

        default:
            rtapi_print("halui: %s: unknown error peeking status buffer\n", __func__);
            return -1;
            break;
    }

    return 0;
}


#define EMC_COMMAND_DELAY   0.1	// how long to sleep between checks

static int emcCommandWaitDone()
{
    double end;
    for (end = 0.0; end < doneTimeout; end += EMC_COMMAND_DELAY) {
        updateStatus();
        int serial_diff = emcStatus->echo_serial_number - emcCommandSerialNumber;

        if (serial_diff < 0) {
            continue;
        }

        if (serial_diff > 0) {
            return 0;
        }

        if (emcStatus->status == RCS_DONE) {
            return 0;
        }

        if (emcStatus->status == RCS_ERROR) {
            return -1;
        }

        esleep(EMC_COMMAND_DELAY);
    }

    return -1;
}

static int emcCommandSend(RCS_CMD_MSG & cmd)
{
    // write command
    if (emcCommandBuffer->write(&cmd)) {
        rtapi_print("halui: %s: error writing to Task\n", __func__);
        return -1;
    }
    emcCommandSerialNumber = cmd.serial_number;

    // wait for receive
    double end;
    for (end = 0.0; end < receiveTimeout; end += EMC_COMMAND_DELAY) {
        updateStatus();
        int serial_diff = emcStatus->echo_serial_number - emcCommandSerialNumber;

        if (serial_diff >= 0) {
            return 0;
        }

        esleep(EMC_COMMAND_DELAY);
    }

    rtapi_print("halui: %s: no echo from Task after %.3f seconds\n", __func__, receiveTimeout);
    return -1;
}

static void thisQuit()
{
    //don't forget the big HAL sin ;)
    hal_exit(comp_id);

    if(emcCommandBuffer) { delete emcCommandBuffer;  emcCommandBuffer = 0; }
    if(emcStatusBuffer) { delete emcStatusBuffer;  emcStatusBuffer = 0; }
    if(emcErrorBuffer) { delete emcErrorBuffer;  emcErrorBuffer = 0; }
    exit(0);
}


#define CLOSE(a,b,eps) ((a)-(b) < +(eps) && (a)-(b) > -(eps))
#define LINEAR_CLOSENESS 0.0001
#define ANGULAR_CLOSENESS 0.0001
#define CM_PER_MM 0.1
#define GRAD_PER_DEG (100.0/90.0)
#define RAD_PER_DEG TO_RAD	// from posemath.h



// this function looks at the received NML status message
// and modifies the appropriate HAL pins
static void modify_hal_pins()
{
    int joint;
    int spindle;

    if (emcStatus->task.state == EMC_TASK_STATE_ON) {
        std::cout << "MON" << std::endl;
    } else {
        std::cout << "MOFF" << std::endl;
    }

    if (emcStatus->task.state == EMC_TASK_STATE_ESTOP) {
        std::cout << "ESTOP" << std::endl;
    } else {
        std::cout << "NO-ESTOP" << std::endl;
    }
//
//    if (halui_sent_mdi) { // we have an ongoing MDI command
//        if (emcStatus->status == 1) { //which seems to have finished
//            halui_sent_mdi = 0;
//            switch (halui_old_mode) {
//                case EMC_TASK_MODE_MANUAL: sendManual();break;
//                case EMC_TASK_MODE_MDI: break;
//                case EMC_TASK_MODE_AUTO: sendAuto();break;
//                default: sendManual();break;
//            }
//        }
//    }
//
//
//    if (emcStatus->task.mode == EMC_TASK_MODE_MANUAL) {
//        *(halui_data->mode_is_manual)=1;
//    } else {
//        *(halui_data->mode_is_manual)=0;
//    }
//
//    if (emcStatus->task.mode == EMC_TASK_MODE_AUTO) {
//        *(halui_data->mode_is_auto)=1;
//    } else {
//        *(halui_data->mode_is_auto)=0;
//    }
//
//    if (emcStatus->task.mode == EMC_TASK_MODE_MDI) {
//        *(halui_data->mode_is_mdi)=1;
//    } else {
//        *(halui_data->mode_is_mdi)=0;
//    }
//
//    if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE_TELEOP) {
//        *(halui_data->mode_is_teleop)=1;
//    } else {
//        *(halui_data->mode_is_teleop)=0;
//    }
//
//    if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE_FREE) {
//        *(halui_data->mode_is_joint)=1;
//    } else {
//        *(halui_data->mode_is_joint)=0;
//    }
//
//    *(halui_data->program_is_paused) = emcStatus->task.interpState == EMC_TASK_INTERP_PAUSED;
//    *(halui_data->program_is_running) = emcStatus->task.interpState == EMC_TASK_INTERP_READING ||
//                                        emcStatus->task.interpState == EMC_TASK_INTERP_WAITING;
//    *(halui_data->program_is_idle) = emcStatus->task.interpState == EMC_TASK_INTERP_IDLE;
//    *(halui_data->program_os_is_on) = emcStatus->task.optional_stop_state;
//    *(halui_data->program_bd_is_on) = emcStatus->task.block_delete_state;
//
//    *(halui_data->mv_value) = emcStatus->motion.traj.maxVelocity;
//    *(halui_data->fo_value) = emcStatus->motion.traj.scale; //feedoverride from 0 to 1 for 100%
//    *(halui_data->ro_value) = emcStatus->motion.traj.rapid_scale; //rapid override from 0 to 1 for 100%
//
//    *(halui_data->mist_is_on) = emcStatus->io.coolant.mist;
//    *(halui_data->flood_is_on) = emcStatus->io.coolant.flood;
//    *(halui_data->lube_is_on) = emcStatus->io.lube.on;
//
//    *(halui_data->tool_number) = emcStatus->io.tool.toolInSpindle;
//    *(halui_data->tool_length_offset_x) = emcStatus->task.toolOffset.tran.x;
//    *(halui_data->tool_length_offset_y) = emcStatus->task.toolOffset.tran.y;
//    *(halui_data->tool_length_offset_z) = emcStatus->task.toolOffset.tran.z;
//    *(halui_data->tool_length_offset_a) = emcStatus->task.toolOffset.a;
//    *(halui_data->tool_length_offset_b) = emcStatus->task.toolOffset.b;
//    *(halui_data->tool_length_offset_c) = emcStatus->task.toolOffset.c;
//    *(halui_data->tool_length_offset_u) = emcStatus->task.toolOffset.u;
//    *(halui_data->tool_length_offset_v) = emcStatus->task.toolOffset.v;
//    *(halui_data->tool_length_offset_w) = emcStatus->task.toolOffset.w;
//
//    if (emcStatus->io.tool.toolInSpindle == 0) {
//        *(halui_data->tool_diameter) = 0.0;
//    } else {
//        int idx;
//        for (idx = 0; idx <= tooldata_last_index_get(); idx ++) { // note <=
//            CANON_TOOL_TABLE tdata;
//            if (tooldata_get(&tdata,idx) != IDX_OK) {
//                fprintf(stderr,"UNEXPECTED idx %s %d\n",__FILE__,__LINE__);
//            }
//            if (tdata.toolno == emcStatus->io.tool.toolInSpindle) {
//                *(halui_data->tool_diameter) = tdata.diameter;
//                break;
//            }
//        }
//        if (idx == CANON_POCKETS_MAX) {
//            // didn't find the tool
//            *(halui_data->tool_diameter) = 0.0;
//        }
//    }
//
    for (spindle = 0; spindle < num_spindles; spindle++){
//        *(halui_data->spindle_is_on[spindle]) = (emcStatus->motion.spindle[spindle].enabled);
//        *(halui_data->spindle_runs_forward[spindle]) = (emcStatus->motion.spindle[spindle].direction == 1);
//        *(halui_data->spindle_runs_backward[spindle]) = (emcStatus->motion.spindle[spindle].direction == -1);
//        *(halui_data->spindle_brake_is_on[spindle]) = emcStatus->motion.spindle[spindle].brake;
//        *(halui_data->so_value[spindle]) = emcStatus->motion.spindle[spindle].spindle_scale; //spindle-speed-override from 0 to 1 for 100%
    }
//
    for (joint=0; joint < num_joints; joint++) {
//        *(halui_data->joint_is_homed[joint]) = emcStatus->motion.joint[joint].homed;
//        *(halui_data->joint_on_soft_min_limit[joint]) = emcStatus->motion.joint[joint].minSoftLimit;
//        *(halui_data->joint_on_soft_max_limit[joint]) = emcStatus->motion.joint[joint].maxSoftLimit;
//        *(halui_data->joint_on_hard_min_limit[joint]) = emcStatus->motion.joint[joint].minHardLimit;
//        *(halui_data->joint_on_hard_max_limit[joint]) = emcStatus->motion.joint[joint].maxHardLimit;
//        *(halui_data->joint_override_limits[joint]) = emcStatus->motion.joint[joint].overrideLimits;
//        *(halui_data->joint_has_fault[joint]) = emcStatus->motion.joint[joint].fault;
    }
//
    if (axis_mask & 0x0001) {
        // *(halui_data->axis_pos_commanded[0]) = emcStatus->motion.traj.position.tran.x;
        // *(halui_data->axis_pos_feedback[0]) = emcStatus->motion.traj.actualPosition.tran.x;
        std::cout << "x" << emcStatus->motion.traj.position.tran.x << "\t" << emcStatus->motion.traj.actualPosition.tran.x << std::endl;
        // double x = emcStatus->motion.traj.actualPosition.tran.x - emcStatus->task.g5x_offset.tran.x - emcStatus->task.toolOffset.tran.x;
        // double y = emcStatus->motion.traj.actualPosition.tran.y - emcStatus->task.g5x_offset.tran.y - emcStatus->task.toolOffset.tran.y;
        // x = x * cos(-emcStatus->task.rotation_xy * TO_RAD) - y * sin(-emcStatus->task.rotation_xy * TO_RAD);
        // *(halui_data->axis_pos_relative[0]) = x - emcStatus->task.g92_offset.tran.x;
    }
//
    if (axis_mask & 0x0002) {
//        *(halui_data->axis_pos_commanded[1]) = emcStatus->motion.traj.position.tran.y;
//        *(halui_data->axis_pos_feedback[1]) = emcStatus->motion.traj.actualPosition.tran.y;
//        double x = emcStatus->motion.traj.actualPosition.tran.x - emcStatus->task.g5x_offset.tran.x - emcStatus->task.toolOffset.tran.x;
//        double y = emcStatus->motion.traj.actualPosition.tran.y - emcStatus->task.g5x_offset.tran.y - emcStatus->task.toolOffset.tran.y;
//        y = y * cos(-emcStatus->task.rotation_xy * TO_RAD) + x * sin(-emcStatus->task.rotation_xy * TO_RAD);
//        *(halui_data->axis_pos_relative[1]) = y - emcStatus->task.g92_offset.tran.y;
    }
//
    if (axis_mask & 0x0004) {
        std::cout << "x" << emcStatus->motion.traj.position.tran.z << "\t" << emcStatus->motion.traj.actualPosition.tran.z << std::endl;
//        *(halui_data->axis_pos_commanded[2]) = emcStatus->motion.traj.position.tran.z;
//        *(halui_data->axis_pos_feedback[2]) = emcStatus->motion.traj.actualPosition.tran.z;
//        *(halui_data->axis_pos_relative[2]) = emcStatus->motion.traj.actualPosition.tran.z - emcStatus->task.g5x_offset.tran.z - emcStatus->task.g92_offset.tran.z - emcStatus->task.toolOffset.tran.z;
    }
//
//    if (axis_mask & 0x0008) {
//        *(halui_data->axis_pos_commanded[3]) = emcStatus->motion.traj.position.a;
//        *(halui_data->axis_pos_feedback[3]) = emcStatus->motion.traj.actualPosition.a;
//        *(halui_data->axis_pos_relative[3]) = emcStatus->motion.traj.actualPosition.a - emcStatus->task.g5x_offset.a - emcStatus->task.g92_offset.a - emcStatus->task.toolOffset.a;
//    }
//
//    if (axis_mask & 0x0010) {
//        *(halui_data->axis_pos_commanded[4]) = emcStatus->motion.traj.position.b;
//        *(halui_data->axis_pos_feedback[4]) = emcStatus->motion.traj.actualPosition.b;
//        *(halui_data->axis_pos_relative[4]) = emcStatus->motion.traj.actualPosition.b - emcStatus->task.g5x_offset.b - emcStatus->task.g92_offset.b - emcStatus->task.toolOffset.b;
//    }
//
//    if (axis_mask & 0x0020) {
//        *(halui_data->axis_pos_commanded[5]) = emcStatus->motion.traj.position.c;
//        *(halui_data->axis_pos_feedback[5]) = emcStatus->motion.traj.actualPosition.c;
//        *(halui_data->axis_pos_relative[5]) = emcStatus->motion.traj.actualPosition.c - emcStatus->task.g5x_offset.c - emcStatus->task.g92_offset.c - emcStatus->task.toolOffset.c;
//    }
//
//    if (axis_mask & 0x0040) {
//        *(halui_data->axis_pos_commanded[6]) = emcStatus->motion.traj.position.u;
//        *(halui_data->axis_pos_feedback[6]) = emcStatus->motion.traj.actualPosition.u;
//        *(halui_data->axis_pos_relative[6]) = emcStatus->motion.traj.actualPosition.u - emcStatus->task.g5x_offset.u - emcStatus->task.g92_offset.u - emcStatus->task.toolOffset.u;
//    }
//
//    if (axis_mask & 0x0080) {
//        *(halui_data->axis_pos_commanded[7]) = emcStatus->motion.traj.position.v;
//        *(halui_data->axis_pos_feedback[7]) = emcStatus->motion.traj.actualPosition.v;
//        *(halui_data->axis_pos_relative[7]) = emcStatus->motion.traj.actualPosition.v - emcStatus->task.g5x_offset.v - emcStatus->task.g92_offset.v - emcStatus->task.toolOffset.v;
//    }
//
//    if (axis_mask & 0x0100) {
//        *(halui_data->axis_pos_commanded[8]) = emcStatus->motion.traj.position.w;
//        *(halui_data->axis_pos_feedback[8]) = emcStatus->motion.traj.actualPosition.w;
//        *(halui_data->axis_pos_relative[8]) = emcStatus->motion.traj.actualPosition.w - emcStatus->task.g5x_offset.w - emcStatus->task.g92_offset.w - emcStatus->task.toolOffset.w;
//    }
//
//    *(halui_data->joint_is_homed[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].homed;
//    *(halui_data->joint_on_soft_min_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].minSoftLimit;
//    *(halui_data->joint_on_soft_max_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].maxSoftLimit;
//    *(halui_data->joint_on_hard_min_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].minHardLimit;
//    *(halui_data->joint_override_limits[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].overrideLimits;
//    *(halui_data->joint_on_hard_max_limit[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].maxHardLimit;
//    *(halui_data->joint_has_fault[num_joints]) = emcStatus->motion.joint[*(halui_data->joint_selected)].fault;
//
}


int EmcApi::init(const char* iniFilePath) {
//    // get configuration information
    //if (0 != iniLoad(emc_inifile)) {
    if (0 != iniLoad(iniFilePath)) {
        rcs_print_error("iniLoad error\n");
        return 2;
    }

//    //init HAL and export pins
//    if (0 != halui_hal_init()) {
//        rcs_print_error("hal_init error\n");
//        exit(1);
//    }

//    //initialize safe values
//    hal_init_pins();



    // init NML
    if (0 != tryNml()) {
        rcs_print_error("can't connect to emc\n");
        thisQuit();
        return 1;
    }

#ifdef TOOL_NML //{
    //fprintf(stderr,"%8d HALUI REGISTER %p\n",getpid(),
    tool_nml_register((CANON_TOOL_TABLE*)&emcStatus->io.tool.toolTable);
#else //}{
    tool_mmap_user();
#endif //}

    std::cout << "Lcnc Init" << std::endl;

}


void EmcApi::update() {
    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();
    modify_hal_pins();


//    done = 0;
//    /* Register the routine that catches the SIGINT signal */
//    signal(SIGINT, quit);
//    /* catch SIGTERM too - the run script uses it to shut things down */
//    signal(SIGTERM, quit);
//
//    while (!done) {
//        static bool task_start_synced = 0;
//        if (!task_start_synced) {
//            // wait for task to establish nonzero linearUnits
//            if (emcStatus->motion.traj.linearUnits != 0) {
//                // set once at startup, no changes are expected:
////                *(halui_data->units_per_mm) = emcStatus->motion.traj.linearUnits;
//                task_start_synced = 1;
//            }
//        }
////        check_hal_changes(); //if anything changed send NML messages
//        modify_hal_pins(); //if status changed modify HAL too
//        esleep(0.5); //sleep for a while
//        updateStatus();
//    }
//    thisQuit();



}

int main(int argc, const char *argv) {

}