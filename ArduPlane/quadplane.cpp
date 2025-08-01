#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include "AC_AttitudeControl/AC_AttitudeControl_TS.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.
    // @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPVARPTR(motors, "M_", 2, QuadPlane, plane.quadplane.motors_var_info),

    // 3 ~ 8 were used by quadplane attitude control PIDs

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all VTOL flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 3000),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: ms
    // @Range: 500 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000),

    // 12 ~ 16 were used by position, velocity and acceleration PIDs

    // @Group: P
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPPTR(pos_control, "P", 17, QuadPlane, AC_PosControl),

    // @Param: PILOT_SPD_UP
    // @DisplayName: Pilot maximum vertical speed up
    // @Description: The maximum ascending vertical velocity the pilot may request in m/s
    // @Units: m/s
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PILOT_SPD_UP", 18, QuadPlane, pilot_speed_z_max_up_ms, 2.50),
   
    // @Param: PILOT_SPD_DN
    // @DisplayName: Pilot maximum vertical speed down
    // @Description: The maximum vertical velocity the pilot may request in m/s going down. If 0, uses Q_PILOT_SPD_UP value.
    // @Units: m/s
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PILOT_SPD_DN", 60, QuadPlane, pilot_speed_z_max_dn_ms, 0),

     // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: m/s/s
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PILOT_ACCEL_Z",  19, QuadPlane, pilot_accel_z_mss,  2.5),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPPTR(wp_nav, "WP_",  20, QuadPlane, AC_WPNav),

    // @Param: RC_SPEED
    // @DisplayName: RC output speed in Hz
    // @Description: This is the PWM refresh rate in Hz for QuadPlane quad motors
    // @Units: Hz
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RC_SPEED", 21, QuadPlane, rc_speed, 490),

    // @Param: THR_MIN_PWM
    // @DisplayName: Minimum PWM output
    // @Description: This is the minimum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    // 22: THR_MIN_PWM

    // @Param: THR_MAX_PWM
    // @DisplayName: Maximum PWM output
    // @Description: This is the maximum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    // 23: THR_MAX_PWM

    // @Param: ASSIST_SPEED
    // @DisplayName: Quadplane assistance speed
    // @Description: This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. The default value of 0 disables assistance but will generate a pre-arm failure to encourage users to set this parameter to -1, or a positive, non-zero value. If this is set to -1 then all Q_ASSIST features are disabled except during transitions. A high non-zero,positive value will lead to more false positives which can waste battery. A lower value will result in less false positive, but will result in assistance taking longer to trigger. If unsure then set to 3 m/s below the minimum airspeed you will fly at. If you don't have an airspeed sensor then use 5 m/s below the minimum airspeed you fly at.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_SPEED", 24, QuadPlane, assist.speed, 0),

    // @Param: YAW_RATE_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: This is the maximum yaw rate for pilot input on rudder stick in degrees/second
    // @Units: deg/s
    // @Range: 50 500
    // @Increment: 1
    // @User: Standard

    // YAW_RATE_MAX index 25

    // @Param: LAND_FINAL_SPD
    // @DisplayName: Land final speed
    // @Description: The descent speed for the final stage of landing in m/s
    // @Units: m/s
    // @Range: 0.3 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_SPD", 26, QuadPlane, land_final_speed_ms, 0.5),

    // @Param: LAND_FINAL_ALT
    // @DisplayName: Land final altitude
    // @Description: The altitude at which we should switch to Q_LAND_SPEED descent rate
    // @Units: m
    // @Range: 0.5 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_ALT", 27, QuadPlane, land_final_alt_m, 6),

    // 28 was used by THR_MID

    // @Param: TRAN_PIT_MAX
    // @DisplayName: Transition max pitch
    // @Description: Maximum pitch during transition to auto fixed wing flight
    // @User: Standard
    // @Range: 0 30
    // @Units: deg
    // @Increment: 1
    AP_GROUPINFO("TRAN_PIT_MAX", 29, QuadPlane, transition_pitch_max, 3),

    // frame class was moved from 30 when consolidating AP_Motors classes

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 7:Tri, 10: Single/Dual, 12:DodecaHexa, 14:Deca, 15:Scripting Matrix, 17:Dynamic Scripting Matrix
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15:I, 16:MOTOR_FRAME_TYPE_NYT_PLUS, 17:MOTOR_FRAME_TYPE_NYT_X, 18: BetaFlightXReversed, 19: Y4
    // @User: Standard
    AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1),

    // @Param: VFWD_GAIN
    // @DisplayName: Forward velocity hold gain
    // @Description: The use of this parameter is no longer recommended and has been superseded by a method that works in all VTOL modes with the exception of QAUTOTUNE which is controlled by the Q_FWD_THR_USE parameter. This Q_VFD_GAIN parameter controls use of the forward motor in VTOL modes that use the velocity controller. Set to 0 to disable this function. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use with QLOITER for position hold with the forward motor. 
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("VFWD_GAIN", 32, QuadPlane, vel_forward.gain, 0),

    // 33 was used by WVANE_GAIN

    // 34 was used by WVANE_MINROLL

    // @Param: RTL_ALT
    // @DisplayName: QRTL return altitude
    // @Description: The altitude which QRTL mode heads to initially
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt_m, 15),

    // @Param: RTL_MODE
    // @DisplayName: VTOL RTL mode
    // @Description: If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination, VTOL approach: vehicle will RTL at RTL alt and circle with a radius of Q_FW_LND_APR_RAD down to Q_RTL_ALT and then transition into the wind and QRTL, see 'AUTO VTOL Landing', QRTL Always: do a QRTL instead of RTL
    // @Values: 0:Disabled,1:Enabled,2:VTOL approach,3:QRTL Always
    // @User: Standard
    AP_GROUPINFO("RTL_MODE", 36, QuadPlane, rtl_mode, 0),

    // 37: TILT_MASK
    // 38: TILT_RATE_UP
    // 39: TILT_MAX

    // @Param: GUIDED_MODE
    // @DisplayName: Enable VTOL in GUIDED mode
    // @Description: This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GUIDED_MODE", 40, QuadPlane, guided_mode, 0),

    // 41 was used by THR_MIN

    // @Param: ESC_CAL
    // @DisplayName: ESC Calibration
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read https://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Standard
    AP_GROUPINFO("VFWD_ALT", 43, QuadPlane, vel_forward_alt_cutoff_m,  0),

    // @Param: LAND_ICE_CUT
    // @DisplayName: Cut IC engine on landing
    // @Description: This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LAND_ICE_CUT", 44, QuadPlane, land_icengine_cut,  1),
    
    // @Param: ASSIST_ANGLE
    // @DisplayName: Quadplane assistance angle
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also positive and non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least Q_ASSIST_DELAY seconds. Set to zero to disable angle assistance.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist.angle, 30),

    // @Param: ASSIST_OPTIONS
    // @DisplayName: Quadplane assistance options
    // @Description: Options for special QAssist features
    // @Bitmask: 0: Disable force fixed wing controller recovery
    // @Bitmask: 1: Disable quadplane spin recovery
    // @User: Standard
    AP_GROUPINFO("ASSIST_OPTIONS", 47, QuadPlane, assist.options, 0),
    
    // 47: TILT_TYPE // was AP_Int8, re-used by AP_Int16 ASSIST_OPTIONS
    // 48: TAILSIT_ANGLE
    // 61: TAILSIT_ANG_VT
    // 49: TILT_RATE_DN
    // 50: TAILSIT_INPUT
    // 51: TAILSIT_MASK
    // 52: TAILSIT_MASKCH
    // 53: TAILSIT_VFGAIN
    // 54: TAILSIT_VHGAIN
    // 56: TAILSIT_VHPOW

    // @Param: MAV_TYPE
    // @DisplayName: MAVLink type identifier
    // @Description: This controls the mavlink type given in HEARTBEAT messages. For some GCS types a particular setting will be needed for correct operation.
    // @Values: 0:AUTO,1:FIXED_WING,2:QUADROTOR,3:COAXIAL,4:HELICOPTER,7:AIRSHIP,8:FREE_BALLOON,9:ROCKET,10:GROUND_ROVER,11:SURFACE_BOAT,12:SUBMARINE,16:FLAPPING_WING,17:KITE,19:VTOL_DUOROTOR,20:VTOL_QUADROTOR,21:VTOL_TILTROTOR
    AP_GROUPINFO("MAV_TYPE", 57, QuadPlane, mav_type, 0),

    // @Param: OPTIONS
    // @DisplayName: quadplane options
    // @Description: See description for each bitmask bit description
    // @Bitmask: 0: Level Transition-keep wings within LEVEL_ROLL_LIMIT and only use forward motor(s) for climb during transition
    // @Bitmask: 1: Allow FW Takeoff-if bit is not set then NAV_TAKEOFF command on quadplanes will instead perform a NAV_VTOL takeoff
    // @Bitmask: 2: Allow FW Land-if bit is not set then NAV_LAND command on quadplanes will instead perform a NAV_VTOL_LAND
    // @Bitmask: 3: Vtol Takeoff Frame-command NAV_VTOL_TAKEOFF alt set by the command's reference frame not above current location
    // @Bitmask: 4: Always use FW spiral approach-always use Use a fixed wing spiral approach for VTOL landings
    // @Bitmask: 5: USE QRTL-instead of QLAND for rc failsafe when in VTOL modes
    // @Bitmask: 6: Use Governor-use ICE Idle Governor in MANUAL for forward motor
    // @Bitmask: 7: Force Qassist-on always
    // @Bitmask: 8: Mtrs_Only_Qassist-in tailsitters only uses VTOL motors and not flying surfaces for QASSIST
    // @Bitmask: 10: Disarmed Yaw Tilt-enable motor tilt for yaw when disarmed
    // @Bitmask: 11: Delay Spoolup-delay VTOL spoolup for 2 seconds after arming
    // @Bitmask: 12: Disable speed based Qassist when using synthetic airspeed estimates
    // @Bitmask: 13: Disable Ground Effect Compensation-on baro altitude reports
    // @Bitmask: 14: Ignore forward flight angle limits-in Qmodes and use Q_ANGLE_MAX exclusively
    // @Bitmask: 15: ThrLandControl-enable throttle stick control of landing rate
    // @Bitmask: 16: DisableApproach-disable use of approach and airbrake stages in VTOL landing
    // @Bitmask: 17: EnableLandResposition-enable pilot controlled repositioning in AUTO land.Descent will pause while repositioning
    // @Bitmask: 18: ARMVTOL-arm only in VTOL modes (or AUTO mode when current nav cmd is VTOL Takeoff)
    // @Bitmask: 19: CompleteTransition-to fixed wing if Q_TRANS_FAIL timer times out instead of QLAND
    // @Bitmask: 20: Force RTL mode-forces RTL mode on rc failsafe in VTOL modes overriding bit 5(USE_QRTL)
    // @Bitmask: 21: Tilt rotor-tilt motors up when disarmed in FW modes (except manual) to prevent ground strikes.
    // @Bitmask: 22: Scale FF by the ratio of VTOL to plane angle P gains in Position 1 phase of transition into VTOL flight as well as reducing VTOL angle P based on airspeed.
    AP_GROUPINFO("OPTIONS", 58, QuadPlane, options, 0),

    AP_SUBGROUPEXTENSION("",59, QuadPlane, var_info2),

    // 60 is used above for VELZ_MAX_DN
    // 61 was used above for TAILSIT_ANG_VT

    AP_GROUPEND
};

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo QuadPlane::var_info2[] = {
    // @Param: TRANS_DECEL
    // @DisplayName: Transition deceleration
    // @Description: This is deceleration rate that will be used in calculating the stopping distance when transitioning from fixed wing flight to multicopter flight.
    // @Units: m/s/s
    // @Increment: 0.1
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("TRANS_DECEL", 1, QuadPlane, transition_decel_mss, 2.0),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    AP_SUBGROUPPTR(loiter_nav, "LOIT_",  2, QuadPlane, AC_Loiter),

    // 3: TAILSIT_GSCMAX

    // @Param: TRIM_PITCH
    // @DisplayName: Quadplane AHRS trim pitch
    // @Description: This sets the compensation for the pitch angle trim difference between calibrated AHRS level and vertical flight pitch. NOTE! this is relative to calibrated AHRS trim, not forward flight trim which includes PTCH_TRIM_DEG. For tailsitters, this is relative to a baseline of 90 degrees in AHRS.
    // @Units: deg
    // @Range: -10 +10
    // @Increment: 0.1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TRIM_PITCH", 4, QuadPlane, ahrs_trim_pitch, 0),

    // 5: TAILSIT_RLL_MX

#if QAUTOTUNE_ENABLED
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp
    AP_SUBGROUPINFO(qautotune, "AUTOTUNE_",  6, QuadPlane, QAutoTune),
#endif

    // @Param: FW_LND_APR_RAD
    // @DisplayName: Quadplane fixed wing landing approach radius
    // @Description: This provides the radius used, when using a fixed wing landing approach. If set to 0 then the WP_LOITER_RAD will be selected.
    // @Units: m
    // @Range: 0 200
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("FW_LND_APR_RAD", 7, QuadPlane, fw_land_approach_radius_m, 0),

    // @Param: TRANS_FAIL
    // @DisplayName: Quadplane transition failure time
    // @Description: Maximum time allowed for forward transitions, exceeding this time will cancel the transition and the aircraft will immediately change to the mode set by Q_TRANS_FAIL_ACT or finish the transition depending on Q_OPTIONS bit 19. 0 for no limit.
    // @Units: s
    // @Range: 0 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRANS_FAIL", 8, QuadPlane, transition_failure.timeout, 0),

    // 9: TAILSIT_MOTMX

    // @Param: THROTTLE_EXPO
    // @DisplayName: Throttle expo strength
    // @Description: Amount of curvature in throttle curve: 0 is linear, 1 is cubic
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THROTTLE_EXPO", 10, QuadPlane, throttle_expo, 0.2),

    // @Param: ACRO_RLL_RATE
    // @DisplayName: QACRO mode roll rate
    // @Description: The maximum roll rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_RLL_RATE", 11, QuadPlane, acro_roll_rate, 360),

    // @Param: ACRO_PIT_RATE
    // @DisplayName: QACRO mode pitch rate
    // @Description: The maximum pitch rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_PIT_RATE", 12, QuadPlane, acro_pitch_rate, 180),

    // @Param: ACRO_YAW_RATE
    // @DisplayName: QACRO mode yaw rate
    // @Description: The maximum yaw rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_YAW_RATE", 13, QuadPlane, acro_yaw_rate, 90),

    // @Param: TKOFF_FAIL_SCL
    // @DisplayName: Takeoff time failure scalar
    // @Description: Scalar for how long past the expected takeoff time a takeoff should be considered as failed and the vehicle will switch to QLAND. If set to 0 there is no limit on takeoff time.
    // @Range: 1.1 5.0
    // @Increment: 5.1
    // @User: Advanced
    AP_GROUPINFO("TKOFF_FAIL_SCL", 14, QuadPlane, takeoff_failure_scalar, 0),

    // @Param: TKOFF_ARSP_LIM
    // @DisplayName: Takeoff airspeed limit
    // @Description: Airspeed limit during takeoff. If the airspeed exceeds this level the vehicle will switch to QLAND. This is useful for ensuring that you don't takeoff into excessively strong wind. If set to 0 there is no limit on airspeed during takeoff.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TKOFF_ARSP_LIM", 15, QuadPlane, maximum_takeoff_airspeed_ms, 0),

    // @Param: ASSIST_ALT
    // @DisplayName: Quadplane assistance altitude
    // @Description: This is the altitude below which quadplane assistance will be triggered. This acts the same way as Q_ASSIST_ANGLE and Q_ASSIST_SPEED, but triggers if the aircraft drops below the given altitude while the VTOL motors are not running. A value of zero disables this feature. The altitude is calculated as being above ground level. The height above ground is given from a Lidar used if available and RNGFND_LANDING=1. Otherwise it comes from terrain data if TERRAIN_FOLLOW=1 and comes from height above home otherwise.
    // @Units: m
    // @Range: 0 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ALT", 16, QuadPlane, assist.alt, 0),

    // 17: TAILSIT_GSCMSK
    // 18: TAILSIT_GSCMIN

    // @Param: ASSIST_DELAY
    // @DisplayName: Quadplane assistance delay
    // @Description: This is delay between the assistance thresholds being met and the assistance starting.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_DELAY", 19, QuadPlane, assist.delay, 0.5),
    
    // @Param: FWD_MANTHR_MAX
    // @DisplayName: VTOL manual forward throttle max percent
    // @Description: Maximum value for manual forward throttle; used with RC option FWD_THR (209)
    // @Range: 0 100
    AP_GROUPINFO("FWD_MANTHR_MAX", 20, QuadPlane, fwd_thr_max, 0),

    // 21: TAILSIT_DSKLD
    // 22: TILT_FIX_ANGLE
    // 23: TILT_FIX_GAIN
    // 24: TAILSIT_RAT_FW
    // 25: TAILSIT_RAT_VT

    // @Group: TAILSIT_
    // @Path: tailsitter.cpp
    AP_SUBGROUPINFO(tailsitter, "TAILSIT_", 26, QuadPlane, Tailsitter),

    // @Group: TILT_
    // @Path: tiltrotor.cpp
    AP_SUBGROUPINFO(tiltrotor, "TILT_", 27, QuadPlane, Tiltrotor),

    // @Param: BACKTRANS_MS
    // @DisplayName: SLT and Tiltrotor back transition pitch limit duration
    // @Description: Pitch angle will increase from 0 to angle max over this duration when switching into VTOL flight in a position control mode. 0 Disables.
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("BACKTRANS_MS", 28, QuadPlane, back_trans_pitch_limit_ms, 3000),

    // @Param: TRANS_FAIL_ACT
    // @DisplayName: Quadplane transition failure action
    // @Description: This sets the mode that is changed to when Q_TRANS_FAIL time elapses, if set. See also Q_OPTIONS bit 19: CompleteTransition if Q_TRANS_FAIL
    // @Values: -1:Warn only, 0:QLand, 1:QRTL
    AP_GROUPINFO("TRANS_FAIL_ACT", 29, QuadPlane, transition_failure.action, 0),

    // @Group: WVANE_
    // @Path: ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp
    AP_SUBGROUPPTR(weathervane, "WVANE_", 30, QuadPlane, AC_WeatherVane),

    // @Param: LAND_ALTCHG
    // @DisplayName: Land detection altitude change threshold
    // @Description: The maximum altitude change allowed during land detection. You can raise this value if you find that landing detection takes a long time to complete. It is the maximum change in altitude over a period of 4 seconds for landing to be detected
    // @Units: m
    // @Range: 0.1 0.6
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("LAND_ALTCHG", 31, QuadPlane, landing_detect.detect_alt_change_m, 0.2),

    // @Param: NAVALT_MIN
    // @DisplayName: Minimum navigation altitude
    // @Description: This is the altitude in meters above which navigation begins in auto takeoff. Below this altitude the target roll and pitch will be zero. A value of zero disables the feature
    // @Units: m
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("NAVALT_MIN", 32, QuadPlane, takeoff_navalt_min_m, 0),

    // @Param: PLT_Y_RATE
    // @DisplayName: Pilot controlled yaw rate
    // @Description: Pilot controlled yaw rate max. Used in all pilot controlled modes except QAcro
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard

    // @Param: PLT_Y_EXPO
    // @DisplayName: Pilot controlled yaw expo
    // @Description: Pilot controlled yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced

    // @Param: PLT_Y_RATE_TC
    // @DisplayName: Pilot yaw rate control input time constant
    // @Description: Pilot yaw rate control input time constant. Low numbers lead to sharper response, higher numbers to softer response.
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_pilot, "PLT_Y_", 33, QuadPlane, AC_CommandModel),

    // @Param: RTL_ALT_MIN
    // @DisplayName: QRTL minimum altitude
    // @Description: If VTOL motors are active QRTL mode will VTOL climb to at least this altitude before returning home. If outside 150% the larger of WP_LOITER_RAD and RTL_RADIUS the vehicle will VTOL climb to Q_RTL_ALT. This parameter has no effect if the vehicle is in forward flight. Should be between Q_LAND_FINAL_ALT and Q_RTL_ALT
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT_MIN", 34, QuadPlane, qrtl_alt_min_m, 10),

    // @Param: FWD_THR_GAIN
    // @DisplayName: Q mode fwd throttle gain
    // @Description: This parameter sets the gain from forward accel/tilt to forward throttle in certain Q modes. The Q modes this feature operates in is controlled by the Q_FWD_THR_USE parameter. Vehicles using separate forward thrust motors, eg quadplanes, should set this parameter to (all up weight) / (maximum combined thrust of forward motors) with a value of 2 being typical. Vehicles that tilt lifting rotors to provide forward thrust should set this parameter to (all up weight) / (weight lifted by tilting rotors) which for most aircraft can be approximated as (total number of lifting rotors) / (number of lifting rotors that tilt). When using this method of forward throttle control, the forward tilt angle limit is controlled by the Q_FWD_PIT_LIM parameter.
    // @Range: 0.0 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FWD_THR_GAIN", 35, QuadPlane, q_fwd_thr_gain, 2.0f),

    // @Param: FWD_PIT_LIM
    // @DisplayName: Q mode forward pitch limit
    // @Description: When forward throttle is being controlled by the Q_FWD_THR_GAIN parameter in Q modes, the vehicle forward (nose down) pitch rotation will be limited to the value specified by this parameter and the any additional forward acceleration required will be produced by use of the forward thrust motor(s) or tilting of moveable rotors. Larger values allow the vehicle to pitch more nose down. Set initially to the amount of nose down pitch required to remove wing lift.
    // @Units: deg
    // @Range: 0.0 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FWD_PIT_LIM", 36, QuadPlane, q_fwd_pitch_lim, 3.0f),

    // @Param: FWD_THR_USE
    // @DisplayName: Q mode forward throttle use
    // @Description: This parameter determines when the feature that uses forward throttle instead of forward tilt is used. The amount of forward throttle is controlled by the Q_FWD_THR_GAIN parameter. The maximum amount of forward pitch allowed is controlled by the Q_FWD_PIT_LIM parameter. Q_FWD_THR_USE = 0 disables the feature. Q_FWD_THR_USE = 1 enables the feature in all position controlled modes such as QLOITER, QLAND, QRTL and VTOL TAKEOFF. Q_FWD_THR_USE = 2 enables the feature in all Q modes except QAUTOTUNE and QACRO. When enabling the feature, the legacy method of controlling forward throttle use via velocity controller error should be disabled by setting Q_VFWD_GAIN to 0. Do not use this feature with tailsitters.
    // @Values: 0:Off,1:On in all position controlled Q modes,2:On in all Q modes except QAUTOTUNE and QACRO
    // @User: Standard
    AP_GROUPINFO("FWD_THR_USE", 37, QuadPlane, q_fwd_thr_use, uint8_t(FwdThrUse::OFF)),

    // @Param: BCK_PIT_LIM
    // @DisplayName: Q mode rearward pitch limit
    // @Description: This sets the maximum number of degrees of back or pitch up in Q modes when the airspeed is at AIRSPEED_MIN, and is used to prevent excessive sutructural loads when pitching up decelerate. If airspeed is above or below AIRSPEED_MIN, the pitch up/back will be adjusted according to the formula pitch_limit = Q_BCK_PIT_LIM * (AIRSPEED_MIN / IAS)^2. The backwards/up pitch limit controlled by this parameter is in addition to limiting applied by PTCH_LIM_MAX_DEG and Q_ANGLE_MAX. The BCK_PIT_LIM limit is only applied when Q_FWD_THR_USE is set to 1 or 2 and the vehicle is flying in a mode that uses forward throttle instead of forward tilt to generate forward speed. Set to a non positive value 0 to deactivate this limit.
    // @Units: deg
    // @Range: 0.0 15.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BCK_PIT_LIM", 38, QuadPlane, q_bck_pitch_lim, 10.0f),

    // @Param: APPROACH_DIST
    // @DisplayName: Q mode approach distance
    // @Description: The minimum distance from the destination to use the fixed wing airbrake and approach code for landing approach. This is useful if you don't want the fixed wing approach logic to be used when you are close to the destination. Set to zero to always use fixed wing approach.
    // @Units: m
    // @Range: 0.0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("APPROACH_DIST", 39, QuadPlane, approach_distance_m, 0),
    
    AP_GROUPEND
};

/*
  defaults for all quadplanes
 */
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FLTD", 10.0 },
    { "Q_A_RAT_RLL_SMAX", 50.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FLTD", 10.0 },
    { "Q_A_RAT_PIT_SMAX", 50.0 },
    { "Q_A_RAT_YAW_SMAX", 50.0 },
    { "Q_A_RATE_R_MAX",   75.0 },
    { "Q_A_RATE_P_MAX",   75.0 },
    { "Q_A_RATE_Y_MAX",   75.0 },
    { "Q_M_SPOOL_TIME",   0.25 },
    { "Q_LOIT_ANG_MAX",   15.0 },
    { "Q_LOIT_ACC_MAX",   250.0 },
    { "Q_LOIT_BRK_ACCEL", 50.0 },
    { "Q_LOIT_BRK_JERK",  250 },
    { "Q_LOIT_SPEED",     500 },
    { "Q_WP_SPEED",       500 },
    { "Q_WP_ACCEL",       100 },
    { "Q_P_JERK_XY",      2   },
    // lower rotational accel limits
    { "Q_A_ACCEL_R_MAX", 40000 },
    { "Q_A_ACCEL_P_MAX", 40000 },
    { "Q_A_ACCEL_Y_MAX", 10000 },
};

/*
  conversion table for quadplane parameters
 */
const AP_Param::ConversionInfo q_conversion_table[] = {
    // tailsitter params have moved but retain the same names
    { Parameters::k_param_quadplane, 48,  AP_PARAM_INT8,  "Q_TAILSIT_ANGLE" },
    { Parameters::k_param_quadplane, 61,  AP_PARAM_INT8,  "Q_TAILSIT_ANG_VT" },
    { Parameters::k_param_quadplane, 50,  AP_PARAM_INT8,  "Q_TAILSIT_INPUT" },
    { Parameters::k_param_quadplane, 53,  AP_PARAM_FLOAT, "Q_TAILSIT_VFGAIN" },
    { Parameters::k_param_quadplane, 54,  AP_PARAM_FLOAT, "Q_TAILSIT_VHGAIN" },
    { Parameters::k_param_quadplane, 56,  AP_PARAM_FLOAT, "Q_TAILSIT_VHPOW" },
    { Parameters::k_param_quadplane, 251,   AP_PARAM_FLOAT, "Q_TAILSIT_GSCMAX" },
    { Parameters::k_param_quadplane, 379,   AP_PARAM_FLOAT, "Q_TAILSIT_RLL_MX" },
    { Parameters::k_param_quadplane, 635,   AP_PARAM_INT16, "Q_TAILSIT_MOTMX" },
    { Parameters::k_param_quadplane, 1147,  AP_PARAM_INT16, "Q_TAILSIT_GSCMSK" },
    { Parameters::k_param_quadplane, 1211,  AP_PARAM_FLOAT, "Q_TAILSIT_GSCMIN" },
    { Parameters::k_param_quadplane, 1403,  AP_PARAM_FLOAT, "Q_TAILSIT_DSKLD" },
    { Parameters::k_param_quadplane, 1595,  AP_PARAM_FLOAT, "Q_TAILSIT_RAT_FW" },
    { Parameters::k_param_quadplane, 1659,  AP_PARAM_FLOAT, "Q_TAILSIT_RAT_FW" },

    // tiltrotor params have moved but retain the same names
    { Parameters::k_param_quadplane, 37,  AP_PARAM_INT16,  "Q_TILT_MASK" },
    { Parameters::k_param_quadplane, 38,  AP_PARAM_INT16,  "Q_TILT_RATE_UP" },
    { Parameters::k_param_quadplane, 39,  AP_PARAM_INT8,  "Q_TILT_MAX" },
    { Parameters::k_param_quadplane, 47,  AP_PARAM_INT8,  "Q_TILT_TYPE" },
    { Parameters::k_param_quadplane, 49,  AP_PARAM_INT16,  "Q_TILT_RATE_DN" },
    { Parameters::k_param_quadplane, 55,  AP_PARAM_FLOAT,  "Q_TILT_YAW_ANGLE" },
    { Parameters::k_param_quadplane, 1467,  AP_PARAM_FLOAT,  "Q_TILT_FIX_ANGLE" },
    { Parameters::k_param_quadplane, 1531,  AP_PARAM_FLOAT,  "Q_TILT_FIX_GAIN" },

    // PARAMETER_CONVERSION - Added: Jan-2022
    { Parameters::k_param_quadplane, 33,  AP_PARAM_FLOAT, "Q_WVANE_GAIN" },     // Moved from quadplane to weathervane library
    { Parameters::k_param_quadplane, 34,  AP_PARAM_FLOAT, "Q_WVANE_ANG_MIN" },  // Q_WVANE_MINROLL moved from quadplane to weathervane library

    // PARAMETER_CONVERSION - Added: July-2022
    { Parameters::k_param_quadplane, 25,  AP_PARAM_FLOAT, "Q_PLT_Y_RATE" },   // Moved from quadplane to command model library
};

// PARAMETER_CONVERSION - Added: Oct-2021
const AP_Param::ConversionInfo mot_pwm_conversion_table[] = {
    { Parameters::k_param_quadplane, 22,  AP_PARAM_INT16, "Q_M_PWM_MIN" },
    { Parameters::k_param_quadplane, 23,  AP_PARAM_INT16, "Q_M_PWM_MAX" },
};

QuadPlane::QuadPlane(AP_AHRS &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);

    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Quadplane");
    }
    _singleton = this;
}


// setup default motors for the frame class
void QuadPlane::setup_default_channels(uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        SRV_Channels::set_aux_channel_default(SRV_Channels::get_motor_function(i), CH_5+i);
    }
}
    

bool QuadPlane::setup(void)
{
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }

    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav) + sizeof(*ahrs_view) + sizeof(*loiter_nav) + sizeof(*weathervane)) {
        AP_BoardConfig::config_error("Not enough memory for quadplane");
    }

    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    switch ((AP_Motors::motor_frame_class)frame_class) {
    case AP_Motors::MOTOR_FRAME_QUAD:
        setup_default_channels(4);
        break;
    case AP_Motors::MOTOR_FRAME_HEXA:
        setup_default_channels(6);
        break;
    case AP_Motors::MOTOR_FRAME_OCTA:
    case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        setup_default_channels(8);
        break;
    case AP_Motors::MOTOR_FRAME_Y6:
        setup_default_channels(7);
        break;
    case AP_Motors::MOTOR_FRAME_DECA:
        setup_default_channels(10);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        SRV_Channels::set_default_function(CH_5, SRV_Channel::k_motor1);
        SRV_Channels::set_default_function(CH_6, SRV_Channel::k_motor2);
        SRV_Channels::set_default_function(CH_8, SRV_Channel::k_motor4);
        SRV_Channels::set_default_function(CH_11, SRV_Channel::k_motor7);
        AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
    case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX:
    case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
        break;
    default:
        AP_BoardConfig::config_error("Unsupported Q_FRAME_CLASS %u", (unsigned int)(frame_class.get()));
    }

    // Make sure not both a tailsiter and tiltrotor
    if ((tailsitter.enable > 0) && (tiltrotor.enable > 0)) {
        AP_BoardConfig::config_error("set TAILSIT_ENABLE 0 or TILT_ENABLE 0");
    }

    switch ((AP_Motors::motor_frame_class)frame_class) {
#if AP_MOTORS_TRI_ENABLED
    case AP_Motors::MOTOR_FRAME_TRI:
        motors = NEW_NOTHROW AP_MotorsTri(rc_speed);
        motors_var_info = AP_MotorsTri::var_info;
        break;
#endif  // AP_MOTORS_TRI_ENABLED
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
        // this is a duo-motor tailsitter
        tailsitter.tailsitter_motors = NEW_NOTHROW AP_MotorsTailsitter(rc_speed);
        motors = tailsitter.tailsitter_motors;
        motors_var_info = AP_MotorsTailsitter::var_info;
        break;
    case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
#if AP_SCRIPTING_ENABLED
            motors = NEW_NOTHROW AP_MotorsMatrix_Scripting_Dynamic(plane.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
    default:
        motors = NEW_NOTHROW AP_MotorsMatrix(rc_speed);
        motors_var_info = AP_MotorsMatrix::var_info;
        break;
    }

    if (!motors) {
        AP_BoardConfig::allocation_error("motors");
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // create the attitude view used by the VTOL code
    ahrs_view = ahrs.create_view((tailsitter.enable > 0) ? ROTATION_PITCH_90 : ROTATION_NONE, ahrs_trim_pitch);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("ahrs_view");
    }

    attitude_control = NEW_NOTHROW AC_AttitudeControl_TS(*ahrs_view, aparm, *motors);
    if (!attitude_control) {
        AP_BoardConfig::allocation_error("attitude_control");
    }

    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = NEW_NOTHROW AC_PosControl(*ahrs_view, *motors, *attitude_control);
    if (!pos_control) {
        AP_BoardConfig::allocation_error("pos_control");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = NEW_NOTHROW AC_WPNav(*ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        AP_BoardConfig::allocation_error("wp_nav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = NEW_NOTHROW AC_Loiter(*ahrs_view, *pos_control, *attitude_control);
    if (!loiter_nav) {
        AP_BoardConfig::allocation_error("loiter_nav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    weathervane = NEW_NOTHROW AC_WeatherVane();
    if (!weathervane) {
        AP_BoardConfig::allocation_error("weathervane");
    }
    AP_Param::load_object_from_eeprom(weathervane, weathervane->var_info);

    motors->init(frame_class, frame_type);
    motors->update_throttle_range();
    motors->set_update_rate(rc_speed);
    attitude_control->parameter_sanity_check();

    // Try to convert mot PWM params, if still invalid force conversion
    AP_Param::convert_old_parameters(&mot_pwm_conversion_table[0], ARRAY_SIZE(mot_pwm_conversion_table));
    if (!motors->check_mot_pwm_params()) {
        AP_Param::convert_old_parameters(&mot_pwm_conversion_table[0], ARRAY_SIZE(mot_pwm_conversion_table), AP_Param::CONVERT_FLAG_FORCE);
    }

    // setup the trim of any motors used by AP_Motors so I/O board
    // failsafe will disable motors
    uint32_t mask = plane.quadplane.motors->get_motor_mask();
    hal.rcout->set_failsafe_pwm(mask, plane.quadplane.motors->get_pwm_output_min());

    // default QAssist state as set with Q_OPTIONS
    if (option_is_set(QuadPlane::Option::Q_ASSIST_FORCE_ENABLE)) {
        assist.set_state(VTOL_Assist::STATE::FORCE_ENABLED);
    }

    setup_defaults();

    AP_Param::convert_old_parameters(&q_conversion_table[0], ARRAY_SIZE(q_conversion_table));

    // centi-conversions added January 2024
    land_final_speed_ms.convert_centi_parameter(AP_PARAM_INT16);
    pilot_speed_z_max_up_ms.convert_centi_parameter(AP_PARAM_INT16);
    pilot_speed_z_max_dn_ms.convert_centi_parameter(AP_PARAM_INT16);
    pilot_accel_z_mss.convert_centi_parameter(AP_PARAM_INT16);

    // Provisionally assign the SLT thrust type.
    // It will be overwritten by tailsitter or tiltorotor setups.
    thrust_type = ThrustType::SLT;

    tailsitter.setup();

    tiltrotor.setup();

    if (!transition) {
        transition = NEW_NOTHROW SLT_Transition(*this, motors);
    }
    if (!transition) {
        AP_BoardConfig::allocation_error("transition");
    }

    // init wp_nav variables after defaults are setup
    wp_nav->wp_and_spline_init_m();

    transition->force_transition_complete();

    // param count will have changed
    AP_Param::invalidate_count();

    char frame_and_type_string[30];
    motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised, %s", frame_and_type_string);
    initialised = true;
    return true;
}

/*
  setup default parameters from defaults_table
 */
void QuadPlane::setup_defaults(void)
{
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));

    // reset ESC calibration
    if (esc_calibration != 0) {
        esc_calibration.set_and_save(0);
    }
    // Quadplanes need the same level of GPS error checking as Copters do, Plane is more relaxed
    AP_Param::set_default_by_name("EK2_CHECK_SCALE",100);
    AP_Param::set_default_by_name("EK3_CHECK_SCALE",100);

}

// run ESC calibration
void QuadPlane::run_esc_calibration(void)
{
    if (!motors->armed()) {
        motors->set_throttle_passthrough_for_esc_calibration(0);
        AP_Notify::flags.esc_calibration = false;
        return;
    }
    if (!AP_Notify::flags.esc_calibration) {
        gcs().send_text(MAV_SEVERITY_INFO, "Starting ESC calibration");
    }
    AP_Notify::flags.esc_calibration = true;
    switch (esc_calibration) {
    case 1:
        // throttle based calibration
        motors->set_throttle_passthrough_for_esc_calibration(plane.get_throttle_input() * 0.01f);
        break;
    case 2:
        // full range calibration
        motors->set_throttle_passthrough_for_esc_calibration(1);
        break;
    }
}

/*
  ask the multicopter attitude control to match the roll and pitch rates being demanded by the
  fixed wing controller if not in a pure VTOL mode
 */
void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    bool use_multicopter_control = in_vtol_mode() && !tailsitter.in_vtol_transition() && !force_fw_control_recovery;
    bool use_yaw_target = false;

    float yaw_target_cd = 0.0;
    if (!use_multicopter_control && transition->update_yaw_target(yaw_target_cd) &&
        !force_fw_control_recovery) {
        use_multicopter_control = true;
        use_yaw_target = true;
    }

    // normal control modes for VTOL and FW flight
    // tailsitter in transition to VTOL flight is not really in a VTOL mode yet
    if (use_multicopter_control) {

        // Pilot input, use yaw rate time constant
        set_pilot_yaw_rate_time_constant();

        // tailsitter-only body-frame roll control options
        // Angle mode attitude control for pitch and body-frame roll, rate control for euler yaw.
        if (tailsitter.enabled() &&
            (tailsitter.input_type & Tailsitter::input::TAILSITTER_INPUT_BF_ROLL)) {

            if (!(tailsitter.input_type & Tailsitter::input::TAILSITTER_INPUT_PLANE)) {
                // In multicopter input mode, the roll and yaw stick axes are independent of pitch
                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(false,
                                                                                plane.nav_roll_cd,
                                                                                plane.nav_pitch_cd,
                                                                                yaw_rate_cds);
                return;
            } else {
                // In plane input mode, the roll and yaw sticks are swapped
                // and their effective axes rotate from yaw to roll and vice versa
                // as pitch goes from zero to 90.
                // So it is necessary to also rotate their scaling.

                // Get the roll angle and yaw rate limits
                int16_t roll_limit = aparm.angle_max;
                // separate limit for tailsitter roll, if set
                if (tailsitter.max_roll_angle > 0) {
                    roll_limit = tailsitter.max_roll_angle * 100.0f;
                }
                // Prevent a divide by zero
                const float yaw_rate_max = command_model_pilot.get_rate();
                float yaw_rate_limit = ((yaw_rate_max < 1.0f) ? 1 : yaw_rate_max) * 100.0f;
                float yaw2roll_scale = roll_limit / yaw_rate_limit;

                // Rotate as a function of Euler pitch and swap roll/yaw
                float euler_pitch = radians(.01f * plane.nav_pitch_cd);
                float spitch = fabsf(sinf(euler_pitch));
                float y2r_scale = linear_interpolate(1, yaw2roll_scale, spitch, 0, 1);

                float p_yaw_rate = plane.nav_roll_cd / y2r_scale;
                float p_roll_angle = -y2r_scale * yaw_rate_cds;

                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(true,
                                                                                p_roll_angle,
                                                                                plane.nav_pitch_cd,
                                                                                p_yaw_rate);
                return;
            }
        }

        // note this is actually in deg/s for some SID_AXIS values for yaw
        Vector3f offset_deg;

#if AP_PLANE_SYSTEMID_ENABLED
        auto &systemid = plane.g2.systemid;
        systemid.update();
        offset_deg = systemid.get_attitude_offset_deg();
#endif

        if (use_yaw_target) {
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(plane.nav_roll_cd + offset_deg.x*100,
                                                               plane.nav_pitch_cd + offset_deg.y*100,
                                                               yaw_target_cd + offset_deg.z*100,
                                                               true);
        } else {
            // use euler angle attitude control
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd + offset_deg.x*100,
                                                                          plane.nav_pitch_cd + offset_deg.y*100,
                                                                          yaw_rate_cds + offset_deg.z*100);
        }
    } else {
        // use the fixed wing desired rates
        Vector3f bf_input_cd { plane.rollController.get_pid_info().target * 100.0f,
                               plane.pitchController.get_pid_info().target * 100.0f,
                               yaw_rate_cds };

        // rotate into multicopter attitude reference frame
        ahrs_view->rotate(bf_input_cd);

        // disable yaw time constant for 1:1 match of desired rates
        disable_yaw_rate_time_constant();

        attitude_control->input_rate_bf_roll_pitch_yaw_no_shaping_cds(bf_input_cd.x, bf_input_cd.y, bf_input_cd.z);
    }
}

// hold in stabilize with given throttle
void QuadPlane::hold_stabilize(float throttle_in)
{    
    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));

    if ((throttle_in <= 0) && !air_mode_active()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
    } else {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        bool should_boost = true;
        if (tailsitter.enabled() && assisted_flight) {
            // tailsitters in forward flight should not use angle boost
            should_boost = false;
        }
#if AP_PLANE_SYSTEMID_ENABLED
        throttle_in += plane.g2.systemid.get_throttle_offset();
#endif
        attitude_control->set_throttle_out(throttle_in, should_boost, 0);
    }
}

// run the multicopter Z controller
void QuadPlane::run_z_controller(void)
{
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED ) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (tailsitter.in_vtol_transition(now)) {
        // never run Z controller in tailsitter transition
        return;
    }
    if ((now - last_pidz_active_ms) > 20 || !pos_control->is_active_U()) {
        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_U_m(-get_pilot_velocity_z_max_dn_m(), pilot_speed_z_max_up_ms, pilot_accel_z_mss);

        // initialise the vertical position controller
        if (!tailsitter.enabled()) {
            pos_control->init_U_controller();
        } else {
            // initialise the vertical position controller with no descent
            pos_control->init_U_controller_no_descent();
        }
        last_pidz_init_ms = now;
    }
    last_pidz_active_ms = now;
    pos_control->update_U_controller();
}

void QuadPlane::relax_attitude_control()
{
    // disable roll and yaw control for vectored tailsitters
    // if not a vectored tailsitter completely disable attitude control
    attitude_control->relax_attitude_controllers(!tailsitter.relax_pitch());
}

/*
  check for an EKF yaw reset
 */
void QuadPlane::check_yaw_reset(void)
{
    if (!initialised) {
        return;
    }
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        LOGGER_WRITE_EVENT(LogEvent::EKF_YAW_RESET);
    }
}

void QuadPlane::set_climb_rate_ms(float target_climb_rate_ms)
{
    pos_control->input_vel_accel_U_m(target_climb_rate_ms, 0, false);
}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate_cms)
{
    // motors use full range
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_velocity_z_max_dn_m(), pilot_speed_z_max_up_ms, pilot_accel_z_mss);

    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));

    // call position controller
    set_climb_rate_ms(target_climb_rate_cms * 0.01);

    run_z_controller();
}

float QuadPlane::get_pilot_throttle()
{
    // get scaled throttle input
    float throttle_in = plane.channel_throttle->get_control_in();

    // normalize to [0,1]
    throttle_in /= plane.channel_throttle->get_range();

    if (is_positive(throttle_expo)) {
        // get hover throttle level [0,1]
        float thr_mid = motors->get_throttle_hover();
        float thrust_curve_expo = constrain_float(throttle_expo, 0.0f, 1.0f);

        // this puts mid stick at hover throttle
        return throttle_curve(thr_mid, thrust_curve_expo, throttle_in);
    } else {
        return throttle_in;
    }
}

/*
  get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle.
  The angle_max_cd and angle_limit_cd are mode dependent
*/
void QuadPlane::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // failsafe check
    if (plane.failsafe.rc_failsafe || plane.failsafe.throttle_counter > 0) {
        roll_out_cd = 0;
        pitch_out_cd = 0;
        return;
    }

    // fetch roll and pitch inputs
    roll_out_cd = plane.channel_roll->get_control_in();
    pitch_out_cd = plane.channel_pitch->get_control_in();

    // limit max lean angle, always allow for 10 degrees
    angle_limit_cd = constrain_float(angle_limit_cd, 1000.0f, angle_max_cd);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max_cd/4500.0;
    roll_out_cd *= scaler;
    pitch_out_cd *= scaler;

    // apply circular limit
    float total_in = norm(pitch_out_cd, roll_out_cd);
    if (total_in > angle_limit_cd) {
        float ratio = angle_limit_cd / total_in;
        roll_out_cd *= ratio;
        pitch_out_cd *= ratio;
    }

    // apply lateral tilt to euler roll conversion
    roll_out_cd = 100 * degrees(atanf(cosf(cd_to_rad(pitch_out_cd)) * tanf(cd_to_rad(roll_out_cd))));
}

/*
  get pilot throttle in for landing code. Return value on scale of 0 to 1
*/
float QuadPlane::get_pilot_land_throttle(void) const
{
    if (plane.rc_failsafe_active()) {
        // assume zero throttle if lost RC
        return 0;
    }
    // get scaled throttle input
    float throttle_in = plane.channel_throttle->get_control_in();

    // normalize to [0,1]
    throttle_in /= plane.channel_throttle->get_range();

    return constrain_float(throttle_in, 0, 1);
}

// helper for is_flying()
bool QuadPlane::is_flying(void)
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (motors->get_throttle() > 0.01f && !motors->limit.throttle_lower) {
        return true;
    }
    if (tailsitter.in_vtol_transition()) {
        return true;
    }
    return false;
}

// crude landing detector to prevent tipover
bool QuadPlane::should_relax(void)
{
    const uint32_t tnow = millis();

    bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
    if (motors->get_throttle() < 0.01f) {
        motor_at_lower_limit = true;
    }

    if (!motor_at_lower_limit) {
        landing_detect.lower_limit_start_ms = 0;
        landing_detect.land_start_ms = 0;
        return false;
    } else if (landing_detect.lower_limit_start_ms == 0) {
        landing_detect.lower_limit_start_ms = tnow;
    }

    return (tnow - landing_detect.lower_limit_start_ms) > 1000;
}

// see if we are flying in vtol
bool QuadPlane::is_flying_vtol(void) const
{
    if (!available()) {
        return false;
    }
    if (motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN) {
        // assume that with no motor outputs we're not flying in VTOL mode
        return false;
    }
    if (motors->get_throttle() > 0.01f) {
        // if we are demanding more than 1% throttle then don't consider aircraft landed
        return true;
    }
    if (plane.control_mode->is_vtol_man_throttle() && air_mode_active()) {
        // in manual throttle modes with airmode on, don't consider aircraft landed
        return true;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (plane.control_mode->is_vtol_man_mode()) {
        // in manual flight modes only consider aircraft landed when pilot demanded throttle is zero
        return is_positive(get_throttle_input());
    }
    if (in_vtol_mode() && millis() - landing_detect.lower_limit_start_ms > 5000) {
        // use landing detector
        return true;
    }
    return false;
}

/*
  smooth out descent rate for landing to prevent a jerk as we get to
  land_final_alt_m. 
 */
float QuadPlane::landing_descent_rate_ms(float height_above_ground_m)
{
    if (poscontrol.last_override_descent_ms != 0) {
        const uint32_t now = AP_HAL::millis();
        if (now - poscontrol.last_override_descent_ms < 1000) {
            return poscontrol.override_descent_rate_ms;
        }
    }

    if (poscontrol.get_state() == QPOS_LAND_FINAL) {
        // when in final use descent rate for final even if alt has climbed again
        height_above_ground_m = MIN(height_above_ground_m, land_final_alt_m);
    }
    const float max_climb_speed_ms = wp_nav->get_default_speed_up_ms();
    float ret_ms = linear_interpolate(land_final_speed_ms, wp_nav->get_default_speed_down_ms(),
                                   height_above_ground_m,
                                   land_final_alt_m, land_final_alt_m + 6);

    if (option_is_set(QuadPlane::Option::THR_LANDING_CONTROL)) {
        // allow throttle control for landing speed
        const float thr_in = get_pilot_land_throttle();
        if (thr_in > THR_CTRL_LAND_THRESH) {
            thr_ctrl_land = true;
        }
        if (thr_ctrl_land) {
            const float dz = 0.1;
            const float thresh1 = 0.5 + dz;
            const float thresh2 = 0.5 - dz;
            const float scaling = 1.0 / (0.5 - dz);
            if (thr_in > thresh1) {
                // start climbing
                ret_ms = -(thr_in - thresh1) * scaling * max_climb_speed_ms;
            } else if (thr_in > thresh2) {
                // hold height
                ret_ms = 0;
            } else {
                ret_ms *= (thresh2 - thr_in) * scaling;
            }
        }    
    }

    if (poscontrol.pilot_correction_active) {
        // stop descent when repositioning
        ret_ms = MIN(0, ret_ms);
    }

    return ret_ms;
}

/*
  get pilot input yaw rate in cd/s
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void) const
{
    const auto rudder_in = plane.channel_rudder->get_control_in();
    bool manual_air_mode = plane.control_mode->is_vtol_man_throttle() && air_mode_active();
    if (!manual_air_mode &&
        !is_positive(get_throttle_input()) &&
        (!plane.control_mode->does_auto_throttle() || motors->limit.throttle_lower) &&
        plane.arming.get_rudder_arming_type() == AP_Arming::RudderArming::ARMDISARM &&
        rudder_in < 0 &&
        fabsf(inertial_nav.get_velocity_z_up_cms()) < (0.5 * get_pilot_velocity_z_max_dn_m()) * 100) {
        // the user may be trying to disarm, disable pilot yaw control
        return 0;
    }

    if ((plane.g.stick_mixing == StickMixing::NONE) &&
        (plane.control_mode == &plane.mode_qrtl ||
         plane.control_mode->is_guided_mode() ||
         in_vtol_auto())) {
        return 0;
    }

    // add in rudder input
    const float yaw_rate_max = command_model_pilot.get_rate();
    float max_rate = yaw_rate_max;
    if (!in_vtol_mode() && tailsitter.enabled()) {
        // scale by RUDD_DT_GAIN when not in a VTOL mode for
        // tailsitters. This allows for flat turns in tailsitters for
        // fixed wing modes if you want them, but prevents crazy yaw
        // rate demands in fixed wing based on your preferred yaw rate
        // when hovering
        max_rate *= plane.g2.rudd_dt_gain * 0.01;
    }
    if (tailsitter.enabled() &&
        tailsitter.input_type & Tailsitter::input::TAILSITTER_INPUT_BF_ROLL) {
        // must have a non-zero max yaw rate for scaling to work
        max_rate = (yaw_rate_max < 1.0f) ? 1 : yaw_rate_max;
    }
    return input_expo(rudder_in * (1/4500.0), command_model_pilot.get_expo()) * max_rate * 100.0;
}

/*
  get overall desired yaw rate in cd/s
 */
float QuadPlane::get_desired_yaw_rate_cds(bool should_weathervane)
{
    float yaw_cds = 0;
    if (assisted_flight) {
        // use bank angle to get desired yaw rate
        yaw_cds += desired_auto_yaw_rate_cds();
    }

    // add in pilot input
    yaw_cds += get_pilot_input_yaw_rate_cds();

    if (should_weathervane) {
        // add in weathervaning
        yaw_cds += get_weathervane_yaw_rate_cds();
    }
    
    return yaw_cds;
}

// get pilot desired climb rate in cm/s
float QuadPlane::get_pilot_desired_climb_rate_cms(void) const
{
    if (!rc().has_valid_input()) {
        // no valid input means no sensible pilot desired climb rate.
        // descend at 0.5m/s for now
        return -50;
    }
    uint16_t dead_zone = plane.channel_throttle->get_dead_zone();
    uint16_t trim = (plane.channel_throttle->get_radio_max() + plane.channel_throttle->get_radio_min()) / 2;
    const float throttle_request = plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) * 0.01f;
    return throttle_request * (throttle_request > 0.0f ? pilot_speed_z_max_up_ms : get_pilot_velocity_z_max_dn_m()) * 100;
}


/*
  initialise throttle_wait based on throttle and is_flying()
 */
void QuadPlane::init_throttle_wait(void)
{
    if (get_throttle_input() >= 10 ||
        plane.is_flying()) {
        throttle_wait = false;
    } else {
        throttle_wait = true;        
    }
}
    
// set motor arming
void QuadPlane::set_armed(bool armed)
{
    if (!initialised) {
        return;
    }
    motors->armed(armed);

    if (plane.control_mode == &plane.mode_guided) {
        guided_wait_takeoff = armed;
    }

    // re-init throttle wait on arm and disarm, to prevent rudder
    // arming on 2nd flight causing yaw
    if (!air_mode_active()) {
        init_throttle_wait();
    }
}


/*
  estimate desired climb rate for assistance (in cm/s)
 */
float QuadPlane::assist_climb_rate_cms(void) const
{
    float climb_rate_cms;
    if (plane.control_mode->does_auto_throttle()) {
        // use altitude_error_cm, spread over 10s interval
        climb_rate_cms = plane.calc_altitude_error_cm() * 0.1f;
    } else {
        // otherwise estimate from pilot input
        climb_rate_cms = plane.g.flybywire_climb_rate * (plane.nav_pitch_cd / (plane.aparm.pitch_limit_max * 100));
        climb_rate_cms *= plane.get_throttle_input();
    }
    climb_rate_cms = constrain_float(climb_rate_cms, -wp_nav->get_default_speed_down_ms() * 100.0, wp_nav->get_default_speed_up_ms() * 100.0);

    // bring in the demanded climb rate over 2 seconds
    const uint32_t ramp_up_time_ms = 2000;
    const uint32_t dt_since_start = last_pidz_active_ms - last_pidz_init_ms;
    if (dt_since_start < ramp_up_time_ms) {
        climb_rate_cms = linear_interpolate(0, climb_rate_cms, dt_since_start, 0, ramp_up_time_ms);
    }
    
    return climb_rate_cms;
}

/*
  calculate desired yaw rate for assistance
 */
float QuadPlane::desired_auto_yaw_rate_cds(bool body_frame) const
{
    float aspeed;
    if (!ahrs.airspeed_estimate(aspeed) || aspeed < plane.aparm.airspeed_min) {
        aspeed = plane.aparm.airspeed_min;
    }
    if (aspeed < 1) {
        aspeed = 1;
    }
    if (body_frame) {
        return degrees(GRAVITY_MSS * sinf(cd_to_rad(plane.nav_roll_cd))/aspeed) * 100;
    }
    return degrees(GRAVITY_MSS * tanf(cd_to_rad(plane.nav_roll_cd))/aspeed) * 100;
}

/*
  update for transition from quadplane to fixed wing mode
 */
void SLT_Transition::update()
{
    const uint32_t now = millis();
    
    if (!plane.arming.is_armed_and_safety_off()) {
        // reset the failure timer if we are disarmed
        transition_start_ms = now;
    }

    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);

    /*
      see if we should provide some assistance
     */
    if (quadplane.assist.should_assist(aspeed, have_airspeed)) {
        // the quad should provide some assistance to the plane
        quadplane.assisted_flight = true;
        // update transition state for vehicles using airspeed wait
        if (!in_forced_transition) {
            const bool show_message = transition_state != State::AIRSPEED_WAIT || transition_start_ms == 0;
            if (show_message) {
                gcs().send_text(MAV_SEVERITY_INFO, "Transition started airspeed %.1f", (double)aspeed);
            }
            transition_state = State::AIRSPEED_WAIT;
            if (transition_start_ms == 0) {
                transition_start_ms = now;
            }
        }
    } else {
        quadplane.assisted_flight = false;
    }


    // if rotors are fully forward then we are not transitioning,
    // unless we are waiting for airspeed to increase (in which case
    // the tilt will decrease rapidly)
    if (quadplane.tiltrotor.fully_fwd() && transition_state != State::AIRSPEED_WAIT) {
        if (transition_state == State::TIMER) {
            float throttle;
            if (plane.quadplane.tiltrotor.get_forward_throttle(throttle)) {
                // Reset the TECS minimum throttle to match throttle of forward thrust motors
                // and set the throttle channel slew rate limiter to prevent a sudden drop in throttle
                plane.TECS_controller.set_throttle_min(throttle, true);
                SRV_Channels::set_slew_last_scaled_output(SRV_Channel::k_throttle, throttle * 100);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100);
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        }
        transition_state = State::DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
    }

    if (transition_state != State::DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }
    
    switch (transition_state) {
    case State::AIRSPEED_WAIT: {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = now;
        }

        // check if we have failed to transition while in State::AIRSPEED_WAIT
        if (transition_start_ms != 0 &&
        (quadplane.transition_failure.timeout > 0) &&
        ((now - transition_start_ms) > ((uint32_t)quadplane.transition_failure.timeout * 1000))) {
            if (!quadplane.transition_failure.warned) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Transition failed, exceeded time limit");
                quadplane.transition_failure.warned = true;
            }
            // if option is set and ground speed> 1/2 AIRSPEED_MIN for non-tiltrotors, then complete transition, otherwise QLAND.
            // tiltrotors will immediately transition
            const bool tiltrotor_with_ground_speed = quadplane.tiltrotor.enabled() && (plane.ahrs.groundspeed() > plane.aparm.airspeed_min * 0.5);
            if (quadplane.option_is_set(QuadPlane::Option::TRANS_FAIL_TO_FW) && tiltrotor_with_ground_speed) {
                transition_state = State::TIMER;
                in_forced_transition = true;
            } else {
                switch (QuadPlane::TRANS_FAIL::ACTION(quadplane.transition_failure.action)) {
                    case QuadPlane::TRANS_FAIL::ACTION::QLAND:
                        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TRANSITION);
                        break;

                    case QuadPlane::TRANS_FAIL::ACTION::QRTL:
                        plane.set_mode(plane.mode_qrtl, ModeReason::VTOL_FAILED_TRANSITION);
                        quadplane.poscontrol.set_state(QuadPlane::QPOS_POSITION1);
                        break;

                    default:
                        break;
                }
            }
        } else {
            quadplane.transition_failure.warned = false;
        }

        transition_low_airspeed_ms = now;
        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !quadplane.assisted_flight) {
            transition_state = State::TIMER;
            airspeed_reached_tilt = quadplane.tiltrotor.current_tilt;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
        }
        quadplane.assisted_flight = true;

        // do not allow a climb on the quad motors during transition a
        // climb would add load to the airframe, and prolongs the
        // transition. We don't limit the climb rate on tilt rotors as
        // otherwise the plane can end up in high-alpha flight with
        // low VTOL thrust and may not complete a transition
        float climb_rate_cms = quadplane.assist_climb_rate_cms();
        if (quadplane.option_is_set(QuadPlane::Option::LEVEL_TRANSITION) && !quadplane.tiltrotor.enabled()) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
        }
        quadplane.hold_hover(climb_rate_cms);

        if (!quadplane.tiltrotor.is_vectored()) {
            // set desired yaw rate to a coordinated turn
            quadplane.attitude_control->reset_yaw_target_and_rate();
            quadplane.attitude_control->rate_bf_yaw_target(quadplane.desired_auto_yaw_rate_cds(true));
        }
        if (quadplane.tiltrotor.enabled() && !quadplane.tiltrotor.has_fw_motor()) {
            // tilt rotors without dedicated fw motors do not have forward throttle output in this stage
            // prevent throttle I wind up
            plane.TECS_controller.reset_throttle_I();
        }

        last_throttle = motors->get_throttle();

        // reset integrators while we are below target airspeed as we
        // may build up too much while still primarily under
        // multicopter control
        plane.pitchController.reset_I();
        plane.rollController.reset_I();

        // give full authority to attitude control
        quadplane.attitude_control->set_throttle_mix_max(1.0f);
        break;
    }

    case State::TIMER: {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // after airspeed is reached we degrade throttle over the transition time, but continue
        // to stabilize and wait for any required forward tilt to complete and the timer to expire
        const uint32_t transition_timer_ms = now - transition_low_airspeed_ms;
        const float trans_time_ms = constrain_float(quadplane.transition_time_ms, 500, 30000);
        const bool tilt_fwd_complete = !quadplane.tiltrotor.enabled() || quadplane.tiltrotor.tilt_angle_achieved();
        if (transition_timer_ms > unsigned(trans_time_ms) && tilt_fwd_complete) {
            transition_state = State::DONE;
            in_forced_transition = false;
            transition_start_ms = 0;
            transition_low_airspeed_ms = 0;
            float throttle;
            if (plane.quadplane.tiltrotor.get_forward_throttle(throttle)) {
                // Reset the TECS minimum throttle to match throttle of forward thrust motors
                // and set the throttle channel slew rate limiter to prevent a sudden drop in throttle
                plane.TECS_controller.set_throttle_min(throttle, true);
                SRV_Channels::set_slew_last_scaled_output(SRV_Channel::k_throttle, throttle * 100);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100);
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Transition done");
        }

        float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
        float throttle_scaled = last_throttle * transition_scale;

        // set zero throttle mix, to give full authority to
        // throttle. This ensures that the fixed wing controllers get
        // a chance to learn the right integrators during the transition
        quadplane.attitude_control->set_throttle_mix_value(0.5 * transition_scale);

        if (throttle_scaled < 0.01) {
            // ensure we don't drop all the way to zero or the motors
            // will stop stabilizing
            throttle_scaled = 0.01;
        }
        if (quadplane.tiltrotor.enabled() && !quadplane.tiltrotor.has_vtol_motor() && !quadplane.tiltrotor.has_fw_motor()) {
            // All motors tilting, Use a combination of vertical and forward throttle based on current tilt angle
            // scale from all VTOL throttle at airspeed_reached_tilt to all forward throttle at fully forward tilt
            // this removes a step change in throttle once assistance is stopped
            const float ratio = (constrain_float(quadplane.tiltrotor.current_tilt, airspeed_reached_tilt, quadplane.tiltrotor.get_fully_forward_tilt()) - airspeed_reached_tilt) / (quadplane.tiltrotor.get_fully_forward_tilt() - airspeed_reached_tilt);
            const float fw_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),0) * 0.01;
            throttle_scaled = constrain_float(throttle_scaled * (1.0-ratio) + fw_throttle * ratio, 0.0, 1.0);
        }
        quadplane.assisted_flight = true;
        quadplane.hold_stabilize(throttle_scaled);

        if (!quadplane.tiltrotor.is_vectored()) {
            // set desired yaw rate to a coordinated turn
            quadplane.attitude_control->reset_yaw_target_and_rate();
            quadplane.attitude_control->rate_bf_yaw_target(quadplane.desired_auto_yaw_rate_cds(true));
        }
        break;
    }

    case State::DONE:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        set_last_fw_pitch();
        in_forced_transition = false;
        return;
    }

    quadplane.motors_output();

    set_last_fw_pitch();
}

void SLT_Transition::VTOL_update()
{
    /*
      setup the transition state appropriately for next time we go into a non-VTOL mode
    */
    transition_start_ms = 0;
    transition_low_airspeed_ms = 0;
    if (quadplane.throttle_wait && !plane.is_flying()) {
        in_forced_transition = false;
        transition_state = State::DONE;
    } else {
        /*
          setup for airspeed wait for later
        */
        transition_state = State::AIRSPEED_WAIT;
    }
    last_throttle = motors->get_throttle();

    // Keep assistance reset while not checking
    quadplane.assist.reset();
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
    if (!setup()) {
        return;
    }

    // keep motors interlock state upto date with E-stop
    motors->set_interlock(!SRV_Channels::get_emergency_stop());

    if ((ahrs_view != NULL) && !is_equal(_last_ahrs_trim_pitch, ahrs_trim_pitch.get())) {
        _last_ahrs_trim_pitch = ahrs_trim_pitch.get();
        ahrs_view->set_pitch_trim(_last_ahrs_trim_pitch);
    }

#if AP_ADVANCEDFAILSAFE_ENABLED
    if (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
#endif
    
    if (motor_test.running) {
        motor_test_output();
        return;
    }

    if (SRV_Channels::get_emergency_stop()) {
        attitude_control->reset_rate_controller_I_terms();
    }

    if (!plane.arming.is_armed_and_safety_off()) {
        /*
          make sure we don't have any residual control from previous flight stages
         */
        if (tailsitter.enabled()) {
            // tailsitters only relax I terms, to make ground testing easier
            attitude_control->reset_rate_controller_I_terms();
        } else {
            // otherwise full relax
            attitude_control->relax_attitude_controllers();
        }
        // todo: do you want to set the throttle at this point?
        pos_control->relax_U_controller(0);
    }

    const uint32_t now = AP_HAL::millis();
    if (!in_vtol_mode() && !in_vtol_airbrake()) {
        // we're in a fixed wing mode, cope with transitions and check
        // for assistance needed
        if (plane.control_mode == &plane.mode_manual ||
            plane.control_mode == &plane.mode_acro ||
            plane.control_mode == &plane.mode_training) {
            // in manual modes quad motors are always off
            if (!tailsitter.enabled()) {
                set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
                motors->output();
            }
            transition->force_transition_complete();
            assisted_flight = false;
        } else {
            transition->update();
        }

    } else {

        assisted_flight = in_vtol_airbrake();

        // output to motors
        motors_output();

        transition->VTOL_update();

    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait &&
        (plane.get_throttle_input() > 10 ||
         !rc().has_valid_input())) {
        throttle_wait = false;
    }

    tiltrotor.update();

    if (in_vtol_mode()) {
        // if enabled output forward throttle else 0
        float fwd_thr = 0;
        if (allow_forward_throttle_in_vtol_mode()) {
            fwd_thr = forward_throttle_pct();
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, fwd_thr);
    }

#if HAL_LOGGING_ENABLED
    // motors logging
    if (motors->armed()) {
        const bool motors_active = in_vtol_mode() || assisted_flight;
        if (motors_active && (motors->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN)) {
            // log ANG at main loop rate
            bool sysid_running = false;
#if AP_PLANE_SYSTEMID_ENABLED
            sysid_running = plane.g2.systemid.is_running();
#endif
            if (!sysid_running) {
                if (show_vtol_view()) {
                    attitude_control->Write_ANG();
                }
                // log RATE at main loop rate
                attitude_control->Write_Rate(*pos_control);
            }

            // log MOTB at 10 Hz
            if (now - last_motb_log_ms > 100) {
                last_motb_log_ms = now;
                motors->Log_Write();
            }
        }
        // log QTUN at 25 Hz if motors are active, or have been active in the last quarter second
        if ((motors_active || (now - last_motors_active_ms < 250)) && (now - last_qtun_log_ms > 40)) {
            last_qtun_log_ms = now;
            Log_Write_QControl_Tuning();
        }
    }
#else
    (void)now;
#endif  // HAL_LOGGING_ENABLED
}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DesiredSpoolState::SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
void QuadPlane::update_throttle_suppression(void)
{
    // if the motors have been running in the last 2 seconds then
    // allow them to run now
    if (AP_HAL::millis() - last_motors_active_ms < 2000) {
        return;
    }

    // see if motors are already disabled
    if (motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        return;
    }

    if (guided_wait_takeoff) {
        goto idle_state;
    }

    /* if the users throttle is above zero then allow motors to run

       if the user has unset the "check throttle zero when arming"
       then the RC controller has a sprung throttle and we should not
       consider non-zero throttle to mean that pilot is commanding
       takeoff unless in a manual throttle mode
    */
    if (!is_zero(get_throttle_input()) &&
        (rc().arming_check_throttle() ||
         plane.control_mode->is_vtol_man_throttle() ||
         plane.channel_throttle->norm_input_dz() > 0)) {
        return;
    }

    // if in a VTOL manual throttle mode and air_mode is on then allow motors to run
    if (plane.control_mode->is_vtol_man_throttle() && air_mode_active()) {
        return;
    }

    // if we are in a fixed wing auto throttle mode and we have
    // unsuppressed the throttle then allow motors to run
    if (plane.control_mode->does_auto_throttle() && !plane.throttle_suppressed) {
        return;
    }

    // if our vertical velocity is greater than 1m/s then allow motors to run
    if (fabsf(inertial_nav.get_velocity_z_up_cms()) > 100) {
        return;
    }

    // if we are more than 5m from home altitude then allow motors to run
    if (plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING) > 5) {
        return;
    }

    // allow for takeoff
    if (plane.control_mode == &plane.mode_auto && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
        return;
    }

idle_state:
    // motors should be in the spin when armed state to warn user they could become active
    set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    motors->set_throttle(0);
    last_motors_active_ms = 0;
}

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void QuadPlane::update_throttle_hover()
{
    if (!available()) {
        return;
    }
    
    // if not armed or landed exit
    if (!motors->armed() || !is_flying_vtol()) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_NEU_ms().z)) {
        return;
    }

    // do not update if quadplane forward motor is running (wing may be generating lift)
    // we use the THR_MIN value to account for petrol motors idling at THR_MIN
    if (!tailsitter.enabled() && (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > MAX(0, plane.aparm.throttle_min + 10))) {
        return;
    }

    // don't update if Z controller not running
    const uint32_t now = AP_HAL::millis();
    if (now - last_pidz_active_ms > 20) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    float aspeed;
    // calc average throttle if we are in a level hover and low airspeed
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        labs(ahrs_view->roll_sensor) < 500 && labs(ahrs_view->pitch_sensor) < 500 &&
        ahrs.airspeed_estimate(aspeed) && aspeed < plane.aparm.airspeed_min * 0.3) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        plane.gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}
/*
  output motors and do any copter needed
 */
void QuadPlane::motors_output(bool run_rate_controller)
{
    /* Delay for ARMING_DELAY_MS after arming before allowing props to spin:
       1) for safety (OPTION_DELAY_ARMING)
       2) to allow motors to return to vertical (OPTION_DISARMED_TILT)
     */
    if (option_is_set(QuadPlane::Option::DISARMED_TILT) || option_is_set(QuadPlane::Option::DELAY_ARMING)) {
        if (plane.arming.get_delay_arming()) {
            // delay motor start after arming
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            return;
        }
    }

#if AP_ADVANCEDFAILSAFE_ENABLED
    if (!plane.arming.is_armed_and_safety_off() ||
        (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) ||
         SRV_Channels::get_emergency_stop()) {
#else
    if (!plane.arming.is_armed_and_safety_off() || SRV_Channels::get_emergency_stop()) {
#endif
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
    if (esc_calibration && AP_Notify::flags.esc_calibration && plane.control_mode == &plane.mode_qstabilize) {
        // output is direct from run_esc_calibration()
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (tailsitter.in_vtol_transition(now) && !assisted_flight) {
        /*
          don't run the motor outputs while in tailsitter->vtol
          transition. That is taken care of by the fixed wing
          stabilisation code
         */
        return;
    }

    if (run_rate_controller) {
        if (now - last_att_control_ms > 100) {
            // relax if have been inactive
            relax_attitude_control();
        }

        // see if we need to be in VTOL recovery
        assist.check_VTOL_recovery();

        // run low level rate controllers that only require IMU data and set loop time
        const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
        motors->set_dt_s(last_loop_time_s);
        attitude_control->set_dt_s(last_loop_time_s);
        pos_control->set_dt_s(last_loop_time_s);
        attitude_control->rate_controller_run();
        // reset sysid and other temporary inputs
        attitude_control->rate_controller_target_reset();
        last_att_control_ms = now;
    }

    // see if motors should be shut down
    update_throttle_suppression();

    motors->output();

    // remember when motors were last active for throttle suppression
    if (motors->get_throttle() > 0.01f || tiltrotor.motors_active()) {
        last_motors_active_ms = now;
    }

}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state) const
{
    if (!available()) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL not available");
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL transition only in AUTO");
        return false;
    }
    switch (state) {
    case MAV_VTOL_STATE_MC:
        if (!plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Entered VTOL mode");
        }
        plane.auto_state.vtol_mode = true;
        // This is a precaution. It should be looked after by the call to QuadPlane::mode_enter(void) on mode entry.
        plane.quadplane.q_fwd_throttle = 0.0f;
        plane.quadplane.q_fwd_pitch_lim_cd = 100.0f * plane.quadplane.q_fwd_pitch_lim;
        return true;
        
    case MAV_VTOL_STATE_FW:
        if (plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Exited VTOL mode");
        }
        plane.auto_state.vtol_mode = false;

        return true;

    default:
        break;
    }

    gcs().send_text(MAV_SEVERITY_NOTICE, "Invalid VTOL mode");
    return false;
}

/*
  are we in a VTOL auto state?
 */
bool QuadPlane::in_vtol_auto(void) const
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
    if (plane.auto_state.vtol_mode) {
        return true;
    }
    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:
        return plane.auto_state.vtol_loiter;
    case MAV_CMD_NAV_TAKEOFF:
        return is_vtol_takeoff(id);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return is_vtol_land(id);
    default:
        return false;
    }
}

/*
  are we in a VTOL mode? This is used to decide if we run the
  transition handling code or not

  note that AIRBRAKE is not considered in_vtol_mode even though the
  VTOL motors are running
 */
bool QuadPlane::in_vtol_mode(void) const
{
    if (!available()) {
        return false;
    }
    if (in_vtol_land_sequence()) {
        return poscontrol.get_state() != QPOS_APPROACH && poscontrol.get_state() != QPOS_AIRBRAKE;
    }
    if (plane.control_mode->is_vtol_mode()) {
        return true;
    }
    if (plane.control_mode->is_guided_mode()
        && plane.auto_state.vtol_loiter &&
        poscontrol.get_state() > QPOS_APPROACH) {
        return true;
    }
    if (plane.control_mode == &plane.mode_guided &&
        guided_takeoff) {
        return true;
    }
    if (in_vtol_auto()) {
        if (!plane.auto_state.vtol_loiter || poscontrol.get_state() > QPOS_AIRBRAKE) {
            return true;
        }
    }
    return false;
}

/*
  are we in a VTOL mode that needs position and velocity estimates?
 */
bool QuadPlane::in_vtol_posvel_mode(void) const
{
    if (!available()) {
        return false;
    }
    return (plane.control_mode == &plane.mode_qloiter ||
            plane.control_mode == &plane.mode_qland ||
            plane.control_mode == &plane.mode_qrtl ||
#if QAUTOTUNE_ENABLED
            plane.control_mode == &plane.mode_qautotune ||
#endif
            (plane.control_mode->is_guided_mode() &&
            plane.auto_state.vtol_loiter &&
             poscontrol.get_state() > QPOS_APPROACH) ||
            in_vtol_auto());
}

/*
  update landing positioning offset
 */
void QuadPlane::update_land_positioning(void)
{
    if (!option_is_set(QuadPlane::Option::REPOSITION_LANDING)) {
        // not enabled
        poscontrol.pilot_correction_active = false;
        poscontrol.target_vel_ms.zero();
        return;
    }
    const float scale = 1.0 / 4500;
    float roll_in = plane.channel_roll->get_control_in() * scale;
    float pitch_in = plane.channel_pitch->get_control_in() * scale;

    // limit correction speed to accel with stopping time constant of 0.5s
    const float speed_max_ms = wp_nav->get_wp_acceleration_mss() * 0.5;
    const float dt = plane.scheduler.get_loop_period_s();

    poscontrol.target_vel_ms = Vector3f(-pitch_in, roll_in, 0) * speed_max_ms;
    poscontrol.target_vel_ms.rotate_xy(ahrs_view->yaw);

    // integrate our corrected position
    poscontrol.correction_ne_m += poscontrol.target_vel_ms.xy() * dt;

    poscontrol.pilot_correction_active = (!is_zero(roll_in) || !is_zero(pitch_in));
    if (poscontrol.pilot_correction_active) {
        poscontrol.pilot_correction_done = true;
    }
}

/*
  run (and possibly init) xy controller
 */
void QuadPlane::run_xy_controller(float accel_limit_mss)
{
    float accel_mss = wp_nav->get_wp_acceleration_mss();
    if (is_positive(accel_limit_mss)) {
        // allow for accel limit override
        accel_mss = MAX(accel_mss, accel_limit_mss);
    }
    const float speed_ms = wp_nav->get_default_speed_NE_ms();
    pos_control->set_max_speed_accel_NE_m(speed_ms, accel_mss);
    pos_control->set_correction_speed_accel_NE_m(speed_ms, accel_mss);
    if (!pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }
    pos_control->set_lean_angle_max_cd(MIN(4500, MAX(accel_mss_to_angle_deg(accel_limit_mss) * 100, aparm.angle_max)));
    if (q_fwd_throttle > 0.95f) {
        // prevent wind up of the velocity controller I term due to a saturated forward throttle
        pos_control->set_externally_limited_NE();
    }
    pos_control->update_NE_controller();
}

/*
  initialise QPOS_APPROACH
 */
void QuadPlane::poscontrol_init_approach(void)
{
    const float dist = plane.current_loc.get_distance(plane.next_WP_loc);
    if (option_is_set(QuadPlane::Option::DISABLE_APPROACH) ||
        (is_positive(approach_distance_m) && dist < approach_distance_m)) {
        // go straight to QPOS_POSITION1
        poscontrol.set_state(QPOS_POSITION1);
        gcs().send_text(MAV_SEVERITY_INFO,"VTOL Position1 d=%.1f", dist);
    } else if (poscontrol.get_state() != QPOS_APPROACH) {
        // check if we are close to the destination. We don't want to
        // do a full approach when very close
        if (dist < transition_threshold_m()) {
            if (tailsitter.enabled() || motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL Position1 d=%.1f", dist);
                poscontrol.set_state(QPOS_POSITION1);
                transition->set_last_fw_pitch();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL airbrake v=%.1f d=%.0f sd=%.0f h=%.1f",
                                plane.ahrs.groundspeed(),
                                dist,
                                stopping_distance_m(),
                                plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
                poscontrol.set_state(QPOS_AIRBRAKE);
            }
        } else {
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL approach d=%.1f", dist);
            poscontrol.set_state(QPOS_APPROACH);
        }
        poscontrol.thrust_loss_start_ms = 0;
    }
    poscontrol.pilot_correction_done = false;
    poscontrol.correction_ne_m.zero();
    poscontrol.slow_descent = false;
}

#if HAL_LOGGING_ENABLED
/*
  log the QPOS message
 */
void QuadPlane::log_QPOS(void)
{
// @LoggerMessage: QPOS
// @Description: Quadplane position data
// @Field: TimeUS: Time since system startup
// @Field: State: Position control state
// @FieldValueEnum: State: QuadPlane::position_control_state
// @Field: Dist: Distance to next waypoint
// @Field: TSpd: Target speed
// @Field: TAcc: Target acceleration
// @Field: OShoot: True if landing point is overshot or heading off by more than 60 degrees

    AP::logger().WriteStreaming("QPOS", "TimeUS,State,Dist,TSpd,TAcc,OShoot", "QBfffB",
                                AP_HAL::micros64(),
                                poscontrol.get_state(),
                                plane.auto_state.wp_distance,
                                poscontrol.target_speed_ms,
                                poscontrol.target_accel_mss,
                                poscontrol.overshoot);
}
#endif

/*
  change position control state
 */
void QuadPlane::PosControlState::set_state(enum position_control_state s)
{
    const uint32_t now = AP_HAL::millis();
    if (state != s) {
        auto &qp = plane.quadplane;
        pilot_correction_done = false;
        // handle resets needed for when the state changes
        if (s == QPOS_POSITION1) {
            reached_wp_speed = false;
            // never do a rate reset, if attitude control is not active it will be automatically reset before running, see: last_att_control_ms
            // if it is active then the rate control should not be reset at all
            qp.attitude_control->reset_yaw_target_and_rate(false);
            pos1_speed_limit_ms = plane.ahrs.groundspeed_vector().length();
            done_accel_init = false;
        } else if (s == QPOS_AIRBRAKE) {
            // start with zero integrator on vertical throttle
            qp.pos_control->get_accel_U_pid().set_integrator(0);
        } else if (s == QPOS_LAND_DESCEND) {
            // reset throttle descent control
            qp.thr_ctrl_land = false;
            qp.land_descend_start_alt_m = plane.current_loc.alt*0.01;
            last_override_descent_ms = 0;
        } else if (s == QPOS_LAND_ABORT) {
            // reset throttle descent control
            qp.thr_ctrl_land = false;
        } else if (s == QPOS_LAND_FINAL) {
            // remember last pos reset to handle GPS glitch in LAND_FINAL
            Vector2f rpos;
            last_pos_reset_ms = plane.ahrs.getLastPosNorthEastReset(rpos);
            qp.landing_detect.land_start_ms = 0;
            qp.landing_detect.lower_limit_start_ms = 0;
        }
        // double log to capture the state change
#if HAL_LOGGING_ENABLED
        qp.log_QPOS();
#endif
        state = s;
#if HAL_LOGGING_ENABLED
        qp.log_QPOS();
#endif
        last_log_ms = now;
        overshoot = false;
    }
    last_state_change_ms = now;

    // we consider setting the state to be equivalent to running to
    // prevent code from overriding the state as stale
    last_run_ms = now;
}

/*
  main landing controller. Used for landing and RTL.
 */
void QuadPlane::vtol_position_controller(void)
{
    if (!setup()) {
        return;
    }

    const Location &loc = plane.next_WP_loc;
    uint32_t now_ms = AP_HAL::millis();

    // distance that we switch to QPOS_POSITION2
    const float position2_dist_threshold_m = 10.0;

    // target speed when we reach position2 threshold
    const float position2_target_speed_ms = 3.0;

    if (plane.arming.is_armed_and_safety_off()) {
        poscontrol.last_run_ms = now_ms;
    }

    // avoid running the z controller in approach and airbrake if we're not already running it
    // and tilt is more than tilt max
    bool suppress_z_controller = false;

    Vector2f landing_velocity_ne_ms;
    if (now_ms - poscontrol.last_velocity_match_ms < 1000) {
        landing_velocity_ne_ms = poscontrol.velocity_match_ms;
    }

    // horizontal position control
    switch (poscontrol.get_state()) {

    case QPOS_NONE:
        poscontrol.set_state(QPOS_POSITION1);
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;

    case QPOS_APPROACH:
        if (in_vtol_mode()) {
            // this means we're not running transition update code and
            // thus not doing qassist checking, force POSITION1 mode
            // now. We don't expect this to trigger, it is a failsafe
            // for a logic error
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 nvtol");
            poscontrol.set_state(QPOS_POSITION1);
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
        FALLTHROUGH;

    case QPOS_AIRBRAKE: {
        float aspeed_ms;
        const Vector2f closing_vel_ne_ms = landing_closing_velocity_NE_ms();
        const Vector2f desired_closing_vel_ne_ms = landing_desired_closing_velocity_NE_ms();
        const float groundspeed_ms = plane.ahrs.groundspeed();
        const float distance_m = plane.auto_state.wp_distance;
        const float closing_speed_ms = closing_vel_ne_ms.length();
        const float desired_closing_speed_ms = desired_closing_vel_ne_ms.length();
        if (!plane.ahrs.airspeed_estimate(aspeed_ms)) {
            aspeed_ms = groundspeed_ms;
        }

        if (tiltrotor.enabled() && poscontrol.get_state() == QPOS_AIRBRAKE) {
            if ((now_ms - last_pidz_active_ms > 2000 && tiltrotor.tilt_over_max_angle()) ||
                tiltrotor.current_tilt >= tiltrotor.get_fully_forward_tilt()) {
                // use low throttle stabilization when airbraking on a
                // tiltrotor. We don't want quite zero throttle as we
                // want some drag, but don't want to run the Z
                // controller which can result in high throttle on
                // motors that are tilted forward, thus increasing
                // speed
                suppress_z_controller = true;
                hold_stabilize(0.01);
            }
        }
        
        // speed for crossover to POSITION1 controller
        const float aspeed_threshold_ms = MAX(plane.aparm.airspeed_min - 2, assist.speed);

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.auto_state.crosstrack ? plane.prev_WP_loc : plane.current_loc, loc);

        // use TECS for throttle
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.TECS_controller.get_throttle_demand());

        // use TECS for pitch
        int32_t commanded_pitch = plane.TECS_controller.get_pitch_demand();
        plane.nav_pitch_cd = constrain_int32(commanded_pitch, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
        if (poscontrol.get_state() == QPOS_AIRBRAKE) {
            // don't allow down pitch in airbrake
            plane.nav_pitch_cd = MAX(plane.nav_pitch_cd, 0);
        }

        // use nav controller roll
        plane.calc_nav_roll();

        // work out the point to enter airbrake mode. We want enough
        // distance to stop, plus some margin for the time it takes to
        // change the accel (jerk limit) plus the min time in airbrake
        // mode. For simplicity we assume 2 seconds margin
        const float stop_distance = stopping_distance_m() + 2*closing_speed_ms;

        if (!suppress_z_controller && poscontrol.get_state() == QPOS_AIRBRAKE) {
            hold_hover(0);
            // don't run Z controller again in this loop
            suppress_z_controller = true;
        }

        /*
          see if we should start airbraking stage. For non-tailsitters
          we can use the VTOL motors as airbrakes by firing them up
          before we transition. This gives a smoother transition and
          gives us a nice lot of deceleration
         */
        if (poscontrol.get_state() == QPOS_APPROACH && distance_m < stop_distance) {
            if (tailsitter.enabled() || motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                // tailsitters don't use airbrake stage for landing
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f d=%.0f sd=%.0f h=%.1f",
                                groundspeed_ms,
                                plane.auto_state.wp_distance,
                                stop_distance,
                                plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
                poscontrol.set_state(QPOS_POSITION1);
                transition->set_last_fw_pitch();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL airbrake v=%.1f d=%.0f sd=%.0f h=%.1f",
                                groundspeed_ms,
                                distance_m,
                                stop_distance,
                                plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
                poscontrol.set_state(QPOS_AIRBRAKE);
            }
        }

        /*
          we must switch to POSITION1 if our airspeed drops below the
          assist speed. We additionally switch to POSITION1 if we are
          too far above our desired velocity profile, or our attitude
          has deviated too much
         */
        const int32_t attitude_error_threshold_cd = 1000;

        // use at least 1s of airbrake time to ensure motors have a chance to
        // properly spin up
        const uint32_t min_airbrake_ms = 1000;
        if (poscontrol.get_state() == QPOS_AIRBRAKE &&
            poscontrol.time_since_state_start_ms() > min_airbrake_ms &&
            (aspeed_ms < aspeed_threshold_ms || // too low airspeed
             fabsf(degrees(closing_vel_ne_ms.angle(desired_closing_vel_ne_ms))) > 60 || // wrong direction
             closing_speed_ms > MAX(desired_closing_speed_ms*1.2, desired_closing_speed_ms+2) || // too fast
             closing_speed_ms < desired_closing_speed_ms*0.5 || // too slow ground speed
             labs(plane.ahrs.roll_sensor - plane.nav_roll_cd) > attitude_error_threshold_cd || // bad attitude
             labs(plane.ahrs.pitch_sensor - plane.nav_pitch_cd) > attitude_error_threshold_cd)) {
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f d=%.1f h=%.1f dc=%.1f",
                            (double)groundspeed_ms,
                            (double)plane.auto_state.wp_distance,
                            plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING),
                            desired_closing_speed_ms);
            poscontrol.set_state(QPOS_POSITION1);
            transition->set_last_fw_pitch();

            // switch to vfwd for throttle control
            vel_forward.integrator = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

            // adjust the initial forward throttle based on our desired and actual closing speed
            // this allows for significant initial forward throttle
            // when we have a strong headwind, but low throttle in the usual case where
            // we want to slow down ready for POSITION2
            vel_forward.integrator = linear_interpolate(0, vel_forward.integrator,
                                                        closing_speed_ms,
                                                        1.2*desired_closing_speed_ms, 0.5*desired_closing_speed_ms);

            // limit our initial forward throttle in POSITION1 to be 0.5 of cruise throttle
            vel_forward.integrator = constrain_float(vel_forward.integrator, 0, plane.aparm.throttle_cruise*0.5);
            
            vel_forward.last_ms = now_ms;
        }

        if (!tiltrotor.enabled() && !tailsitter.enabled()) {
            /*
              cope with fwd motor thrust loss during approach. We detect
              this by looking for the fwd throttle saturating. This only
              applies to separate lift-thrust vehicles
            */
            bool throttle_saturated = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) >= plane.aparm.throttle_max;
            if (throttle_saturated &&
                motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
                plane.auto_state.sink_rate > 0.2 && aspeed_ms < aspeed_threshold_ms + 4) {
                if (poscontrol.thrust_loss_start_ms == 0) {
                    poscontrol.thrust_loss_start_ms = now_ms;
                }
                if (now_ms - poscontrol.thrust_loss_start_ms > 5000) {
                    gcs().send_text(MAV_SEVERITY_INFO,"VTOL pos1 thrust loss as=%.1f at=%.1f",
                                    aspeed_ms, aspeed_threshold_ms);
                    poscontrol.set_state(QPOS_POSITION1);
                    transition->set_last_fw_pitch();
                }
            } else {
                poscontrol.thrust_loss_start_ms = 0;
            }

            // handle loss of forward thrust in approach based on low airspeed detection
            if (poscontrol.get_state() == QPOS_APPROACH && aspeed_ms < aspeed_threshold_ms &&
                motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL pos1 low speed as=%.1f at=%.1f",
                                aspeed_ms, aspeed_threshold_ms);
                poscontrol.set_state(QPOS_POSITION1);
                transition->set_last_fw_pitch();
            }
        }

        if (poscontrol.get_state() == QPOS_APPROACH) {
            poscontrol_init_approach();
        }
        break;
    }

    case QPOS_POSITION1: {
        setup_target_position();

        if (tailsitter.enabled() && tailsitter.in_vtol_transition(now_ms)) {
            break;
        }

        const Vector2f wp_distance_ne_m = plane.current_loc.get_distance_NE(loc);
        const float wp_distance_m = wp_distance_ne_m.length();
        const Vector2f rel_groundspeed_vector_ne_ms = landing_closing_velocity_NE_ms();
        const float rel_groundspeed_sq = rel_groundspeed_vector_ne_ms.length_squared();
        float closing_groundspeed_ms = 0;

        if (wp_distance_m > 0.1) {
            closing_groundspeed_ms = rel_groundspeed_vector_ne_ms * wp_distance_ne_m.normalized();
        }

        // calculate speed we should be at to reach the position2
        // target speed at the position2 distance threshold, assuming
        // Q_TRANS_DECEL is correct
        const float stopping_speed_ms = safe_sqrt(MAX(0, wp_distance_m - position2_dist_threshold_m) * 2 * transition_decel_mss + sq(position2_target_speed_ms));

        float approach_speed_ms = stopping_speed_ms;

        // maximum configured VTOL speed
        const float wp_speed_ms = MAX(1.0, wp_nav->get_default_speed_NE_ms());
        const float scaled_wp_speed_ms = get_scaled_wp_speed(degrees(wp_distance_ne_m.angle()));

        // limit target speed to a the pos1 speed limit, which starts out at the initial speed
        // but is adjusted if we start putting our nose down. We always allow at least twice
        // the WP speed
        approach_speed_ms = MIN(MAX(poscontrol.pos1_speed_limit_ms, 2 * wp_speed_ms), approach_speed_ms);

        if (poscontrol.reached_wp_speed ||
            rel_groundspeed_sq < sq(wp_speed_ms) ||
            wp_speed_ms > 1.35*scaled_wp_speed_ms) {
            // once we get below the Q_WP_SPEED then we don't want to
            // speed up again. At that point we should fly within the
            // limits of the configured VTOL controller we also apply
            // this limit when we are more than 45 degrees off the
            // target in yaw, which is when we start to become
            // unstable
            approach_speed_ms = MIN(approach_speed_ms, scaled_wp_speed_ms);
            poscontrol.reached_wp_speed = true;
        }

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        Vector2f target_speed_ne_ms;
        Vector2f target_accel_ne_mss;
        bool have_target_yaw = false;
        float target_yaw_deg;
        const float approach_accel_mss = MIN(accel_needed(wp_distance_m, sq(closing_groundspeed_ms)), transition_decel_mss * 2);
        if (wp_distance_m > 0.1) {
            Vector2f diff_wp_norm = wp_distance_ne_m.normalized();
            target_speed_ne_ms = diff_wp_norm * approach_speed_ms;
            target_accel_ne_mss = diff_wp_norm * (-approach_accel_mss);
            target_yaw_deg = degrees(diff_wp_norm.angle());
            const float yaw_err_deg = wrap_180(target_yaw_deg - degrees(plane.ahrs.get_yaw_rad()));
            bool overshoot = (closing_groundspeed_ms < 0 || fabsf(yaw_err_deg) > 60);
            if (overshoot && !poscontrol.overshoot) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL Overshoot d=%.1f cs=%.1f yerr=%.1f",
                                wp_distance_m, closing_groundspeed_ms, yaw_err_deg);
                poscontrol.overshoot = true;
                pos_control->set_accel_desired_NE_mss(Vector2f());
            }
            if (poscontrol.overshoot) {
                /* we have overshot the landing point or our nose is
                   off by more than 60 degrees. Zero target accel and
                   point nose at the landing point. Set target speed
                   to our position2 threshold speed
                */
                target_accel_ne_mss.zero();

                // allow up to the WP speed when we are further away, slowing to the pos2 target speed
                // when we are close
                approach_speed_ms = linear_interpolate(position2_target_speed_ms, wp_speed_ms,
                                                  wp_distance_m,
                                                  position2_dist_threshold_m*1.5,
                                                  2*position2_dist_threshold_m + stopping_distance_m(rel_groundspeed_sq));

                target_speed_ne_ms = diff_wp_norm * approach_speed_ms;
                have_target_yaw = true;

                // adjust target yaw angle for wind. We calculate yaw based on the target speed
                // we want assuming no speed scaling due to direction
                const Vector2f wind_ms = plane.ahrs.wind_estimate().xy();
                const float gnd_speed_ms = plane.ahrs.groundspeed();
                Vector2f target_speed_xy = landing_velocity_ne_ms + diff_wp_norm * gnd_speed_ms - wind_ms;
                target_yaw_deg = degrees(target_speed_xy.angle());
            }
        }
        const float target_speed_ms = target_speed_ne_ms.length();

        target_speed_ne_ms += landing_velocity_ne_ms;
        poscontrol.target_speed_ms = target_speed_ms;
        poscontrol.target_accel_mss = approach_accel_mss;

        if (!poscontrol.reached_wp_speed &&
            rel_groundspeed_sq < sq(target_speed_ms) &&
            rel_groundspeed_sq > sq(2*wp_speed_ms) &&
            plane.nav_pitch_cd < 0) {
            // we have slowed down more than expected, likely due to
            // drag from the props and we're starting to put our nose
            // down as a result. We want to accept the slowdown and
            // re-calculate the target speed profile
            poscontrol.pos1_speed_limit_ms = sqrtf(rel_groundspeed_sq);
        }

        // use input shaping and abide by accel and jerk limits
        pos_control->input_vel_accel_NE_m(target_speed_ne_ms, target_accel_ne_mss);

        // run horizontal velocity controller
        run_xy_controller(MAX(approach_accel_mss, transition_decel_mss)*1.5);

        if (!poscontrol.done_accel_init) {
            /*
              the pos controller init assumes zero accel, we need to
              override that so that we can start decelerating more
              quickly at the start of POSITION1
             */
            poscontrol.done_accel_init = true;
            pos_control->set_accel_desired_NE_mss(target_accel_ne_mss);
        }
        
        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();

        if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
            pos_control->set_externally_limited_NE();
        }

        // call attitude controller
        disable_yaw_rate_time_constant();

        // setup scaling of roll and pitch angle P gains to match fixed wing gains
        setup_rp_fw_angle_gains();

        if (have_target_yaw) {
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(plane.nav_roll_cd,
                                                               plane.nav_pitch_cd,
                                                               target_yaw_deg * 100, true);
        } else {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          desired_auto_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        }
        if ((plane.auto_state.wp_distance < position2_dist_threshold_m) && tiltrotor.tilt_angle_achieved() &&
            fabsf(rel_groundspeed_sq) < sq(3 * position2_target_speed_ms)) {
            // if continuous tiltrotor only advance to position 2 once tilts have finished moving
            poscontrol.set_state(QPOS_POSITION2);
            poscontrol.pilot_correction_done = false;
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f h=%.1f",
                            (double)ahrs.groundspeed(), (double)plane.auto_state.wp_distance,
                            plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
        }
        break;
    }

    case QPOS_POSITION2:
    case QPOS_LAND_ABORT:
    case QPOS_LAND_DESCEND: {
        setup_target_position();
        /*
          for final land repositioning and descent we run the position controller
         */
        Vector2f zero;
        Vector2f vel_ne_ms = poscontrol.target_vel_ms.xy() + landing_velocity_ne_ms;
        pos_control->input_pos_vel_accel_NE_m(poscontrol.target_neu_m.xy(), vel_ne_ms, zero);

        // also run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        update_land_positioning();

        run_xy_controller(transition_decel_mss*1.5);

        // nav roll and pitch are controlled by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();

        if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
            pos_control->set_externally_limited_NE();
        }

        // call attitude controller
        set_pilot_yaw_rate_time_constant();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        break;
    }

    case QPOS_LAND_FINAL:
        update_land_positioning();

        // relax when close to the ground
        if (should_relax()) {
            pos_control->relax_velocity_controller_NE();
        } else {
            Vector2f zero;
            Vector2f vel_ne_ms = poscontrol.target_vel_ms.xy() + landing_velocity_ne_ms;
            Vector2f rpos;
            const uint32_t last_reset_ms = plane.ahrs.getLastPosNorthEastReset(rpos);
            /* we use velocity control when we may be touching the
              ground or if we've had a position reset from AHRS. This
              helps us handle a GPS glitch in the final land phase,
              and also prevents trying to reposition after touchdown
            */
            if (motors->limit.throttle_lower ||
                motors->get_throttle() < 0.5*motors->get_throttle_hover() ||
                last_reset_ms != poscontrol.last_pos_reset_ms) {
                pos_control->input_vel_accel_NE_m(vel_ne_ms, zero);
            } else {
                // otherwise use full pos control
                pos_control->input_pos_vel_accel_NE_m(poscontrol.target_neu_m.xy(), vel_ne_ms, zero);
            }
        }

        run_xy_controller();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();

        // call attitude controller
        set_pilot_yaw_rate_time_constant();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        break;

    case QPOS_LAND_COMPLETE:
        // nothing to do
        break;
    }

    // now height control
    switch (poscontrol.get_state()) {
    case QPOS_NONE:
        poscontrol.set_state(QPOS_POSITION1);
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;

    case QPOS_APPROACH:
    case QPOS_AIRBRAKE:
        // we just want stability from the VTOL controller in these
        // phases of landing, so relax the Z controller, unless we are
        // providing assistance
        if (transition->complete()) {
            pos_control->relax_U_controller(0);
        }
        break;
    case QPOS_POSITION1:
        if (tailsitter.in_vtol_transition(now_ms)) {
            pos_control->relax_U_controller(0);
            break;
        }
        FALLTHROUGH;
    case QPOS_POSITION2: {
        bool vtol_loiter_auto = false;
        if (plane.control_mode == &plane.mode_auto) {
            switch (plane.mission.get_current_nav_cmd().id) {
            case MAV_CMD_NAV_LOITER_UNLIM:
            case MAV_CMD_NAV_LOITER_TIME:
            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_LOITER_TO_ALT:
                vtol_loiter_auto = true;
                break;
            }
        }
        if (plane.control_mode == &plane.mode_guided || vtol_loiter_auto) {
            plane.ahrs.get_location(plane.current_loc);
            int32_t target_altitude_cm;
            if (!plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,target_altitude_cm)) {
                break;
            }
            if (poscontrol.slow_descent &&
                plane.prev_WP_loc.get_distance(plane.next_WP_loc) > 50) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = plane.current_loc.line_path_proportion(plane.prev_WP_loc, plane.next_WP_loc);
                int32_t prev_alt_cm;
                if (plane.prev_WP_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,prev_alt_cm)) {
                    target_altitude_cm = linear_interpolate(prev_alt_cm,
                                                         target_altitude_cm,
                                                         plane.auto_state.wp_proportion,
                                                         0, 1);
                }
            }
            float zero = 0;
            float target_u_m = target_altitude_cm * 0.01;
            pos_control->input_pos_vel_accel_U_m(target_u_m, zero, 0);
        } else if (plane.control_mode == &plane.mode_qrtl) {
            Location loc2 = loc;
            loc2.change_alt_frame(Location::AltFrame::ABOVE_ORIGIN);
            float target_u_m = loc2.alt * 0.01;
            float zero = 0;
            pos_control->input_pos_vel_accel_U_m(target_u_m, zero, 0);
        } else {
            set_climb_rate_ms(0);
        }
        break;
    }

    case QPOS_LAND_DESCEND:
    case QPOS_LAND_ABORT:
    case QPOS_LAND_FINAL: {
        float height_above_ground_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
        if (poscontrol.get_state() == QPOS_LAND_FINAL) {
            if (!option_is_set(QuadPlane::Option::DISABLE_GROUND_EFFECT_COMP)) {
                ahrs.set_touchdown_expected(true);
            }
        }
        if (poscontrol.get_state() == QPOS_LAND_ABORT) {
            set_climb_rate_ms(wp_nav->get_default_speed_up_ms());
            break;
        }
        const float descent_rate_ms = landing_descent_rate_ms(height_above_ground_m);
        pos_control->land_at_climb_rate_m(-descent_rate_ms, descent_rate_ms > 0);
        break;
    }

    case QPOS_LAND_COMPLETE:
        break;
    }

    /*
      run the z controller unless something has already run it or set a target throttle
     */
    if (!suppress_z_controller) {
        // otherwise run z controller
        run_z_controller();
    }

#if HAL_LOGGING_ENABLED
    if (now_ms - poscontrol.last_log_ms >= 40) {
        // log poscontrol at 25Hz
        poscontrol.last_log_ms = now_ms;
        log_QPOS();
    }
#endif
}

/*
  determine which fwd throttle handling method is active
 */
QuadPlane::ActiveFwdThr QuadPlane::get_vfwd_method(void) const
{
    const bool have_fwd_thr_gain = is_positive(q_fwd_thr_gain);
    const bool have_vfwd_gain = is_positive(vel_forward.gain);

#if AP_ICENGINE_ENABLED
    const auto ice_state = plane.g2.ice_control.get_state();
    if (ice_state != AP_ICEngine::ICE_DISABLED && ice_state != AP_ICEngine::ICE_RUNNING) {
        // we need the engine running for fwd throttle
        return ActiveFwdThr::NONE;
    }
#endif

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        return ActiveFwdThr::NONE;
    }
#endif

    if (have_fwd_thr_gain) {
        if (vfwd_enable_active) {
            // user has used AUX function to activate new method
            return ActiveFwdThr::NEW;
        }
        if (q_fwd_thr_use == FwdThrUse::ALL) {
            return ActiveFwdThr::NEW;
        }
        if (q_fwd_thr_use == FwdThrUse::POSCTRL && pos_control->is_active_NE()) {
            return ActiveFwdThr::NEW;
        }
    }
    if (have_vfwd_gain && pos_control->is_active_NE()) {
        return ActiveFwdThr::OLD;
    }
    return ActiveFwdThr::NONE;
}

/*
  map from pitch tilt to fwd throttle when enabled
 */
void QuadPlane::assign_tilt_to_fwd_thr(void)
{

    const auto fwd_thr_active = get_vfwd_method();
    if (fwd_thr_active != ActiveFwdThr::NEW) {
        q_fwd_throttle = 0.0f;
        q_fwd_pitch_lim_cd = 100.0f * q_fwd_pitch_lim;
        return;
    }
    // Handle the case where we are limiting the forward pitch angle to prevent negative wing lift
    // and are using the forward thrust motor or tilting rotors to provide the forward acceleration
    float fwd_tilt_rad = radians(constrain_float(-0.01f * (float)plane.nav_pitch_cd, 0.0f, 45.0f));
    q_fwd_throttle = MIN(q_fwd_thr_gain * tanf(fwd_tilt_rad), 1.0f);

    // Relax forward tilt limit if the position controller is saturating in the forward direction because
    // the forward thrust motor could be failed. Do not do this with tilt rotors because they do not rely on
    // forward throttle during VTOL flight
    if (!tiltrotor.enabled()) {
        const float fwd_tilt_range_cd = (float)aparm.angle_max - 100.0f * q_fwd_pitch_lim;
        if (is_positive(fwd_tilt_range_cd)) {
            // rate limit the forward tilt change to slew between the motor good and motor failed
            // value over 10 seconds
            const bool fwd_limited = plane.quadplane.pos_control->is_active_NE() and plane.quadplane.pos_control->get_fwd_pitch_is_limited();
            const float fwd_pitch_lim_cd_tgt = fwd_limited ? (float)aparm.angle_max : 100.0f * q_fwd_pitch_lim;
            const float delta_max = 0.1f * fwd_tilt_range_cd * plane.G_Dt;
            q_fwd_pitch_lim_cd += constrain_float((fwd_pitch_lim_cd_tgt - q_fwd_pitch_lim_cd), -delta_max, delta_max);
            // Don't let the forward pitch limit be more than the forward pitch demand before limiting to
            // avoid opening up the limit more than necessary
            q_fwd_pitch_lim_cd = MIN(q_fwd_pitch_lim_cd, MAX(-(float)plane.nav_pitch_cd, 100.0f * q_fwd_pitch_lim));
        } else {
            // take the lesser of the two limits
            q_fwd_pitch_lim_cd = (float)aparm.angle_max;
        }
    }

    // Prevent the wing from being overloaded when braking from high speed in a VTOL mode
    float nav_pitch_upper_limit_cd = 100.0f * q_bck_pitch_lim;
    float aspeed;
    if (is_positive(q_bck_pitch_lim) && ahrs.airspeed_estimate(aspeed)) {
        const float reference_speed = MAX(plane.aparm.airspeed_min, MIN_AIRSPEED_MIN);
        float speed_scaler = sq(reference_speed / MAX(aspeed, 0.1f));
        nav_pitch_upper_limit_cd *= speed_scaler;
        nav_pitch_upper_limit_cd = MIN(nav_pitch_upper_limit_cd, (float)aparm.angle_max);

        const float tconst = 0.5f;
        const float dt = AP_HAL::millis() - q_pitch_limit_update_ms;
        q_pitch_limit_update_ms = AP_HAL::millis();
        if (is_positive(dt)) {
            const float coef = dt / (dt + tconst);
            q_bck_pitch_lim_cd = (1.0f - coef) * q_bck_pitch_lim_cd + coef * nav_pitch_upper_limit_cd;
        }

        plane.nav_pitch_cd = MIN(plane.nav_pitch_cd, (int32_t)q_bck_pitch_lim_cd);

#if HAL_LOGGING_ENABLED
        // @LoggerMessage: QBRK
        // @Description: Quadplane Braking
        // @Field: TimeUS: Time since system startup
        // @Field: SpdScaler: braking speed scaler
        // @Field: NPULCD: upper limit for navigation pitch
        // @Field: QBPLCD: upper limit for back transition pitch
        // @Field: NPCD: demanded navigation pitch
        AP::logger().WriteStreaming("QBRK",
                                "TimeUS,SpdScaler,NPULCD,QBPLCD,NPCD",  // labels
                                "Qffii",    // fmt
                                AP_HAL::micros64(),
                                (double)speed_scaler,
                                (double)nav_pitch_upper_limit_cd,
                                (int32_t)q_bck_pitch_lim_cd,
                                (int32_t)plane.nav_pitch_cd);
#endif
    }

    float fwd_thr_scaler;
    if (!in_vtol_land_approach()) {
        // To prevent forward motor prop strike, reduce throttle to zero when close to ground.
        float alt_cutoff_m = MAX(0, vel_forward_alt_cutoff_m);
        float height_above_ground_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
        fwd_thr_scaler = linear_interpolate(0.0f, 1.0f, height_above_ground_m, alt_cutoff_m, alt_cutoff_m + 2);
    } else {
        // When we are doing horizontal positioning in a VTOL land we always allow the fwd motor
        // to run. Otherwise a bad height above landing point estimate could cause the aircraft
        // not to be able to approach the landing point
        fwd_thr_scaler = 1.0f;
    }
    q_fwd_throttle *= fwd_thr_scaler;

    // When reducing forward throttle use, relax lower pitch limit to maintain forward
    // acceleration capability.
    const float nav_pitch_lower_limit_cd = - (int32_t)((float)aparm.angle_max * (1.0f - fwd_thr_scaler) + q_fwd_pitch_lim_cd * fwd_thr_scaler);

#if HAL_LOGGING_ENABLED
    // Diagnostics logging - remove when feature is fully flight tested.
    // @LoggerMessage: FWDT
    // @Description: Forward Throttle calculations
    // @Field: TimeUS: Time since system startup
    // @Field: fts: forward throttle scaler
    // @Field: qfplcd: quadplane forward pitch limit
    // @Field: npllcd: navigation pitch lower limit
    // @Field: npcd: demanded navigation pitch
    // @Field: qft: quadplane forward throttle
    // @Field: npulcd: upper limit for navigation pitch
    AP::logger().WriteStreaming("FWDT",
                                "TimeUS,fts,qfplcd,npllcd,npcd,qft,npulcd",  // labels
                                "Qffffff",    // fmt
                                AP_HAL::micros64(),
                                (double)fwd_thr_scaler,
                                (double)q_fwd_pitch_lim_cd,
                                (double)nav_pitch_lower_limit_cd,
                                (double)plane.nav_pitch_cd,
                                (double)q_fwd_throttle,
                                (float)nav_pitch_upper_limit_cd);
#endif

    plane.nav_pitch_cd = MAX(plane.nav_pitch_cd, (int32_t)nav_pitch_lower_limit_cd);
}

/*
  we want to limit WP speed to a lower speed when more than 20 degrees
  off pointing at the destination. quadplanes are often
  unstable when flying sideways or backwards
*/
float QuadPlane::get_scaled_wp_speed(float target_bearing_deg) const
{
    const float yaw_difference = fabsf(wrap_180(degrees(plane.ahrs.get_yaw_rad()) - target_bearing_deg));
    const float wp_speed_ms = wp_nav->get_default_speed_NE_ms();
    if (yaw_difference > 20) {
        // this gives a factor of 2x reduction in max speed when
        // off by 90 degrees, and 3x when off by 180 degrees
        const float speed_reduction = linear_interpolate(1, 3,
                                                         yaw_difference,
                                                         20, 160);
        return wp_speed_ms / speed_reduction;
    }
    return wp_speed_ms;
}

/*
  setup the target position based on plane.next_WP_loc
 */
void QuadPlane::setup_target_position(void)
{
    const Location &loc = plane.next_WP_loc;
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    if (!in_vtol_land_approach() || poscontrol.get_state() > QPOS_APPROACH) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    Vector2f diff2d = origin.get_distance_NE(loc);
    diff2d += poscontrol.correction_ne_m;
    poscontrol.target_neu_m.x = diff2d.x;
    poscontrol.target_neu_m.y = diff2d.y;
    poscontrol.target_neu_m.z = (plane.next_WP_loc.alt - origin.alt) * 0.01;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_velocity_z_max_dn_m(), pilot_speed_z_max_up_ms, pilot_accel_z_mss);
    pos_control->set_correction_speed_accel_U_mss(-get_pilot_velocity_z_max_dn_m(), pilot_speed_z_max_up_ms, pilot_accel_z_mss);
}

/*
  run takeoff controller to climb vertically
 */
void QuadPlane::takeoff_controller(void)
{
    // reset fixed wing controller to neutral as base output
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    if (!plane.arming.is_armed_and_safety_off()) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    const auto spool_state = motors->get_desired_spool_state();
    if (plane.control_mode == &plane.mode_guided && guided_takeoff
        && tiltrotor.enabled() && !tiltrotor.fully_up() &&
        spool_state != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // waiting for motors to tilt up
        takeoff_start_time_ms = now;
        return;
    }

    // don't takeoff up until rudder is re-centered after rudder arming
    if (plane.arming.last_arm_method() == AP_Arming::Method::RUDDER &&
        (takeoff_last_run_ms == 0 ||
         now - takeoff_last_run_ms > 1000) &&
        !rc().seen_neutral_rudder() &&
        spool_state <= AP_Motors::DesiredSpoolState::GROUND_IDLE) {
        // start motor spinning if not spinning already so user sees it is armed
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        takeoff_start_time_ms = now;
        if (now - plane.takeoff_state.rudder_takeoff_warn_ms > TAKEOFF_RUDDER_WARNING_TIMEOUT) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff waiting for rudder release");
            plane.takeoff_state.rudder_takeoff_warn_ms = now;
        }
        return;
    }


    /*
      for takeoff we use the position controller
    */
    setup_target_position();

    // set position control target and update

    Vector2f vel_ne_ms, zero;
    if (AP_HAL::millis() - poscontrol.last_velocity_match_ms < 1000) {
        vel_ne_ms = poscontrol.velocity_match_ms;
    }

    /*
      support zeroing roll/pitch during early part of takeoff. This
      can help particularly with poor GPS velocity data
     */
    bool no_navigation = false;
    if (takeoff_navalt_min_m > 0) {
        const float alt_m = plane.current_loc.alt * 0.01;
        if (takeoff_last_run_ms == 0 ||
            now - takeoff_last_run_ms > 1000) {
            takeoff_start_alt_m = alt_m;
        }
        if (alt_m - takeoff_start_alt_m < takeoff_navalt_min_m) {
            no_navigation = true;
        }
    }
    takeoff_last_run_ms = now;

    if (no_navigation) {
        pos_control->relax_velocity_controller_NE();
    } else {
        pos_control->input_vel_accel_NE_m(vel_ne_ms, zero);

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();
    }

    run_xy_controller();

    set_pilot_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());

    float vel_u_ms = wp_nav->get_default_speed_up_ms();
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        // for guided takeoff we aim for a specific height with zero
        // velocity at that height
        Location origin;
        if (ahrs.get_origin(origin)) {
            // a small margin to ensure we do move to the next takeoff
            // stage
            const int32_t margin_cm = 5;
            float pos_u_m = (margin_cm + plane.next_WP_loc.alt - origin.alt) * 0.01;
            vel_u_ms = 0;
            pos_control->input_pos_vel_accel_U_m(pos_u_m, vel_u_ms, 0);
        } else {
            set_climb_rate_ms(vel_u_ms);
        }
    } else {
        set_climb_rate_ms(vel_u_ms);
    }

    run_z_controller();
}

/*
  run waypoint controller between prev_WP_loc and next_WP_loc
 */
void QuadPlane::waypoint_controller(void)
{
    setup_target_position();

    const Location &loc = plane.next_WP_loc;
    const uint32_t now = AP_HAL::millis();
    if (!loc.same_loc_as(last_auto_target) ||
        now - last_loiter_ms > 500) {
        wp_nav->set_wp_destination_NEU_m(poscontrol.target_neu_m.tofloat());
        last_auto_target = loc;
    }
    last_loiter_ms = now;

    /*
      this is full copter control of auto flight
    */
    // run wpnav controller
    wp_nav->update_wpnav();

    // nav roll and pitch are controller by waypoint controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    assign_tilt_to_fwd_thr();

    if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_NE();
    }

    // call attitude controller
    disable_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(plane.nav_roll_cd,
                                                       plane.nav_pitch_cd,
                                                       wp_nav->get_yaw(),
                                                       true);

    // climb based on altitude error
    set_climb_rate_ms(assist_climb_rate_cms() * 0.01);
    run_z_controller();
}


/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(void)
{
    if (!setup()) {
        return;
    }

    if (poscontrol.get_state() > QPOS_APPROACH) {
        bool should_run_motors = false;

        // don't run the motors if in an arming delay
        if (plane.arming.get_delay_arming()) {
            should_run_motors = false;
        }

        // don't run motors if we are in the wait state for payload place
        if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::SHUT_DOWN &&
            plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE) &&
            poscontrol.get_state() == QPOS_LAND_COMPLETE) {
            should_run_motors = false;
        }
        
        if (should_run_motors) {
            set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
    }

    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        if (is_vtol_takeoff(id)) {
            takeoff_controller();
        }
        break;
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
    case MAV_CMD_NAV_LAND:
        if (is_vtol_land(id)) {
            vtol_position_controller();
        }
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT: {
        const uint32_t now = AP_HAL::millis();
        if (now - poscontrol.last_run_ms > 100) {
            // ensure that poscontrol is reset
            poscontrol.set_state(QPOS_POSITION1);
        }
        vtol_position_controller();
    }
        break;
    default:
        waypoint_controller();
        break;
    }
}

/*
  start a VTOL takeoff
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    // we always use the current location in XY for takeoff. The altitude defaults
    // to relative to current height, but if Q_OPTIONS is set to respect takeoff frame
    // then it will use normal frame handling for height
    Location loc = cmd.content.location;
    loc.lat = 0;
    loc.lng = 0;
    plane.set_next_WP(loc);
    if (option_is_set(QuadPlane::Option::RESPECT_TAKEOFF_FRAME)) {
        // convert to absolute frame for takeoff
        if (!plane.next_WP_loc.change_alt_frame(Location::AltFrame::ABSOLUTE) ||
            plane.current_loc.alt >= plane.next_WP_loc.alt) {
            // we are above the takeoff already, no need to do anything
            return false;
        }
    } else {
        plane.next_WP_loc.set_alt_cm(plane.current_loc.alt + cmd.content.location.alt,
                                     Location::AltFrame::ABSOLUTE);
    }
    throttle_wait = false;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_velocity_z_max_dn_m(), pilot_speed_z_max_up_ms, pilot_accel_z_mss);
    pos_control->set_correction_speed_accel_U_mss(-get_pilot_velocity_z_max_dn_m(), pilot_speed_z_max_up_ms, pilot_accel_z_mss);

    // initialise the vertical position controller
    pos_control->init_U_controller();

    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.current_loc, plane.next_WP_loc);

    // calculate the time required to complete a takeoff
    // this may be conservative and accept extra time due to clamping
    // derived from the following latex equations if you want a nicely formatted view
    // t_{accel} = \frac{V_max - V_z}{a}
    // d_{accel} = V_z*t_{accel} + \frac{1}{2}*a*t_{accel}^2
    // d_{remaining} = d_{total} - d_{accel}
    // t_{constant} = \frac{d_{remaining}}{V_z}
    // t = max(t_{accel}, 0) + max(t_{constant}, 0)
    const float d_total_m = (plane.next_WP_loc.alt - plane.current_loc.alt) * 0.01f;
    const float accel_m_s_s = MAX(0.1, pilot_accel_z_mss);
    const float vel_max_ms = MAX(0.1, pilot_speed_z_max_up_ms);
    const float vel_u_ms = inertial_nav.get_velocity_z_up_cms() * 0.01f;
    const float t_accel_s = (vel_max_ms - vel_u_ms) / accel_m_s_s;
    const float d_accel_m = vel_u_ms * t_accel_s + 0.5f * accel_m_s_s * sq(t_accel_s);
    const float d_remaining_m = d_total_m - d_accel_m;
    const float t_constant = d_remaining_m / vel_max_ms;
    const float travel_time_s = MAX(t_accel_s, 0) + MAX(t_constant, 0);

    // setup the takeoff failure handling code
    takeoff_start_time_ms = millis();
    takeoff_time_limit_ms = MAX(travel_time_s * takeoff_failure_scalar * 1000, 5000); // minimum time 5 seconds

    return true;
}


/*
  start a VTOL landing
 */
bool QuadPlane::do_vtol_land(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.copy_alt_from(plane.current_loc);

    // initialise the position controller
    pos_control->init_NE_controller();
    pos_control->init_U_controller();

    throttle_wait = false;
    landing_detect.lower_limit_start_ms = 0;
    landing_detect.land_start_ms = 0;

    plane.crash_state.is_crashed = false;
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.auto_state.crosstrack ? plane.prev_WP_loc : plane.current_loc,
                                          plane.next_WP_loc);

    poscontrol_init_approach();
    return true;
}

/*
  check if a VTOL takeoff has completed
 */
bool QuadPlane::verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd)
{
    if (!available()) {
        return true;
    }

    const uint32_t now = millis();

    // reset takeoff if we aren't armed
    if (!plane.arming.is_armed_and_safety_off()) {
        do_vtol_takeoff(cmd);
        return false;
    }

    if (now - takeoff_start_time_ms < 3000 &&
        !option_is_set(QuadPlane::Option::DISABLE_GROUND_EFFECT_COMP)) {
        ahrs.set_takeoff_expected(true);
    }
    
    // check for failure conditions
    if (is_positive(takeoff_failure_scalar) && ((now - takeoff_start_time_ms) > takeoff_time_limit_ms)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to complete takeoff within time limit");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TAKEOFF);
        return false;
    }

#if AP_AIRSPEED_ENABLED
    if (is_positive(maximum_takeoff_airspeed_ms) && (plane.airspeed.get_airspeed() > maximum_takeoff_airspeed_ms)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to complete takeoff, excessive wind");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TAKEOFF);
        return false;
    }
#endif

    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        return false;
    }
    transition->restart();
    plane.TECS_controller.set_pitch_max(transition_pitch_max);
    plane.TECS_controller.set_pitch_min(-transition_pitch_max);

    // todo: why are you doing this, I want to delete it.
    set_alt_target_current();

#if AP_FENCE_ENABLED
    plane.fence.auto_enable_fence_after_takeoff();
#endif

    if (plane.control_mode == &plane.mode_auto) {
        // we reset TECS so that the target height filter is not
        // constrained by the climb and sink rates from the initial
        // takeoff height.
        plane.TECS_controller.reset();
    }

    // don't crosstrack on next WP
    plane.auto_state.next_wp_crosstrack = false;

    return true;
}

/*
  a landing detector based on change in altitude over a timeout
 */
bool QuadPlane::land_detector(uint32_t timeout_ms)
{
    bool might_be_landed = should_relax() && !poscontrol.pilot_correction_active;
    if (!might_be_landed) {
        landing_detect.land_start_ms = 0;
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    float height_m = inertial_nav.get_position_z_up_cm() * 0.01;
    if (landing_detect.land_start_ms == 0) {
        landing_detect.land_start_ms = now;
        landing_detect.vpos_start_m = height_m;
    }

    // we only consider the vehicle landed when the motors have been
    // at minimum for timeout_ms+1000 and the vertical position estimate has not
    // changed by more than 20cm for timeout_ms
    if (fabsf(height_m - landing_detect.vpos_start_m) > landing_detect.detect_alt_change_m) {
        // height has changed, call off landing detection
        landing_detect.land_start_ms = 0;
        return false;
    }
           
    if ((now - landing_detect.land_start_ms) < timeout_ms ||
        (now - landing_detect.lower_limit_start_ms) < (timeout_ms+1000)) {
        // not landed yet
        return false;
    }

    return true;
}

/*
  check if a landing is complete
 */
bool QuadPlane::check_land_complete(void)
{
    if (poscontrol.get_state() != QPOS_LAND_FINAL) {
        // only apply to final landing phase
        return false;
    }
    if (land_detector(4000)) {
        poscontrol.set_state(QPOS_LAND_COMPLETE);
        gcs().send_text(MAV_SEVERITY_INFO,"Land complete");

        if (plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)) {
            // for payload place with full landing we shutdown motors
            // and wait for the lua script to trigger a climb (using
            // landing abort) or disarm
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            return false;
        }

        if (plane.control_mode != &plane.mode_auto ||
            !plane.mission.continue_after_land()) {
            // disarm on land unless we have MIS_OPTIONS setup to
            // continue after land in AUTO
            plane.arming.disarm(AP_Arming::Method::LANDED);
        }
        return true;
    }
    return false;
}


/*
  check if we should switch from QPOS_LAND_DESCEND to QPOS_LAND_FINAL
 */
bool QuadPlane::check_land_final(void)
{
    float height_above_ground_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    // we require 2 readings at 10Hz to be within 5m of each other to
    // trigger the switch to land final. This prevents a short term
    // glitch at high altitude from triggering land final
    const float max_change_m = 5;
    if (height_above_ground_m < land_final_alt_m &&
        fabsf(height_above_ground_m - last_land_final_agl_m) < max_change_m) {
        return true;
    }
    last_land_final_agl_m = height_above_ground_m;

    /*
      also apply landing detector, in case we have landed in descent
      phase. Use a longer threshold
     */
    return land_detector(6000);
}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(void)
{
    if (!available()) {
        return true;
    }

    if (poscontrol.get_state() == QPOS_POSITION2) {
        // see if we should move onto the descend stage of landing
        const float descend_dist_threshold_m = 2.0;
        const float descend_speed_threshold_ms = 3.0;
        bool reached_position = false;
        if (poscontrol.pilot_correction_done) {
            reached_position = !poscontrol.pilot_correction_active;
        } else {
            const float dist_m = (inertial_nav.get_position_neu_cm().topostype() * 0.01 - poscontrol.target_neu_m).xy().length();
            reached_position = dist_m < descend_dist_threshold_m;
        }
        Vector2f approach_vel_ne_ms;
        if (AP_HAL::millis() - poscontrol.last_velocity_match_ms < 1000) {
            approach_vel_ne_ms = poscontrol.velocity_match_ms;
        }
        Vector3f vel_ned_ms;
        UNUSED_RESULT(plane.ahrs.get_velocity_NED(vel_ned_ms));
        
        if (reached_position &&
            (vel_ned_ms.xy() - approach_vel_ne_ms).length() < descend_speed_threshold_ms) {
            poscontrol.set_state(QPOS_LAND_DESCEND);
            poscontrol.pilot_correction_done = false;
            pos_control->set_lean_angle_max_cd(0);
            poscontrol.correction_ne_m.zero();
#if AP_LANDINGGEAR_ENABLED
            plane.g2.landing_gear.deploy_for_landing();
#endif
            last_land_final_agl_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
            gcs().send_text(MAV_SEVERITY_INFO,"Land descend started");
            if (plane.control_mode == &plane.mode_auto) {
                // set height to mission height, so we can use the mission
                // WP height for triggering land final if no rangefinder
                // available
                plane.set_next_WP(plane.mission.get_current_nav_cmd().content.location);
            } else {
                plane.set_next_WP(plane.next_WP_loc);
                plane.next_WP_loc.copy_alt_from(ahrs.get_home());
            }
        }
    }

    // at land_final_alt_m begin final landing
    if (poscontrol.get_state() == QPOS_LAND_DESCEND && check_land_final()) {
        poscontrol.set_state(QPOS_LAND_FINAL);

#if AP_ICENGINE_ENABLED
        // cut IC engine if enabled
        if (land_icengine_cut != 0) {
            plane.g2.ice_control.engine_control(0, 0, 0, false);
        }
#endif  // AP_ICENGINE_ENABLED
        gcs().send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    // at land_final_alt_m begin final landing
    if (poscontrol.get_state() == QPOS_LAND_ABORT &&
        plane.current_loc.alt * 0.01 >= land_descend_start_alt_m) {
        // continue to next WP, if there is one
        return true;
    }

    if (plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE) &&
        (poscontrol.get_state() == QPOS_LAND_DESCEND ||
         poscontrol.get_state() == QPOS_LAND_FINAL)) {
        const auto &cmd = plane.mission.get_current_nav_cmd();
        if (cmd.p1 > 0 && plane.current_loc.alt * 0.01 < land_descend_start_alt_m - cmd.p1 * 0.01) {
            gcs().send_text(MAV_SEVERITY_INFO,"Payload place aborted");
            poscontrol.set_state(QPOS_LAND_ABORT);
        }
    }
    
    if (check_land_complete() && plane.mission.continue_after_land()) {
        gcs().send_text(MAV_SEVERITY_INFO,"Mission continue");
        return true;
    }
    return false;
}

#if HAL_LOGGING_ENABLED
// Write a control tuning packet
void QuadPlane::Log_Write_QControl_Tuning()
{
    float des_alt_m = 0.0f;
    float target_climb_rate_ms = 0;
    if (plane.control_mode != &plane.mode_qstabilize) {
        des_alt_m = pos_control->get_pos_desired_U_m();
        target_climb_rate_ms = pos_control->get_vel_target_U_ms();
    }

    // Assemble assistance bitmask, definition here is used to generate log documentation
    enum class log_assistance_flags {
        in_assisted_flight = 1U<<0, // true if VTOL assist is active
        forced             = 1U<<1, // true if assistance is forced
        speed              = 1U<<2, // true if assistance due to low airspeed
        alt                = 1U<<3, // true if assistance due to low altitude
        angle              = 1U<<4, // true if assistance due to attitude error
        fw_force           = 1U<<5, // true if forcing use of fixed wing controllers
        spin_recovery      = 1U<<6, // true if recovering from a spin
    };

    uint8_t assist_flags = 0;
    if (assisted_flight) {
        assist_flags |= (uint8_t)log_assistance_flags::in_assisted_flight;
    }
    if (assist.in_force_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::forced;
    }
    if (assist.in_speed_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::speed;
    }
    if (assist.in_alt_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::alt;
    }
    if (assist.in_angle_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::angle;
    }
    if (force_fw_control_recovery) {
        assist_flags |= (uint8_t)log_assistance_flags::fw_force;
    }
    if (in_spin_recovery) {
        assist_flags |= (uint8_t)log_assistance_flags::spin_recovery;
    }

    struct log_QControl_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_QTUN_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f,
        baro_alt            : int32_t(plane.barometer.get_altitude() * 100),
        target_climb_rate   : int16_t(target_climb_rate_ms * 100.0),
        climb_rate          : int16_t(inertial_nav.get_velocity_z_up_cms()),
        throttle_mix        : attitude_control->get_throttle_mix(),
        transition_state    : transition->get_log_transition_state(),
        assist              : assist_flags,
    };
    plane.logger.WriteBlock(&pkt, sizeof(pkt));

    // write multicopter position control message
    pos_control->write_log();

    // Write tiltrotor tilt angle log
    tiltrotor.write_log();
}
#endif


/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
float QuadPlane::forward_throttle_pct()
{
    // handle special case where forward thrust motor is used instead of forward pitch.
    if (get_vfwd_method() == ActiveFwdThr::NEW) {
        return 100.0f * q_fwd_throttle;
    }

    /*
      Unless an RC channel is assigned for manual forward throttle control,
      we don't use forward throttle in QHOVER or QSTABILIZE as they are the primary
      recovery modes for a quadplane and need to be as simple as
      possible. They will drift with the wind.
    */
    if (plane.control_mode == &plane.mode_qacro ||
        plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover) {

        if (rc_fwd_thr_ch == nullptr) {
            return 0;
        } else {
            // calculate fwd throttle demand from manual input
            float fwd_thr = rc_fwd_thr_ch->percent_input();

            // set forward throttle to fwd_thr_max * (manual input + mix): range [0,100]
            fwd_thr *= 0.01f * constrain_float(fwd_thr_max, 0, 100);
            return fwd_thr;
        }
    }

    /*
      see if the controller should be active
    */
    if (get_vfwd_method() != ActiveFwdThr::OLD) {
        return 0;
    }

    /*
      in modes with a velocity controller
    */
    float deltat = (AP_HAL::millis() - vel_forward.last_ms) * 0.001f;
    if (deltat > 1 || deltat < 0) {
        vel_forward.integrator = 0;
        deltat = 0.1;
    }
    if (deltat < 0.1) {
        // run at 10Hz
        return vel_forward.last_pct;
    }
    vel_forward.last_ms = AP_HAL::millis();
    
    // work out the desired speed in forward direction
    Vector3f desired_velocity_ned_ms = pos_control->get_vel_desired_NEU_ms();
    desired_velocity_ned_ms.z *= -1;    // convert to NED m/s

    Vector3f vel_ned_ms;
    if (!plane.ahrs.get_velocity_NED(vel_ned_ms)) {
        // we don't know our velocity? EKF must be pretty sick
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
        return 0;
    }
    // get component of velocity error in fwd body frame direction
    Vector3f vel_error_body_ms = ahrs.get_rotation_body_to_ned().transposed() * (desired_velocity_ned_ms - vel_ned_ms);

    float fwd_vel_error_ms = vel_error_body_ms.x;

    // scale forward velocity error by maximum airspeed
    fwd_vel_error_ms /= MAX(plane.aparm.airspeed_max, 5);

    // add in a component from our current pitch demand. This tends to
    // move us to zero pitch. Assume that LIM_PITCH would give us the
    // WP nav speed.
    fwd_vel_error_ms -= wp_nav->get_default_speed_NE_ms() * plane.nav_pitch_cd / (plane.aparm.pitch_limit_max * 100);

    if (should_relax() && vel_ned_ms.length() < 1) {
        // we may be landed
        fwd_vel_error_ms = 0;
        vel_forward.integrator *= 0.95f;
    }
    
    // integrator as throttle percentage (-100 to 100)
    vel_forward.integrator += fwd_vel_error_ms * deltat * vel_forward.gain * 100;

    // inhibit reverse throttle and allow petrol engines with min > 0
    int8_t fwd_throttle_min = plane.have_reverse_thrust() ? 0 : plane.aparm.throttle_min;
    vel_forward.integrator = constrain_float(vel_forward.integrator, fwd_throttle_min, plane.aparm.throttle_cruise);

    if (in_vtol_land_approach()) {
        // when we are doing horizontal positioning in a VTOL land
        // we always allow the fwd motor to run. Otherwise a bad
        // lidar could cause the aircraft not to be able to
        // approach the landing point when landing below the takeoff point
        vel_forward.last_pct = vel_forward.integrator;
    } else if ((in_vtol_land_final() && motors->limit.throttle_lower) ||
#if AP_RANGEFINDER_ENABLED
               (plane.rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) &&
                (plane.rangefinder.status_orient(plane.rangefinder_orientation()) == RangeFinder::Status::OutOfRangeLow))) {
#else
              false) {
#endif
        // we're in the settling phase of landing or using a rangefinder that is out of range low, disable fwd motor
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
    } else {
        // If we are below alt_cutoff_m then scale down the effect until
        // it turns off at alt_cutoff_m and decay the integrator
        float alt_cutoff_m = MAX(0, vel_forward_alt_cutoff_m);
        float height_above_ground_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);

        vel_forward.last_pct = linear_interpolate(0, vel_forward.integrator,
                                                  height_above_ground_m, alt_cutoff_m, alt_cutoff_m + 2);
    }
    if (is_zero(vel_forward.last_pct)) {
        // if the percent is 0 then decay the integrator
        vel_forward.integrator *= 0.95f;
    }

    return vel_forward.last_pct;
}

/*
  get weathervaning yaw rate in cd/s
 */
float QuadPlane::get_weathervane_yaw_rate_cds(void)
{
    /*
      we only do weathervaning in modes where we are doing VTOL
      position control.
    */
    if (!in_vtol_mode() ||
        !transition->allow_weathervane() ||
        !motors->armed() || (motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) ||
        plane.control_mode == &plane.mode_qstabilize ||
#if QAUTOTUNE_ENABLED
        plane.control_mode == &plane.mode_qautotune ||
#endif
        plane.control_mode == &plane.mode_qhover ||
        should_relax()
        ) {
        // Ensure the weathervane controller is reset to prevent weathervaning from happening outside of the timer
        weathervane->reset();
        return 0.0;
    }

    const bool is_takeoff = in_vtol_auto() && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id);
    float wv_output;
    if (weathervane->get_yaw_out(wv_output,
                                     plane.channel_rudder->get_control_in(),
                                     plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING),
                                     pos_control->get_roll_cd(),
                                     pos_control->get_pitch_cd(),
                                     is_takeoff,
                                     in_vtol_land_sequence())) {
        return constrain_float(wv_output * (1/45.0), -100.0, 100.0) * command_model_pilot.get_rate() * 0.5;
    }

    return 0.0;
}

/*
  start guided mode control
 */
void QuadPlane::guided_start(void)
{
    guided_takeoff = false;
    setup_target_position();
    int32_t from_alt;
    int32_t to_alt;
    poscontrol_init_approach();
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
    } else {
        // default back to old method
        poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    }
}

/*
  update guided mode control
 */
void QuadPlane::guided_update(void)
{
    if (plane.control_mode == &plane.mode_guided && guided_takeoff && plane.current_loc.alt < plane.next_WP_loc.alt) {
        throttle_wait = false;
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        takeoff_controller();
    } else {
        if (guided_takeoff) {
            poscontrol.set_state(QPOS_POSITION2);
        }
        guided_takeoff = false;
        // run VTOL position controller
        vtol_position_controller();
    }
}

void QuadPlane::afs_terminate(void)
{
    if (available()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
    }
}

/*
  return true if we should do guided mode loitering using VTOL motors
 */
bool QuadPlane::guided_mode_enabled(void)
{
    if (!available()) {
        return false;
    }
    // only use quadplane guided when in AUTO or GUIDED mode
    if (plane.control_mode != &plane.mode_guided && plane.control_mode != &plane.mode_auto) {
        return false;
    }
    if (plane.control_mode == &plane.mode_auto &&
        plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_TURNS) {
        // loiter turns is a fixed wing only operation
        return false;
    }
    return guided_mode != 0;
}

/*
  set altitude target to current altitude
 */
void QuadPlane::set_alt_target_current(void)
{
    pos_control->set_pos_desired_U_m(inertial_nav.get_position_z_up_cm() * 0.01);
}

// user initiated takeoff for guided mode
bool QuadPlane::do_user_takeoff(float takeoff_altitude)
{
    if (plane.control_mode != &plane.mode_guided) {
        gcs().send_text(MAV_SEVERITY_INFO, "User Takeoff only in GUIDED mode");
        return false;
    }
    if (!plane.arming.is_armed_and_safety_off()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be armed for takeoff");
        return false;
    }
    if (is_flying()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already flying - no takeoff");
        return false;
    }
    plane.auto_state.vtol_loiter = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.current_loc;
    plane.next_WP_loc.offset_up_m(takeoff_altitude);
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    guided_start();
    guided_takeoff = true;
    guided_wait_takeoff = false;
    if (!option_is_set(QuadPlane::Option::DISABLE_GROUND_EFFECT_COMP)) {
        ahrs.set_takeoff_expected(true);
    }
    return true;
}

// return true if the wp_nav controller is being updated
bool QuadPlane::using_wp_nav(void) const
{
    if (plane.control_mode == &plane.mode_qloiter || plane.control_mode == &plane.mode_qland) {
        return true;
    }
    return false;
}

/*
  return mav_type for heartbeat
 */
MAV_TYPE QuadPlane::get_mav_type(void) const
{
    if (mav_type.get() == 0) {
        return MAV_TYPE_FIXED_WING;
    }
    return MAV_TYPE(mav_type.get());
}

/*
  return true if current mission item is a vtol takeoff
*/
bool QuadPlane::is_vtol_takeoff(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_TAKEOFF) {
        return true;
    }
    if (id == MAV_CMD_NAV_TAKEOFF && available() && !option_is_set(QuadPlane::Option::ALLOW_FW_TAKEOFF)) {
        // treat fixed wing takeoff as VTOL takeoff
        return true;
    }
    return false;
}

/*
  return true if current mission item is a vtol land
*/
bool QuadPlane::is_vtol_land(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_LAND || id == MAV_CMD_NAV_PAYLOAD_PLACE) {
        if (landing_with_fixed_wing_spiral_approach()) {
            return plane.vtol_approach_s.approach_stage == Plane::VTOLApproach::Stage::VTOL_LANDING;
        } else {
            return true;
        }
    }
    if (id == MAV_CMD_NAV_LAND && available() && !option_is_set(QuadPlane::Option::ALLOW_FW_LAND)) {
        // treat fixed wing land as VTOL land
        return true;
    }
    return false;
}

/*
  return true if we are in a transition to fwd flight from hover
 */
bool QuadPlane::in_frwd_transition(void) const
{
    return available() && transition->active_frwd();
}

/*
  calculate current stopping distance for a quadplane in fixed wing flight
 */
float QuadPlane::stopping_distance_m(float ground_speed_squared_m) const
{
    // use v^2/(2*accel). This is only quite approximate as the drag
    // varies with pitch, but it gives something for the user to
    // control the transition distance in a reasonable way
    return ground_speed_squared_m / (2 * transition_decel_mss);
}

/*
  calculate acceleration needed to stop in the given distance given current speed
 */
float QuadPlane::accel_needed(float stop_distance, float ground_speed_squared) const
{
    return ground_speed_squared / (2 * MAX(1,stop_distance));
}

/*
  calculate current stopping distance for a quadplane in fixed wing flight
 */
float QuadPlane::stopping_distance_m(void)
{
    return stopping_distance_m(plane.ahrs.groundspeed_vector().length_squared());
}

/*
  distance below which we don't do approach, based on stopping
  distance for cruise speed
 */
float QuadPlane::transition_threshold_m(void)
{
    // 1.5 times stopping distance for cruise speed
    return 1.5 * stopping_distance_m(sq(plane.aparm.airspeed_cruise));
}

#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity

void QuadPlane::update_throttle_mix(void)
{
    // update filtered acceleration
    Vector3f accel_ef_mss = ahrs.get_accel_ef();
    accel_ef_mss.z += GRAVITY_MSS;
    throttle_mix_accel_ef_filter.apply(accel_ef_mss, plane.scheduler.get_loop_period_s());

    // transition will directly manage the mix
    if (!transition->allow_update_throttle_mix()) {
        return;
    }

    // if disarmed or landed prioritise throttle
    if (!motors->armed()) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    if (plane.control_mode->is_vtol_man_throttle()) {
        // manual throttle
        if (!is_positive(get_throttle_input()) && !air_mode_active()) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD;

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        bool accel_moving = (throttle_mix_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_NEU_ms().z >= 0.0f;

        bool use_mix_max = large_angle_request || large_angle_error || accel_moving || descent_not_demanded;

        /*
          special case for auto landing, we want a high degree of
          attitude control until LAND_FINAL
         */
        if (in_vtol_land_sequence()) {
            use_mix_max = !in_vtol_land_final();
        }

        if (use_mix_max) {
            attitude_control->set_throttle_mix_max(1.0);
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
}

/*
  see if we are in the approach phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_approach(void) const
{
    if (plane.control_mode == &plane.mode_qrtl &&
        poscontrol.get_state() <= QPOS_POSITION2) {
        return true;
    }
    if (in_vtol_auto()) {
        if (is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
            (poscontrol.get_state() == QPOS_APPROACH ||
             poscontrol.get_state() == QPOS_AIRBRAKE ||
             poscontrol.get_state() == QPOS_POSITION1 ||
             poscontrol.get_state() == QPOS_POSITION2)) {
            return true;
        }
    }
    return false;
}

/*
  see if we are in the descent phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_descent(void) const
{
    const auto state = poscontrol.get_state();
    if (plane.control_mode == &plane.mode_qrtl &&
        (state == QPOS_LAND_DESCEND || state == QPOS_LAND_FINAL || state == QPOS_LAND_ABORT)) {
        return true;
    }
    if (in_vtol_auto() && is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        (state == QPOS_LAND_DESCEND || state == QPOS_LAND_FINAL || state == QPOS_LAND_ABORT)) {
        return true;
    }
    return false;
}

/*
  see if we are in the final phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_final(void) const
{
    return in_vtol_land_descent() && poscontrol.get_state() == QPOS_LAND_FINAL;
}

/*
  see if we are in any of the phases of a VTOL landing
 */
bool QuadPlane::in_vtol_land_sequence(void) const
{
    return plane.control_mode == &plane.mode_qrtl || in_vtol_land_approach() || in_vtol_land_descent() || in_vtol_land_final();
}

/*
  see if we are in the VTOL position control phase of a landing
 */
bool QuadPlane::in_vtol_land_poscontrol(void) const
{
    if (in_vtol_auto() && is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        poscontrol.get_state() >= QPOS_POSITION1) {
        return true;
    }
    return false;
}

/*
  see if we are in the airbrake phase of a VTOL landing
 */
bool QuadPlane::in_vtol_airbrake(void) const
{
    if (plane.control_mode == &plane.mode_qrtl &&
        poscontrol.get_state() == QPOS_AIRBRAKE) {
        return true;
    }
    if (plane.control_mode == &plane.mode_auto &&
        is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        poscontrol.get_state() == QPOS_AIRBRAKE) {
        return true;
    }
    return false;
}

// return true if we should show VTOL view
bool QuadPlane::show_vtol_view() const
{
    return available() && transition->show_vtol_view() && !force_fw_control_recovery;
}

// return true if we should show VTOL view
bool SLT_Transition::show_vtol_view() const
{

    return quadplane.in_vtol_mode();
}

/*
  return the PILOT_VELZ_MAX_DN value if non zero, otherwise returns the PILOT_VELZ_MAX value.
  return is in cm/s
*/
uint16_t QuadPlane::get_pilot_velocity_z_max_dn_m() const
{
    if (is_zero(pilot_speed_z_max_dn_ms)) {
        return abs(pilot_speed_z_max_up_ms);
    }
    return abs(pilot_speed_z_max_dn_ms);
}

/*
  should we use the fixed wing attitude controllers for roll/pitch control
 */
bool QuadPlane::use_fw_attitude_controllers(void) const
{
    if (available() &&
        motors->armed() &&
        motors->get_desired_spool_state() >= AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
        !tailsitter.enabled() &&
        poscontrol.get_state() != QPOS_AIRBRAKE &&
        !force_fw_control_recovery) {

        if (in_vtol_mode()) {
            // in VTOL modes always slave fixed wing to VTOL rate control
            return false;
        }

        if (transition->use_multirotor_control_in_fwd_transition()) {
            /*
              special case for vectored yaw tiltrotors in forward
              transition, keep multicopter control until we reach
              target transition airspeed. This can result in loss of
              yaw control on some tilt-vectored airframes without
              strong VTOL yaw control
            */
            return false;
        }
    }

    return true;
}

/*
  calculate our closing velocity vector on the landing point, taking
  into account target velocity
*/
Vector2f QuadPlane::landing_closing_velocity_NE_ms()
{
    Vector2f landing_velocity_ne_m;
    if (AP_HAL::millis() - poscontrol.last_velocity_match_ms < 1000) {
        landing_velocity_ne_m = poscontrol.velocity_match_ms;
    }
    return ahrs.groundspeed_vector() - landing_velocity_ne_m;
}

/*
  calculate our desired closing velocity vector on the landing point.
*/
Vector2f QuadPlane::landing_desired_closing_velocity_NE_ms()
{
    if (poscontrol.get_state() >= QPOS_LAND_DESCEND) {
        return Vector2f(0,0);
    }
    const Vector2f diff_wp_NE_m = plane.current_loc.get_distance_NE(plane.next_WP_loc);
    float dist_m = diff_wp_NE_m.length();
    if (dist_m < 1) {
        return Vector2f(0,0);
    }

    // base target speed based on sqrt of distance
    float target_speed_ms = safe_sqrt(2 * transition_decel_mss * dist_m);

    // don't let the target speed go above landing approach speed
    const float eas2tas = plane.ahrs.get_EAS2TAS();
    float land_speed_ms = plane.aparm.airspeed_cruise;
    float tecs_land_airspeed_ms = plane.TECS_controller.get_land_airspeed();
    if (is_positive(tecs_land_airspeed_ms)) {
        land_speed_ms = tecs_land_airspeed_ms;
    } else {
        // use half way between min airspeed and cruise if
        // TECS_LAND_AIRSPEED not set
        land_speed_ms = 0.5 * (land_speed_ms + plane.aparm.airspeed_min);
    }
    target_speed_ms = MIN(target_speed_ms, eas2tas * land_speed_ms);

    Vector2f target_speed_xy = diff_wp_NE_m.normalized() * target_speed_ms;

    return target_speed_xy;
}

/*
  get target airspeed for landing, for use by TECS
*/
float QuadPlane::get_land_airspeed_ms(void)
{
    const auto qstate = poscontrol.get_state();
    if (qstate == QPOS_APPROACH ||
        plane.control_mode == &plane.mode_rtl) {
        const float cruise_speed_ms = plane.aparm.airspeed_cruise;
        // assume cruise speed, but try to do better:
        float approach_speed_ms = cruise_speed_ms;
        float tecs_land_airspeed_ms = plane.TECS_controller.get_land_airspeed();
        if (is_positive(tecs_land_airspeed_ms)) {
            approach_speed_ms = tecs_land_airspeed_ms;
        } else if (qstate == QPOS_APPROACH) {
            // default to half way between min airspeed and cruise
            // airspeed when on the approach
            approach_speed_ms = 0.5*(cruise_speed_ms + plane.aparm.airspeed_min);
        }
        const float time_to_pos1 = (plane.auto_state.wp_distance - stopping_distance_m(sq(approach_speed_ms))) / MAX(approach_speed_ms, 5);
        /*
          slow down to landing approach speed as we get closer to landing
        */
        approach_speed_ms = linear_interpolate(approach_speed_ms, cruise_speed_ms,
                                            time_to_pos1,
                                            20, 60);
        return approach_speed_ms;
    }

    if (qstate == QPOS_AIRBRAKE) {
        // during airbraking ask TECS to slow us to stall speed
        return plane.aparm.airspeed_min;
    }
    
    // calculate speed based on landing desired velocity
    Vector2f vel_ne_ms = landing_desired_closing_velocity_NE_ms();
    const Vector2f wind_ms = plane.ahrs.wind_estimate().xy();
    const float eas2tas = plane.ahrs.get_EAS2TAS();
    vel_ne_ms -= wind_ms;
    vel_ne_ms /= eas2tas;
    return vel_ne_ms.length();
}

void QuadPlane::set_desired_spool_state(AP_Motors::DesiredSpoolState state)
{
    if (motors->get_desired_spool_state() != state) {
        if (state == AP_Motors::DesiredSpoolState::SHUT_DOWN) {
            // also request zero throttle, so we avoid the slow ramp down
            motors->set_roll(0);
            motors->set_pitch(0);
            motors->set_yaw(0);
            motors->set_throttle(0);
        }
        motors->set_desired_spool_state(state);
    }
}

bool QuadPlane::air_mode_active() const
{
    if ((air_mode == AirMode::ON) || ((air_mode == AirMode::ASSISTED_FLIGHT_ONLY) && assisted_flight)) {
        return true;
    }
    return false;
}

/*
  return scaling factor for tilting rotors in forward flight throttle
  we want to scale back tilt angle for roll/pitch by throttle in forward flight
 */
float QuadPlane::FW_vector_throttle_scaling()
{
    const float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01;
    // scale relative to a fixed 0.5 mid throttle so that changes in TRIM_THROTTLE in missions don't change
    // the scaling of tilt
    const float mid_throttle = 0.5;
    return mid_throttle / constrain_float(throttle, 0.1, 1.0);
}

QuadPlane *QuadPlane::_singleton = nullptr;

bool SLT_Transition::set_FW_roll_limit(int32_t& roll_limit_cd)
{
    if (quadplane.assisted_flight && (transition_state == State::AIRSPEED_WAIT || transition_state == State::TIMER) &&
        quadplane.option_is_set(QuadPlane::Option::LEVEL_TRANSITION)) {
        // the user wants transitions to be kept level to within LEVEL_ROLL_LIMIT
        roll_limit_cd = MIN(roll_limit_cd, plane.g.level_roll_limit*100);
        return true;
    }
    return false;
}

bool SLT_Transition::allow_update_throttle_mix() const
{
    // transition is directly managing throttle mix in these cases
    return !(quadplane.assisted_flight && (transition_state == State::AIRSPEED_WAIT || transition_state == State::TIMER));
}

bool SLT_Transition::active_frwd() const
{
    // We need to be in assisted flight...
    if (!quadplane.assisted_flight) {
        return false;
    }
    // ... and a transition must be active...
    if (!((transition_state == State::AIRSPEED_WAIT) || (transition_state == State::TIMER))) {
        return false;
    }
    // ... but not executing a QPOS_AIRBRAKE maneuver during an automated landing.
    if (quadplane.in_vtol_airbrake()) {
        return false;
    }
    
    return true;
}

/*
  limit VTOL roll/pitch in POSITION1, POSITION2 and waypoint controller. This serves three roles:
   1) an expanding envelope limit on pitch to prevent sudden pitch at the start of a back transition

   2) limiting roll and pitch down to the Q_ANGLE_MAX, as the accel limits may push us beyond that for pitch up.
      This is needed as the position controller doesn't have separate limits for pitch and roll

   3) preventing us pitching up a lot when our airspeed may be low
      enough that the real airspeed may be negative, which would result
      in reversed control surfaces
 */
bool SLT_Transition::set_VTOL_roll_pitch_limit(int32_t& roll_cd, int32_t& pitch_cd)
{
    bool ret = false;
    const int16_t angle_max = quadplane.aparm.angle_max;

    /*
      we always limit roll to Q_ANGLE_MAX
     */
    int32_t new_roll_cd = constrain_int32(roll_cd, -angle_max, angle_max);
    if (new_roll_cd != roll_cd) {
        roll_cd = new_roll_cd;
        ret = true;
    }

    /*
      always limit pitch down to Q_ANGLE_MAX. We need to do this as
      the position controller accel limits may exceed this limit
     */
    if (pitch_cd < -angle_max) {
        pitch_cd = -angle_max;
        ret = true;
    }

    /*
      prevent trying to fly backwards (negative airspeed) at high
      pitch angles, which can result in a high degree of instability
      in SLT aircraft. This can happen with a tailwind in a back
      transition, where the position controller (which is unaware of
      airspeed) demands high pitch to hit the desired landing point
     */
    float airspeed;
    if (pitch_cd > angle_max &&
        plane.ahrs.airspeed_estimate(airspeed) && airspeed < 0.5 * plane.aparm.airspeed_min) {
        const float max_limit_cd = linear_interpolate(angle_max, 4500,
                                                      airspeed,
                                                      0, 0.5 * plane.aparm.airspeed_min);
        if (pitch_cd > max_limit_cd) {
            pitch_cd = max_limit_cd;
            ret = true;
        }
    }

    if (quadplane.back_trans_pitch_limit_ms <= 0) {
        // time based pitch envelope disabled
        return ret;
    }

    const uint32_t limit_time_ms = quadplane.back_trans_pitch_limit_ms;

    const uint32_t dt = AP_HAL::millis() - last_fw_mode_ms;
    if (last_fw_mode_ms == 0 || dt > limit_time_ms) {
        // we are beyond the time limit, don't apply envelope
        last_fw_mode_ms = 0;
        return ret;
    }

    // we limit pitch during initial transition
    const float max_limit_cd = linear_interpolate(MAX(last_fw_nav_pitch_cd,0), MIN(angle_max,plane.aparm.pitch_limit_max*100),
                                            dt,
                                            0, limit_time_ms);

    if (pitch_cd > max_limit_cd) {
        pitch_cd = max_limit_cd;
        return true;
    }

    /*
        limit the pitch down with an expanding envelope. This
        prevents the velocity controller demanding nose down during
        the initial slowdown if the target velocity curve is higher
        than the actual velocity curve (for a high drag
        aircraft). Nose down will cause a lot of downforce on the
        wings which will draw a lot of current and also cause the
        aircraft to lose altitude rapidly.pitch limit varies also with speed
        to prevent inability to progress to position if moving from a loiter
        to landing
    */
    const float min_limit_cd = linear_interpolate(MIN(last_fw_nav_pitch_cd,0), MAX(-angle_max,plane.aparm.pitch_limit_min*100),
                                                  dt,
                                                  0, limit_time_ms);

    if (plane.nav_pitch_cd < min_limit_cd) {
        plane.nav_pitch_cd = min_limit_cd;
        return true;
    }

    return ret;
}

/*
  remember last fixed wing pitch for pitch envelope in back transition
 */
void SLT_Transition::set_last_fw_pitch()
{
    last_fw_mode_ms = AP_HAL::millis();
    last_fw_nav_pitch_cd = plane.nav_pitch_cd;
}

void SLT_Transition::force_transition_complete()
{
    transition_state = State::DONE;
    in_forced_transition = false;
    transition_start_ms = 0;
    transition_low_airspeed_ms = 0;
    set_last_fw_pitch();

    // Keep assistance reset while not checking
    quadplane.assist.reset();
}

MAV_VTOL_STATE SLT_Transition::get_mav_vtol_state() const
{
    if (quadplane.in_vtol_mode()) {
        QuadPlane::position_control_state state = quadplane.poscontrol.get_state();
        if ((state == QuadPlane::position_control_state::QPOS_AIRBRAKE) || (state == QuadPlane::position_control_state::QPOS_POSITION1)) {
            return MAV_VTOL_STATE_TRANSITION_TO_MC;
        }
        return MAV_VTOL_STATE_MC;
    }

    switch (transition_state) {
        case State::AIRSPEED_WAIT:
        case State::TIMER:
            // we enter this state during assisted flight, not just
            // during a forward transition.
            return MAV_VTOL_STATE_TRANSITION_TO_FW;

        case State::DONE:
            return MAV_VTOL_STATE_FW;
    }

    return MAV_VTOL_STATE_UNDEFINED;
}

// Set FW roll and pitch limits and keep TECS informed
void SLT_Transition::set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd)
{
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_airbrake()) {
        // not in FW flight
        return;
    }

    if (transition_state == State::DONE) {
        // transition complete, nothing to do
        return;
    }

    if (!plane.control_mode->does_auto_throttle()) {
        // don't limit pitch when in manually controlled modes like FBWA, ACRO
        return;
    }

    float max_pitch;
    if (transition_state < State::TIMER) {
        if (plane.ahrs.groundspeed() < 3.0) {
            // until we have some ground speed limit to zero pitch
            max_pitch = 0.0;
        } else {
            max_pitch = quadplane.transition_pitch_max;
        }
    } else {
        max_pitch = (quadplane.transition_pitch_max+1.0)*2.0;
    }

    // set a single loop pitch limit in TECS
    plane.TECS_controller.set_pitch_max(max_pitch);
    plane.TECS_controller.set_pitch_min(-max_pitch);

    // ensure pitch is constrained to limit
    nav_pitch_cd = constrain_int32(nav_pitch_cd, -max_pitch*100.0, max_pitch*100.0);
}

/*
  see if we are in a VTOL takeoff
 */
bool QuadPlane::in_vtol_takeoff(void) const
{
    if (in_vtol_auto() && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
        return true;
    }
    return false;
}

// called when we change mode (for any mode, not just Q modes)
void QuadPlane::mode_enter(void)
{
    if (available()) {
        pos_control->set_lean_angle_max_cd(0);
    }
    poscontrol.correction_ne_m.zero();
    poscontrol.velocity_match_ms.zero();
    poscontrol.last_velocity_match_ms = 0;
    poscontrol.set_state(QuadPlane::QPOS_NONE);

    // Clear any pilot corrections
    poscontrol.pilot_correction_done = false;
    poscontrol.pilot_correction_active = false;
    poscontrol.target_vel_ms.zero();

    // clear guided takeoff wait on any mode change, but remember the
    // state for special behaviour
    guided_wait_takeoff_on_mode_enter = guided_wait_takeoff;
    guided_wait_takeoff = false;

    q_fwd_throttle = 0.0f;
    q_fwd_pitch_lim_cd = 100.0f * q_fwd_pitch_lim;

    force_fw_control_recovery = false;
    in_spin_recovery = false;
}

// Set attitude control yaw rate time constant to pilot input command model value
void QuadPlane::set_pilot_yaw_rate_time_constant()
{
    attitude_control->set_yaw_rate_tc(command_model_pilot.get_rate_tc());
}

// Disable attitude control yaw rate time constant
void QuadPlane::disable_yaw_rate_time_constant()
{
    attitude_control->set_yaw_rate_tc(0.0);
}

// Check if servo auto trim is allowed, only if countrol surfaces are fully in use
bool QuadPlane::allow_servo_auto_trim()
{
    if (!available()) {
        // Quadplane disabled, auto trim always allowed
        return true;
    }
    if (in_vtol_mode()) {
        // VTOL motors active in VTOL modes
        return false;
    }
    if (!in_assisted_flight()) {
        // In forward flight and VTOL motors not active
        return true;
    }
    if (tailsitter.enabled() && option_is_set(QuadPlane::Option::TAILSIT_Q_ASSIST_MOTORS_ONLY)) {
        // Tailsitter in forward flight, motors providing active stabalisation with motors only option
        // Control surfaces are running as normal with I term active, motor I term is zeroed
        return true;
    }
    // In forward flight with active VTOL motors
    return false;
}

bool QuadPlane::landing_with_fixed_wing_spiral_approach(void) const
{
    const AP_Mission::Mission_Command cmd = plane.mission.get_current_nav_cmd();

    if (cmd.id == MAV_CMD_NAV_PAYLOAD_PLACE &&
        option_is_set(QuadPlane::Option::MISSION_LAND_FW_APPROACH)) {
        return true;
    }
    
    return ((cmd.id == MAV_CMD_NAV_VTOL_LAND) &&
            (option_is_set(QuadPlane::Option::MISSION_LAND_FW_APPROACH) ||
             cmd.p1 == NAV_VTOL_LAND_OPTIONS_FW_SPIRAL_APPROACH));
}

/*
  setup scaling of roll and pitch angle P gains to match fixed wing gains

  we setup the angle P gain to match fixed wing at high speed (above
  AIRSPEED_MIN) where fixed wing surfaces are presumed to
  dominate. At lower speeds we use the multicopter angle P gains.
*/
void QuadPlane::setup_rp_fw_angle_gains(void)
{
    const float mc_angR = attitude_control->get_angle_roll_p().kP();
    const float mc_angP = attitude_control->get_angle_pitch_p().kP();
    const float fw_angR = 1.0/plane.rollController.tau();
    const float fw_angP = 1.0/plane.pitchController.tau();

    if (!is_positive(mc_angR) || !is_positive(mc_angP)) {
        // bad configuration, don't scale
        return;
    }

    float aspeed;
    if (!ahrs.airspeed_estimate(aspeed)) {
        // can't get airspeed, no scaling of VTOL angle gains
        return;
    }

    const float low_airspeed = 3.0;
    if (aspeed <= low_airspeed || plane.aparm.airspeed_min <= low_airspeed) {
        // no scaling
        return;
    }

    const float angR_scale = linear_interpolate(mc_angR, fw_angR,
                                                aspeed,
                                                low_airspeed, plane.aparm.airspeed_min) / mc_angR;
    const float angP_scale = linear_interpolate(mc_angP, fw_angP,
                                                aspeed,
                                                low_airspeed, plane.aparm.airspeed_min) / mc_angP;
    const Vector3f gain_scale{angR_scale, angP_scale, 1.0};
    attitude_control->set_angle_P_scale(gain_scale);
}

/*
  abort landing, used by scripting for payload place and ship landing abort
  will return false if not in a landing descent
 */
bool QuadPlane::abort_landing(void)
{
    if (poscontrol.get_state() == QPOS_LAND_ABORT ||
        !(plane.control_mode == &plane.mode_auto)) {
        // already aborted or not in AUTO?
        return false;
    }

    // special case for payload place with full landing
    const bool payload_place_landed =
        plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE) &&
        poscontrol.get_state() == QPOS_LAND_COMPLETE;

    if (!payload_place_landed && !in_vtol_land_descent()) {
        return false;
    }
    poscontrol.set_state(QuadPlane::QPOS_LAND_ABORT);
    return true;
}

// Should we allow stick mixing from the pilot
bool QuadPlane::allow_stick_mixing() const
{
    if (!available()) {
        // Quadplane not enabled
        return true;
    }
    // Ask transition logic
    return transition->allow_stick_mixing();
}

/*
  return true if we should disable TECS in the current flight state
  this ensures that TECS resets when we change height in a VTOL mode
 */
bool QuadPlane::should_disable_TECS() const
{
    if (in_vtol_land_descent()) {
        return true;
    }
    if (plane.control_mode == &plane.mode_guided &&
        plane.auto_state.vtol_loiter) {
        return true;
    }
    return false;
}

// Get pilot throttle input with deadzone, this will return 50% throttle in failsafe!
// This is a re-implmentation of Plane::get_throttle_input
// Ignoring the no_deadzone case means we don't need to check for valid RC
// This is handled by Plane::control_failsafe setting of control in
float QuadPlane::get_throttle_input() const
{
    float ret = plane.channel_throttle->get_control_in();
    if (plane.reversed_throttle) {
        // RC option for reverse throttle has been set
        ret = -ret;
    }
    return ret;
}

// return true if forward throttle from forward_throttle_pct() should be used
bool QuadPlane::allow_forward_throttle_in_vtol_mode() const
{
    return in_vtol_mode() && motors->armed() && (motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::SHUT_DOWN);
}

void QuadPlane::Log_Write_AttRate()
{
    attitude_control->Write_ANG();
    attitude_control->Write_Rate(*pos_control);

}

#endif  // HAL_QUADPLANE_ENABLED
