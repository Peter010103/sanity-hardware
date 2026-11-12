void actualPidTrackSanityOutput(const float targetSum[3])
{
    // External-reset tracking keeps the standby PID sum at the actual applied
    // motor differential, without pausing D/FF filters. Saturation is explicit.
    for (unsigned a = 0; a < 3; ++a) {
        pidData[a].I = constrainf(targetSum[a] - pidData[a].P - pidData[a].D - pidData[a].F,
            -pidRuntime.itermLimit, pidRuntime.itermLimit);
        pidData[a].Sum = pidData[a].P + pidData[a].I + pidData[a].D + pidData[a].F;
    }
}

uint16_t dshotConvertToExternal(float motorValue)
{
    float externalValue;

    if (featureIsEnabled(FEATURE_3D)) {
        if (motorValue == DSHOT_CMD_MOTOR_STOP || motorValue < DSHOT_MIN_THROTTLE) {
            externalValue = PWM_RANGE_MIDDLE;
        } else if (motorValue <= DSHOT_3D_FORWARD_MIN_THROTTLE - 1) {
            externalValue = scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, PWM_RANGE_MIDDLE - 1, PWM_RANGE_MIN);
        } else {
            externalValue = scaleRangef(motorValue, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIDDLE + 1, PWM_RANGE_MAX);
        }
    } else {
        externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
    }

    return lrintf(externalValue);
}

float dshotConvertFromExternal(uint16_t externalValue)
{
    float motorValue;

    externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);

    if (featureIsEnabled(FEATURE_3D)) {
        if (externalValue == PWM_RANGE_MIDDLE) {
            motorValue = DSHOT_CMD_MOTOR_STOP;
        } else if (externalValue < PWM_RANGE_MIDDLE) {
            motorValue = scaleRangef(externalValue, PWM_RANGE_MIN, PWM_RANGE_MIDDLE - 1, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, DSHOT_MIN_THROTTLE);
        } else {
            motorValue = scaleRangef(externalValue, PWM_RANGE_MIDDLE + 1, PWM_RANGE_MAX, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
        }
    } else {
        motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_CMD_MOTOR_STOP : scaleRangef(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    }

    return motorValue;
}

float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo)
{
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}
