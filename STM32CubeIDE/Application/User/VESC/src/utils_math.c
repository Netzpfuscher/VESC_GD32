/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "utils_math.h"
//#include "hal.h"
#include "app.h"

#include <string.h>
#include <stdlib.h>

/*
 * Map angle from 0 to 1 in the range min to max. If angle is
 * outside of the range it will be less truncated to the closest
 * angle. Angle units: Degrees
 */
float utils_map_angle(float angle, float min, float max) {
	if (max == min) {
		return -1;
	}

	float range_pos = max - min;
	utils_norm_angle(&range_pos);
	float range_neg = min - max;
	utils_norm_angle(&range_neg);
	float margin = range_neg / 2.0;

	angle -= min;
	utils_norm_angle(&angle);
	if (angle > (360 - margin)) {
		angle -= 360.0;
	}

	float res = angle / range_pos;
	utils_truncate_number(&res, 0.0, 1.0);

	return res;
}



/**
 * Truncate absolute values less than tres to zero. The value
 * tres will be mapped to 0 and the value max to max.
 */
void utils_deadband(float *value, float tres, float max) {
	if (fabsf(*value) < tres) {
		*value = 0.0;
	} else {
		float k = max / (max - tres);
		if (*value > 0.0) {
			*value = k * *value + max * (1.0 - k);
		} else {
			*value = -(k * -*value + max * (1.0 - k));
		}
	}
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -180.0) difference += 2.0 * 180.0;
	while (difference > 180.0) difference -= 2.0 * 180.0;
	return difference;
}

/**
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param angle1
 * The first angle in radians
 * @param angle2
 * The second angle in radians
 * @return
 * The difference between the angles in radians
 */
float utils_angle_difference_rad(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -M_PI) difference += 2.0 * M_PI;
	while (difference > M_PI) difference -= 2.0 * M_PI;
	return difference;
}

/**
 * Takes the average of a number of angles.
 *
 * @param angles
 * The angles in radians.
 *
 * @param angles_num
 * The number of angles.
 *
 * @param weights
 * The weight of the summarized angles
 *
 * @return
 * The average angle.
 */
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num) {
	float s_sum = 0.0;
	float c_sum = 0.0;

	for (int i = 0; i < angles_num; i++) {
		float s, c;
		utils_fast_sincos_better(angles[i], &s, &c);
		s_sum += s * weights[i];
		c_sum += c * weights[i];
	}

	return utils_fast_atan2(s_sum, c_sum);
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
float utils_middle_of_3(float a, float b, float c) {
	float middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
int utils_middle_of_3_int(int a, int b, int c) {
	int middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * Fast atan2
 *
 * See http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 *
 * @param y
 * y
 *
 * @param x
 * x
 *
 * @return
 * The angle in radians
 */
float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-20; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
	}

	UTILS_NAN_ZERO(angle);

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	// compute sine
	if (angle < 0.0) {
		*sin = 1.27323954 * angle + 0.405284735 * angle * angle;
	} else {
		*sin = 1.27323954 * angle - 0.405284735 * angle * angle;
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5 * M_PI;

	if (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	if (angle < 0.0) {
		*cos = 1.27323954 * angle + 0.405284735 * angle * angle;
	} else {
		*cos = 1.27323954 * angle - 0.405284735 * angle * angle;
	}
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	//compute sine
	if (angle < 0.0) {
		*sin = 1.27323954 * angle + 0.405284735 * angle * angle;

		if (*sin < 0.0) {
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225 * (*sin * *sin - *sin) + *sin;
		}
	} else {
		*sin = 1.27323954 * angle - 0.405284735 * angle * angle;

		if (*sin < 0.0) {
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225 * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5 * M_PI;
	if (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	if (angle < 0.0) {
		*cos = 1.27323954 * angle + 0.405284735 * angle * angle;

		if (*cos < 0.0) {
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
		}
	} else {
		*cos = 1.27323954 * angle - 0.405284735 * angle * angle;

		if (*cos < 0.0) {
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
		}
	}
}

//#define SIN_COS_TABLE {\
//0,			 0.006135885, 0.012271538, 0.018406730, 0.024541229, 0.030674803, 0.036807223, 0.042938257,\
//0.049067674, 0.055195244, 0.061320736, 0.067443920, 0.073564564, 0.079682438, 0.085797312, 0.091908956,\
//0.098017140, 0.104121634, 0.110222207, 0.116318631, 0.122410675, 0.128498111, 0.134580709, 0.140658239,\
//0.146730474, 0.152797185, 0.158858143, 0.164913120, 0.170961889, 0.177004220, 0.183039888, 0.189068664,\
//0.195090322, 0.201104635, 0.207111376, 0.213110320, 0.219101240, 0.225083911, 0.231058108, 0.237023606,\
//0.242980180, 0.248927606, 0.254865660, 0.260794118, 0.266712757, 0.272621355, 0.278519689, 0.284407537,\
//0.290284677, 0.296150888, 0.302005949, 0.307849640, 0.313681740, 0.319502031, 0.325310292, 0.331106306,\
//0.336889853, 0.342660717, 0.348418680, 0.354163525, 0.359895037, 0.365612998, 0.371317194, 0.377007410,\
//0.382683432, 0.388345047, 0.393992040, 0.399624200, 0.405241314, 0.410843171, 0.416429560, 0.422000271,\
//0.427555093, 0.433093819, 0.438616239, 0.444122145, 0.449611330, 0.455083587, 0.460538711, 0.465976496,\
//0.471396737, 0.476799230, 0.482183772, 0.487550160, 0.492898192, 0.498227667, 0.503538384, 0.508830143,\
//0.514102744, 0.519355990, 0.524589683, 0.529803625, 0.534997620, 0.540171473, 0.545324988, 0.550457973,\
//0.555570233, 0.560661576, 0.565731811, 0.570780746, 0.575808191, 0.580813958, 0.585797857, 0.590759702,\
//0.595699304, 0.600616479, 0.605511041, 0.610382806, 0.615231591, 0.620057212, 0.624859488, 0.629638239,\
//0.634393284, 0.639124445, 0.643831543, 0.648514401, 0.653172843, 0.657806693, 0.662415778, 0.666999922,\
//0.671558955, 0.676092704, 0.680600998, 0.685083668, 0.689540545, 0.693971461, 0.698376249, 0.702754744,\
//0.707106781, 0.711432196, 0.715730825, 0.720002508, 0.724247083, 0.728464390, 0.732654272, 0.736816569,\
//0.740951125, 0.745057785, 0.749136395, 0.753186799, 0.757208847, 0.761202385, 0.765167266, 0.769103338,\
//0.773010453, 0.776888466, 0.780737229, 0.784556597, 0.788346428, 0.792106577, 0.795836905, 0.799537269,\
//0.803207531, 0.806847554, 0.810457198, 0.814036330, 0.817584813, 0.821102515, 0.824589303, 0.828045045,\
//0.831469612, 0.834862875, 0.838224706, 0.841554977, 0.844853565, 0.848120345, 0.851355193, 0.854557988,\
//0.857728610, 0.860866939, 0.863972856, 0.867046246, 0.870086991, 0.873094978, 0.876070094, 0.879012226,\
//0.881921264, 0.884797098, 0.887639620, 0.890448723, 0.893224301, 0.895966250, 0.898674466, 0.901348847,\
//0.903989293, 0.906595705, 0.909167983, 0.911706032, 0.914209756, 0.916679060, 0.919113852, 0.921514039,\
//0.923879533, 0.926210242, 0.928506080, 0.930766961, 0.932992799, 0.935183510, 0.937339012, 0.939459224,\
//0.941544065, 0.943593458, 0.945607325, 0.947585591, 0.949528181, 0.951435021, 0.953306040, 0.955141168,\
//0.956940336, 0.958703475, 0.960430519, 0.962121404, 0.963776066, 0.965394442, 0.966976471, 0.968522094,\
//0.970031253, 0.971503891, 0.972939952, 0.974339383, 0.975702130, 0.977028143, 0.978317371, 0.979569766,\
//0.980785280, 0.981963869, 0.983105487, 0.984210092, 0.985277642, 0.986308097, 0.987301418, 0.988257568,\
//0.989176510, 0.990058210, 0.990902635, 0.991709754, 0.992479535, 0.993211949, 0.993906970, 0.994564571,\
//0.995184727, 0.995767414, 0.996312612, 0.996820299, 0.997290457, 0.997723067, 0.998118113, 0.998475581,\
//0.998795456, 0.999077728, 0.999322385, 0.999529418, 0.999698819, 0.999830582, 0.999924702, 0.999981175}
//
//const float hSin_Cos_Table[256] = SIN_COS_TABLE;
//
//#define SIN_MASK        0x0300u
//#define U0_90           0x0200u
//#define U90_180         0x0300u
//#define U180_270        0x0000u
//#define U270_360        0x0100u
//
//
//void utils_fast_sincos_better(float angle, float *sin, float *cos)
//{
//
//	int16_t angle_int = (32768.0 / M_PI) * angle;
//
//  int32_t shindex;
//  uint16_t uhindex;
//
//  /* 10 bit index computation  */
//  shindex = ( ( int32_t )32768 + ( int32_t )angle_int );
//  uhindex = ( uint16_t )shindex;
//  uhindex /= ( uint16_t )64;
//
//  switch ( ( uint16_t )( uhindex ) & SIN_MASK )
//  {
//    case U0_90:
//      *sin = hSin_Cos_Table[( uint8_t )( uhindex )];
//      *cos = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
//      break;
//
//    case U90_180:
//      *sin = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
//      *cos = -hSin_Cos_Table[( uint8_t )( uhindex )];
//      break;
//
//    case U180_270:
//      *sin = -hSin_Cos_Table[( uint8_t )( uhindex )];
//      *cos = -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
//      break;
//
//    case U270_360:
//      *sin =  -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
//      *cos =  hSin_Cos_Table[( uint8_t )( uhindex )];
//      break;
//    default:
//      break;
//  }
//}

/**
 * Calculate the values with the lowest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the lowest magnitude.
 */
float utils_min_abs(float va, float vb) {
	float res;
	if (fabsf(va) < fabsf(vb)) {
		res = va;
	} else {
		res = vb;
	}

	return res;
}

/**
 * Calculate the values with the highest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the highest magnitude.
 */
float utils_max_abs(float va, float vb) {
	float res;
	if (fabsf(va) > fabsf(vb)) {
		res = va;
	} else {
		res = vb;
	}

	return res;
}

/**
 * Create string representation of the binary content of a byte
 *
 * @param x
 * The byte.
 *
 * @param b
 * Array to store the string representation in.
 */
void utils_byte_to_binary(int x, char *b) {
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1) {
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode) {
	float ret = 0.0;
	
	if (val < -1.0) {
		val = -1.0;
	}

	if (val > 1.0) {
		val = 1.0;
	}
	
	float val_a = fabsf(val);

	float curve;
	if (val >= 0.0) {
		curve = curve_acc;
	} else {
		curve = curve_brake;
	}

	// See
	// http://math.stackexchange.com/questions/297768/how-would-i-create-a-exponential-ramp-function-from-0-0-to-1-1-with-a-single-val
	if (mode == 0) { // Exponential
		if (curve >= 0.0) {
			ret = 1.0 - powf(1.0 - val_a, 1.0 + curve);
		} else {
			ret = powf(val_a, 1.0 - curve);
		}
	} else if (mode == 1) { // Natural
		if (fabsf(curve) < 1e-10) {
			ret = val_a;
		} else {
			if (curve >= 0.0) {
				ret = 1.0 - ((expf(curve * (1.0 - val_a)) - 1.0) / (expf(curve) - 1.0));
			} else {
				ret = (expf(-curve * val_a) - 1.0) / (expf(-curve) - 1.0);
			}
		}
	} else if (mode == 2) { // Polynomial
		if (curve >= 0.0) {
			ret = 1.0 - ((1.0 - val_a) / (1.0 + curve * val_a));
		} else {
			ret = val_a / (1.0 - curve * (1.0 - val_a));
		}
	} else { // Linear
		ret = val_a;
	}

	if (val < 0.0) {
		ret = -ret;
	}

	return ret;
}

uint32_t utils_crc32c(uint8_t *data, uint32_t len) {
	uint32_t crc = 0xFFFFFFFF;

	for (uint32_t i = 0; i < len;i++) {
		uint32_t byte = data[i];
		crc = crc ^ byte;

		for (int j = 7;j >= 0;j--) {
			uint32_t mask = -(crc & 1);
			crc = (crc >> 1) ^ (0x82F63B78 & mask);
		}
	}

	return ~crc;
}

// Yes, this is only the average...
void utils_fft32_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;

	for (int i = 0;i < 32;i++) {
		*real += real_in[i];
	}

	*real /= 32.0;
}

void utils_fft32_bin1(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;
	for (int i = 0;i < 32;i++) {
		*real += real_in[i] * utils_tab_cos_32_1[i];
		*imag -= real_in[i] * utils_tab_sin_32_1[i];
	}
	*real /= 32.0;
	*imag /= 32.0;
}

void utils_fft32_bin2(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;
	for (int i = 0;i < 32;i++) {
		*real += real_in[i] * utils_tab_cos_32_2[i];
		*imag -= real_in[i] * utils_tab_sin_32_2[i];
	}
	*real /= 32.0;
	*imag /= 32.0;
}

void utils_fft16_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;

	for (int i = 0;i < 16;i++) {
		*real += real_in[i];
	}

	*real /= 16.0;
}

void utils_fft16_bin1(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;
	for (int i = 0;i < 16;i++) {
		*real += real_in[i] * utils_tab_cos_32_1[2 * i];
		*imag -= real_in[i] * utils_tab_sin_32_1[2 * i];
	}
	*real /= 16.0;
	*imag /= 16.0;
}

void utils_fft16_bin2(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;
	for (int i = 0;i < 16;i++) {
		*real += real_in[i] * utils_tab_cos_32_2[2 * i];
		*imag -= real_in[i] * utils_tab_sin_32_2[2 * i];
	}
	*real /= 16.0;
	*imag /= 16.0;
}

void utils_fft8_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;

	for (int i = 0;i < 8;i++) {
		*real += real_in[i];
	}

	*real /= 8.0;
}

void utils_fft8_bin1(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;
	for (int i = 0;i < 8;i++) {
		*real += real_in[i] * utils_tab_cos_32_1[4 * i];
		*imag -= real_in[i] * utils_tab_sin_32_1[4 * i];
	}
	*real /= 8.0;
	*imag /= 8.0;
}

void utils_fft8_bin2(float *real_in, float *real, float *imag) {
	*real = 0.0;
	*imag = 0.0;
	for (int i = 0;i < 8;i++) {
		*real += real_in[i] * utils_tab_cos_32_2[4 * i];
		*imag -= real_in[i] * utils_tab_sin_32_2[4 * i];
	}
	*real /= 8.0;
	*imag /= 8.0;
}

// A mapping of a samsung 30q cell for % remaining capacity vs. voltage from
// 4.2 to 3.2, note that the you lose 15% of the 3Ah rated capacity in this range
float utils_batt_liion_norm_v_to_capacity(float norm_v) {
	// constants for polynomial fit of lithium ion battery
	const float li_p[] = {
						  -2.979767, 5.487810, -3.501286, 1.675683, 0.317147};
	utils_truncate_number(&norm_v,0.0,1.0);
	float v2 = norm_v*norm_v;
	float v3 = v2*norm_v;
	float v4 = v3*norm_v;
	float v5 = v4*norm_v;
	float capacity = li_p[0] * v5 + li_p[1] * v4 + li_p[2] * v3 +
			li_p[3] * v2 + li_p[4] * norm_v;
	return capacity;
}

static int uint16_cmp_func (const void *a, const void *b) {
	return (*(uint16_t*)a - *(uint16_t*)b);
}

uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
		unsigned int *buffer_index, unsigned int filter_len, uint16_t sample) {
	buffer[(*buffer_index)++] = sample;
	*buffer_index %= filter_len;
	uint16_t buffer_sorted[filter_len]; // Assume we have enough stack space
	memcpy(buffer_sorted, buffer, sizeof(uint16_t) * filter_len);
	qsort(buffer_sorted, filter_len, sizeof(uint16_t), uint16_cmp_func);
	return buffer_sorted[filter_len / 2];
}

void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse) {
	float s1, c1, s2, c2, s3, c3;

	if (rotation[2] != 0.0) {
		s1 = sinf(rotation[2]);
		c1 = cosf(rotation[2]);
	} else {
		s1 = 0.0;
		c1 = 1.0;
	}

	if (rotation[1] != 0.0) {
		s2 = sinf(rotation[1]);
		c2 = cosf(rotation[1]);
	} else {
		s2 = 0.0;
		c2 = 1.0;
	}

	if (rotation[0] != 0.0) {
		s3 = sinf(rotation[0]);
		c3 = cosf(rotation[0]);
	} else {
		s3 = 0.0;
		c3 = 1.0;
	}

	float m11 = c1 * c2;	float m12 = c1 * s2 * s3 - c3 * s1;	float m13 = s1 * s3 + c1 * c3 * s2;
	float m21 = c2 * s1;	float m22 = c1 * c3 + s1 * s2 * s3;	float m23 = c3 * s1 * s2 - c1 * s3;
	float m31 = -s2; 		float m32 = c2 * s3;				float m33 = c2 * c3;

	if (reverse) {
		output[0] = input[0] * m11 + input[1] * m21 + input[2] * m31;
		output[1] = input[0] * m12 + input[1] * m22 + input[2] * m32;
		output[2] = input[0] * m13 + input[1] * m23 + input[2] * m33;
	} else {
		output[0] = input[0] * m11 + input[1] * m12 + input[2] * m13;
		output[1] = input[0] * m21 + input[1] * m22 + input[2] * m23;
		output[2] = input[0] * m31 + input[1] * m32 + input[2] * m33;
	}
}

const float utils_tab_sin_32_1[] = {
	0.000000, 0.195090, 0.382683, 0.555570, 0.707107, 0.831470, 0.923880, 0.980785,
	1.000000, 0.980785, 0.923880, 0.831470, 0.707107, 0.555570, 0.382683, 0.195090,
	0.000000, -0.195090, -0.382683, -0.555570, -0.707107, -0.831470, -0.923880, -0.980785,
	-1.000000, -0.980785, -0.923880, -0.831470, -0.707107, -0.555570, -0.382683, -0.195090};

const float utils_tab_sin_32_2[] = {
	0.000000, 0.382683, 0.707107, 0.923880, 1.000000, 0.923880, 0.707107, 0.382683,
	0.000000, -0.382683, -0.707107, -0.923880, -1.000000, -0.923880, -0.707107, -0.382683,
	-0.000000, 0.382683, 0.707107, 0.923880, 1.000000, 0.923880, 0.707107, 0.382683,
	0.000000, -0.382683, -0.707107, -0.923880, -1.000000, -0.923880, -0.707107, -0.382683};

const float utils_tab_cos_32_1[] = {
	1.000000, 0.980785, 0.923880, 0.831470, 0.707107, 0.555570, 0.382683, 0.195090,
	0.000000, -0.195090, -0.382683, -0.555570, -0.707107, -0.831470, -0.923880, -0.980785,
	-1.000000, -0.980785, -0.923880, -0.831470, -0.707107, -0.555570, -0.382683, -0.195090,
	-0.000000, 0.195090, 0.382683, 0.555570, 0.707107, 0.831470, 0.923880, 0.980785};

const float utils_tab_cos_32_2[] = {
	1.000000, 0.923880, 0.707107, 0.382683, 0.000000, -0.382683, -0.707107, -0.923880,
	-1.000000, -0.923880, -0.707107, -0.382683, -0.000000, 0.382683, 0.707107, 0.923880,
	1.000000, 0.923880, 0.707107, 0.382683, 0.000000, -0.382683, -0.707107, -0.923880,
	-1.000000, -0.923880, -0.707107, -0.382683, -0.000000, 0.382683, 0.707107, 0.923880};
