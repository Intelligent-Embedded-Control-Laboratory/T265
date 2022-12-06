/* 2022-12-05 */

#pragma once



namespace Attitude
{
	void quaternion_inverse(float(&Q)[4])
	{
		Q[1] = -Q[1];
		Q[2] = -Q[2];
		Q[3] = -Q[3];
	}

	void quaternion_normalize(float(&Q)[4])
	{
		float quat_length = sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
		Q[0] /= quat_length;
		Q[1] /= quat_length;
		Q[2] /= quat_length;
		Q[3] /= quat_length;
	}

	void quaternion_multiply(float(&P)[4], float(&Q)[4], float(&R)[4])
	{
		float& p0 = P[0];
		float& p1 = P[1];
		float& p2 = P[2];
		float& p3 = P[3];

		float& q0 = Q[0];
		float& q1 = Q[1];
		float& q2 = Q[2];
		float& q3 = Q[3];

		R[0] = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
		R[1] = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
		R[2] = p0 * q2 + p2 * q0 - p1 * q3 + p3 * q1;
		R[3] = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;

		quaternion_normalize(R);
	}

	void quaternion_to_euler(float(&quat)[4], float(&euler321)[3])
	{
		float& q0 = quat[0];
		float& q1 = quat[1];
		float& q2 = quat[2];
		float& q3 = quat[3];

		float q0q1 = q0 * q1;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q1q1 = q1 * q1;
		float q0q2 = q0 * q2;
		float q1q3 = q1 * q3;
		float q0q3 = q0 * q3;
		float q1q2 = q1 * q2;
		float q3q3 = q3 * q3;

		euler321[0] = atan2f(2.0F * (q0q1 + q2q3), 1.0F - 2.0F * (q1q1 + q2q2));
		euler321[1] = asinf(2.0F * (q0q2 - q1q3));
		euler321[2] = atan2f(2.0F * (q0q3 + q1q2), 1.0F - 2.0F * (q2q2 + q3q3));
	}
}