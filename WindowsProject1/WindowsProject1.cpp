#include <d3d12.h>
#include <dxgi1_6.h>
#include <iostream>
#include <chrono>
#include <directxmath.h>


const float PI = 3.1415926535897932;
const float D2R = PI / 180.0f;

const float epsilon = 0.0001f;

const int maxSteps = 1000000;

const DirectX::XMFLOAT3 right_source = { 1.0f, 0.0f, 0.0f };
const DirectX::XMFLOAT3 up_source = { 0.0f, 1.0f, 0.0f };
const DirectX::XMFLOAT3 forward_source = { 0.0f, 0.0f, 1.0f };

const DirectX::XMVECTOR right = DirectX::XMLoadFloat3(&right_source);
const DirectX::XMVECTOR up = DirectX::XMLoadFloat3(&up_source);
const DirectX::XMVECTOR forward = DirectX::XMLoadFloat3(&forward_source);

DirectX::XMVECTOR warmUp[maxSteps];
DirectX::XMVECTOR storeBenchmark1[maxSteps];
DirectX::XMVECTOR storeBenchmark2[maxSteps];

const DirectX::XMVECTOR modelDefaultLookDirection = forward;
const float modelDefaultRotationAroundDefaultLookDirection = 0.0f * D2R;

const float angleDeviationToleranceD = 4.0f;
const float angleDeviationTolerance = angleDeviationToleranceD * D2R;

float deviationDirections[maxSteps];
int countDeviationDirections = 0;
int numFails = 0;
int numFailsSlerp = 0;
int numFailsAxisAngle = 0;
float axisDirectionDeviationSlerp[maxSteps];
float axisLengthDeviationSlerp[maxSteps];
float axisDirectionDeviationAxisAngle[maxSteps];
float axisLengthDeviationAxisAngle[maxSteps];
int countAxisDirectionDeviationSlerp = 0;
int countAxisLengthDeviationSlerp = 0;
int countAxisDirectionDeviationAxisAngle = 0;
int countAxisLengthDeviationAxisAngle = 0;

int main() {
	// application: source for new target orientation

	DirectX::XMFLOAT3 targetDirection = { -1.0f, 1.0f, -1.0f };			// here some value
	float targetRotationAroundTargetDirection = 90.0f * D2R;


	// storage of orientation as direction and twist around that
	// this is a theoretic mimimum
	// for a rotation the result from the former rotation is required
	// in hardware implementations such results might not be present because they might accumulate the rotations and perform their accumulation on the original source orientation
	// reading an orientation and performing a rotation is necessary in both cases thus hardware has no need to "write back" a rotation result when they can just use the original
	// but "writing back" is not neccessary: in a ping-pong architecture the result of a rotation becomes the source for the next rotation and the result of that is placed in place of the source for the former
	// calculating and having a result is neccessary in both cases so the result can just as well be kept which has no overhead in a ping-pong architecture, it has "write back" overhead in a one-way architecture where results are transient and only used to rasterize into a current frame
	// in a one-way architecture overhead of accumulation of rotation operations should be lower than overhead of "write-back"
	// accumulation of quaternions in particular is without overhead in space-domain, overhead in time-domain is fixed rather a single base-10 digit number of operations so almost as low as possible when something is at all
	// accumulated rotation quaternions are a quaternion of the same type again
	// rotating every vertice is not neccessary: only 3 base vectors need to be rotated, every new location of a vertice is simply the product of the matrix of the rotated base vectors with the original location of the vertice
	// 3 rotated base vectors can be achieved with 2 rotations, the last rotated base vector is the cross product of the first two rotated base vectors
	// better even: rotation is not neccessary at all, every new orientation when normalised already is the "rotated base vectors".

	const DirectX::XMVECTOR modelOrientationDirection = modelDefaultLookDirection;
	const float modelOrientationTwist = modelDefaultRotationAroundDefaultLookDirection;


	float aO = 0.0f;	// ANTI-OPTIMISER value


	// BASELINE: measure duration of memory transfer (and possibly warm up cache(s) or RAM)

	DirectX::XMVECTOR zero = { 0.0f, 0.0f, 0.0f, 0.0f };

	// have an array fill before storeBenchmark1 too

	for (int i = 1; i <= maxSteps; ++i) {
		warmUp[i - 1] = zero;
	}

	auto now = std::chrono::high_resolution_clock::now();

	for (int i = 1; i <= maxSteps; ++i) {
		storeBenchmark1[i-1] = zero;
	}

	auto durationA = std::chrono::high_resolution_clock::now() - now;

	now = std::chrono::high_resolution_clock::now();

	for (int i = 1; i <= maxSteps; ++i) {
		storeBenchmark2[i-1] = zero;
	}

	auto durationB = std::chrono::high_resolution_clock::now() - now;


	// ANTI-OPTIMISER

	/*
		I observed having this code here results in durations of benchmark 2 which are up to around 100 x longer.
		If anything, having this code here should prevent an optimisation away of the baseline and as such a baseline should then be present.
		But a present baseline would reduce the duration delivered for benchmark 2 instead of increasing it because the baseline is subtracted.
		And the presence of this code should have no impact on the runtime of benchmark 2 itself.
		Is this a compiler (optimisation) bug ?
		In release builds when not having this code here performance is like in Debug builds where the presence of this code here has no impact on performance.
		I don't observe this anymore hence commented-out goto.
	*/
	//goto pass;		// prevents issue mentioned in comment above from occurring
	for (int i = 1; i <= maxSteps; ++i) {
		aO += DirectX::XMVectorGetW(warmUp[i - 1]);
	}
	for (int i = 1; i <= maxSteps; ++i) {
		aO += DirectX::XMVectorGetW(storeBenchmark1[i - 1]);
	}
	for (int i = 1; i <= maxSteps; ++i) {
		aO += DirectX::XMVectorGetW(storeBenchmark2[i - 1]);
	}
	pass:


	// PREPARATION

	DirectX::XMVECTOR targetModelOrientationDirection = DirectX::XMVector3Normalize(XMLoadFloat3(&targetDirection));

	// current orientation is default orientation and assume is already storerd in a quaternion, thus not part of duration measurement, when using quaternions slerp
	DirectX::XMVECTOR defaultModelOrientation = DirectX::XMQuaternionIdentity();


	// BENCHMARK 1: quaternions and slerp

	now = std::chrono::high_resolution_clock::now();

	// new orientation, eg from looking into a certain direction, must be converted to quaternion first
	// DX(12) Math only offers XMQuaternionRotationRollPitchYaw as seemingly suitable
	// this will result in a quaternion which when applied to the orientation the identity quaternion corresponds to produces the new orientation
	// this is independent of the current orientation being the one the identity quaternion corresponds to here
	DirectX::XMVECTOR objectUp;
	// unwind roll, pitch, yaw around global axis from new orientation
	// unwind yaw, yaw is around global axis up, bring new orientation orthogonal to pitch axis = right, have it face forwardly
	float yaw = 0.0f;
	float dot = DirectX::XMVectorGetW(DirectX::XMVector3Dot(up, targetModelOrientationDirection));
	float pitch = copysignf(1.0f, dot) * 90.0f * D2R;
	// is new orientation (anti)parallel up ie yaw has no effect ?
	if (1.0f - abs(dot) > epsilon) {
		// project new orientation to xz plane
		DirectX::XMVECTOR projectedTargetOrientation = DirectX::XMVector3Cross(DirectX::XMVector3Cross(up, targetModelOrientationDirection), up);
		// yaw is angle between that and forward
		yaw = DirectX::XMVectorGetW(DirectX::XMVector3AngleBetweenVectors(forward, projectedTargetOrientation)) * copysignf(1.0f, DirectX::XMVectorGetW(DirectX::XMVector3Dot(right, targetModelOrientationDirection)));
		// unwind pitch, pitch is around global axis right, this is angle between new orientation and xz plane
		pitch = DirectX::XMVectorGetW(DirectX::XMVector3AngleBetweenVectors(projectedTargetOrientation, targetModelOrientationDirection)) * -copysignf(1.0f, dot);
		// now roll does not do anything to the new orientation, it has been brought back to forward, but apply yaw and pitch to vector corresponding to targetRotationAroundTargetDirection to see how much that rolls around forward
		// twist = 0 for default orientation is up
		// targetRotationAroundTargetDirection = 0 is "object up" of new orientation of object, it corresponds to the following vector
		objectUp = DirectX::XMVector3Cross(targetModelOrientationDirection, DirectX::XMVector3Cross(up, targetModelOrientationDirection));
	}
	else {
		// objectUp is ambigious in this case
		// set -forward from xz plane, this is about benchmarking slerp vs angleAxis, if starting from this applications current starting orientation, -forward would be object up for twist = 0 at new orientation direction = up
		float sign = -copysignf(1.0f, dot);
		objectUp = DirectX::XMVectorMultiply({ sign, sign, sign, sign }, forward);
	}
	// actual twist of new orientation is targetRotationAroundTargetDirection
	// rotate objectUp accordingly
	DirectX::XMVECTOR twistActual = DirectX::XMVector3Rotate(objectUp, DirectX::XMQuaternionRotationAxis(targetModelOrientationDirection, targetRotationAroundTargetDirection));
	// now perform unwinding of yaw and pitch on twistActual
	DirectX::XMVECTOR twist = DirectX::XMVector3Rotate(DirectX::XMVector3Rotate(twistActual, DirectX::XMQuaternionRotationAxis(up, -yaw)), DirectX::XMQuaternionRotationAxis(right, -pitch));
	// now determine roll as angle between twist vector and up
	float roll = DirectX::XMVectorGetW(DirectX::XMVector3AngleBetweenVectors(up, twist)) * -copysignf(1.0f, DirectX::XMVectorGetW(DirectX::XMVector3Dot(right, twist)));

	// something might be wrong here, current input values produce an average deviation of only around 1.5 ° but at least one other input values produce an average deviation of around 30 °

	DirectX::XMVECTOR targetModelOrientationRotation = DirectX::XMQuaternionRotationRollPitchYaw(pitch, yaw, roll);

	float f_maxSteps = static_cast<float>(maxSteps);

	for (int i = 1; i <= maxSteps; ++i) {
		storeBenchmark1[i-1] = DirectX::XMQuaternionSlerp(defaultModelOrientation, targetModelOrientationRotation, static_cast<float>(i) / f_maxSteps);

		// in application now quaternion can be used to transform vector (eg. model's vertice)
		// vectorTransformed = vector + 2 * cross(quaternion.xyz, cross(quaternion.xyz, vector) + quaternion.w * vector)
		// or simply XMVector3Rotate
	}

	auto duration1 = std::chrono::high_resolution_clock::now() - now;


	// BENCHMARK 2: orientation stored as axis, angle and interpolate by axisAngle (rotation)

	now = std::chrono::high_resolution_clock::now();

	DirectX::XMVECTOR axisRotation = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(modelOrientationDirection, targetModelOrientationDirection));
	DirectX::XMVECTOR angleRotationMax = DirectX::XMVector3AngleBetweenNormals(modelOrientationDirection, targetModelOrientationDirection);

	float angleOrientationDelta = targetRotationAroundTargetDirection - modelOrientationTwist;
	if (angleOrientationDelta > PI) {
		angleOrientationDelta -= 2.0f * PI;
	}

	DirectX::XMVECTOR modelOrientationDirectionAlongAxisRotation = DirectX::XMVectorMultiply(DirectX::XMVector3Dot(modelOrientationDirection, axisRotation), axisRotation);
	DirectX::XMVECTOR rotationPlaneBasis1 = DirectX::XMVectorSubtract(modelOrientationDirection, modelOrientationDirectionAlongAxisRotation);		// this has a length of something
	DirectX::XMVECTOR rotationPlaneBasis2 = DirectX::XMVector3Cross(axisRotation, rotationPlaneBasis1);		// this has the same length of something, rotationPlaneBasis1 is orthogonal axisRotation

	float f_maxSteps2 = static_cast<float>(maxSteps);
	DirectX::XMVECTOR v_maxSteps = { f_maxSteps2, f_maxSteps2, f_maxSteps2, f_maxSteps2 };

	for (int i = 1; i <= maxSteps; ++i) {
		float f_i = static_cast<float>(i);
		DirectX::XMVECTOR angle = DirectX::XMVectorDivide(DirectX::XMVectorMultiply({ f_i, f_i, f_i, f_i }, angleRotationMax), v_maxSteps);
		DirectX::XMVECTOR orientationDirectionRotated = DirectX::XMVectorAdd(modelOrientationDirectionAlongAxisRotation, DirectX::XMVectorAdd(DirectX::XMVectorMultiply(DirectX::XMVectorCos(angle), rotationPlaneBasis1), DirectX::XMVectorMultiply(DirectX::XMVectorSin(angle), rotationPlaneBasis2)));
		float orientationTwistRotated = modelOrientationTwist + f_i * angleOrientationDelta / f_maxSteps2;
		storeBenchmark2[i-1] = DirectX::XMVectorSetW(orientationDirectionRotated, orientationTwistRotated);

		// in application now direction and twist can be used to transform vector (eg. model's vertice)
		// axis and angle around it as well as delta twist need to be determined from former and now direction first, delta = now - former !
		// determine axis, angle and delta twist
		// then
		// vectorAlongAxis = dot(vector, axis) * axis;
		// vectorTransformed = vectorAlongAxis + cos(angle) * (vector - vectorAlongAxis) + sin(angle) * cross(axis, vector)
		// vectorTransformedAlongDirection = dot(vectorTransformed, direction) * direction;
		// vectorTransformedFinal = vectorTransformedAlongDirection + cos(delta_twist) * (vectorTransformed - vectorTransformedAlongDirection) + sin(delta_twist) * cross(direction, vectorTransformed)
	}

	auto duration2 = std::chrono::high_resolution_clock::now() - now;


	// SANITY

	DirectX::XMVECTOR slerpResultOrientationDirectionExpected = DirectX::XMVector3Rotate(modelOrientationDirection, targetModelOrientationRotation);
	DirectX::XMVECTOR slerpResultDirectionDiff = DirectX::XMVector3AngleBetweenVectors(slerpResultOrientationDirectionExpected, targetModelOrientationDirection);
	bool slerpResultDirectionAsExpected = DirectX::XMVectorGetW(slerpResultDirectionDiff) < angleDeviationTolerance;
	DirectX::XMVECTOR slerpResultOrientationTwistExpected = DirectX::XMVector3Rotate(up, targetModelOrientationRotation);		// up corresponds to start twist in this application
	DirectX::XMVECTOR slerpResultTwistDiff = DirectX::XMVector3AngleBetweenVectors(slerpResultOrientationTwistExpected, twistActual);
	bool slerpResultTwistAsExpected = DirectX::XMVectorGetW(slerpResultTwistDiff) < angleDeviationTolerance;

	DirectX::XMFLOAT3 axisAngleResultOrientationDirectionExpected = { DirectX::XMVectorGetX(storeBenchmark2[maxSteps - 1]), DirectX::XMVectorGetY(storeBenchmark2[maxSteps - 1]), DirectX::XMVectorGetZ(storeBenchmark2[maxSteps - 1]) };
	DirectX::XMVECTOR axisAngleResultDiff = DirectX::XMVector3AngleBetweenVectors(DirectX::XMLoadFloat3(&axisAngleResultOrientationDirectionExpected), targetModelOrientationDirection);
	bool axisAngleDirectionResultAsExpected = DirectX::XMVectorGetW(axisAngleResultDiff) < angleDeviationTolerance;
	bool axisAngleTwistResultAsExpected = abs(targetRotationAroundTargetDirection - DirectX::XMVectorGetW(storeBenchmark2[maxSteps - 1])) < angleDeviationTolerance;


	// VALIDATION

	for (int i = 1; i <= maxSteps; ++i) {
		DirectX::XMVECTOR rotation = storeBenchmark1[i-1];
		DirectX::XMVECTOR axisAngle = storeBenchmark2[i-1];
		DirectX::XMFLOAT3 aARotatedOrientationDirection = { DirectX::XMVectorGetX(axisAngle), DirectX::XMVectorGetY(axisAngle), DirectX::XMVectorGetZ(axisAngle) };
		DirectX::XMVECTOR sRotatedOrientationDirection = DirectX::XMVector3Rotate(modelOrientationDirection, rotation);
		float directionDelta = DirectX::XMVectorGetW(DirectX::XMVector3AngleBetweenVectors(DirectX::XMLoadFloat3(&aARotatedOrientationDirection), sRotatedOrientationDirection));
		if(directionDelta > angleDeviationTolerance) {
			++numFails;
			float angleAxisAxisLengthDeviation = abs(DirectX::XMVectorGetW(DirectX::XMVector3Length(DirectX::XMLoadFloat3(&aARotatedOrientationDirection))) - 1.0f);
			if (angleAxisAxisLengthDeviation < epsilon) {
				float angleAxisAngleProportionDeviation = abs(DirectX::XMVectorGetW(DirectX::XMVectorDivide(DirectX::XMVector3AngleBetweenVectors(modelOrientationDirection, DirectX::XMLoadFloat3(&aARotatedOrientationDirection)), angleRotationMax)) - static_cast<float>(i) / f_maxSteps2);
				if (angleAxisAngleProportionDeviation < epsilon) {
					// the construction of orientation direction rotation in axisAngle results in a rotation along the shortest possible max angle, the twistRotation is rather trivial and fits, thus everything is fitting for axisAngle thus slerp is not as I would want a vector-length preserving equidistant-stepped rotation to occur
					++numFailsSlerp;
					axisDirectionDeviationSlerp[countAxisDirectionDeviationSlerp++] = directionDelta;
					axisLengthDeviationSlerp[countAxisLengthDeviationSlerp++] = abs(DirectX::XMVectorGetW(DirectX::XMVector3Length(sRotatedOrientationDirection)) - 1.0f);
				}
				else {
					// the angle is not the same proprotion to its max angle like the index is to maxSteps
					++numFailsAxisAngle;
					axisDirectionDeviationAxisAngle[countAxisDirectionDeviationAxisAngle++] = angleAxisAngleProportionDeviation;
				}
			}
			else {
				// no unit vector
				++numFailsAxisAngle;
				axisLengthDeviationAxisAngle[countAxisLengthDeviationAxisAngle++] = angleAxisAxisLengthDeviation;
			}
		}
		deviationDirections[countDeviationDirections++] = directionDelta;
	}

	float averageDeviationDirections = 0.0f;
	for (int i = 0; i < countDeviationDirections; ++i) {
		averageDeviationDirections += deviationDirections[i];
	}
	if (countDeviationDirections > 0) {
		averageDeviationDirections /= static_cast<float>(countDeviationDirections);
	}
	float avgAxisDirectionDeviationSlerp = 0.0f;
	for (int i = 0; i < countAxisDirectionDeviationSlerp; ++i) {
		avgAxisDirectionDeviationSlerp += axisDirectionDeviationSlerp[i];
	}
	if (countAxisDirectionDeviationSlerp > 0) {
		avgAxisDirectionDeviationSlerp /= static_cast<float>(countAxisDirectionDeviationSlerp);
	}
	float avgAxisLengthDeviationSlerp = 0.0f;
	for (int i = 0; i < countAxisLengthDeviationSlerp; ++i) {
		avgAxisLengthDeviationSlerp += axisLengthDeviationSlerp[i];
	}
	if (countAxisLengthDeviationSlerp > 0) {
		avgAxisLengthDeviationSlerp /= static_cast<float>(countAxisLengthDeviationSlerp);
	}
	float avgAxisDirectionDeviationAxisAngle = 0.0f;
	for (int i = 0; i < countAxisDirectionDeviationAxisAngle; ++i) {
		avgAxisDirectionDeviationAxisAngle += axisDirectionDeviationAxisAngle[i];
	}
	if (countAxisDirectionDeviationAxisAngle > 0) {
		avgAxisDirectionDeviationAxisAngle /= static_cast<float>(countAxisDirectionDeviationAxisAngle);
	}
	float avgAxisLengthDeviationAxisAngle = 0.0f;
	for (int i = 0; i < countAxisLengthDeviationAxisAngle; ++i) {
		avgAxisLengthDeviationAxisAngle += axisLengthDeviationAxisAngle[i];
	}
	if (countAxisLengthDeviationAxisAngle > 0) {
		avgAxisLengthDeviationAxisAngle /= static_cast<float>(countAxisLengthDeviationAxisAngle);
	}


	// OUTPUT

	std::cout
		<< "computed value for anti compiler-based measurement code optimisation:  " << aO << std::endl
		<< "quaternion slerp direction final result like to be expected:  " << (slerpResultDirectionAsExpected ? "true" : "false") << std::endl
		<< "quaternion slerp twist final result like to be expected:      " << (slerpResultTwistAsExpected ? "true" : "false") << std::endl
		<< "axisAngle direction final result like to be expected:    " << (axisAngleDirectionResultAsExpected ? "true" : "false") << std::endl
		<< "axisAngle angle twist final result like to be expected:  " << (axisAngleTwistResultAsExpected ? "true" : "false") << std::endl
		<< "arithmetic average of deviation of direction of slerp from direction of angleAxis across all steps:  " << std::fixed << std::setprecision(3) << averageDeviationDirections / D2R << " degrees" << std::endl
		<< "proportion of number of 'orientation in step differs more than " << std::fixed << std::setprecision(1) << angleDeviationToleranceD << " degrees between quaternion slerp and axisAngle' to number of steps:  " << std::fixed << std::setprecision(1) << static_cast<float>(numFails) * 100.0f / static_cast<float>(maxSteps) << "%" << std::endl
		<< "thereof proportion of \"slerp is wrong\":      " << std::fixed << std::setprecision(3) << (numFails > 0 ? static_cast<float>(numFailsSlerp) * 100.0f / static_cast<float>(numFails) : 0.0f) << "%" << std::endl
		<< "thereof proportion of \"axisAngle is wrong\":  " << std::fixed << std::setprecision(3) << (numFails > 0 ? static_cast<float>(numFailsAxisAngle) * 100.0f / static_cast<float>(numFails) : 0.0f) << "%" << std::endl
		<< "average of direction deviation when slerp is wrong:     " << std::fixed << std::setprecision(1) << avgAxisDirectionDeviationSlerp / D2R << " degrees" << std::endl
		<< "average of direction length error when slerp is wrong:  " << std::fixed << std::setprecision(3) << avgAxisLengthDeviationSlerp << " from 1.0f" << std::endl
		<< "average of direction proportion deviation when axisAngle is wrong:  " << std::fixed << std::setprecision(1) << avgAxisDirectionDeviationAxisAngle * 100.0f << " % of whole direction rotation's angle" << std::endl
		<< "average of direction length error when axisAngle is wrong:          " << std::fixed << std::setprecision(3) << avgAxisLengthDeviationAxisAngle << " from 1.0f" << std::endl
		<< "duration quaternion slerp:  " << duration1 - durationA << std::endl
		<< "duration axisAngle:         " << duration2 - durationB << std::endl
		<< "duration axisAngle compared to duration quaternion slerp:  " << (duration2 - durationB) * 100.0f / (duration1 - durationA) << "%" << std::endl;


	std::cout << std::endl << "press enter to exit" << std::endl;
	char r = '0';
	std::cin.get() >> r;


	return 0;
}