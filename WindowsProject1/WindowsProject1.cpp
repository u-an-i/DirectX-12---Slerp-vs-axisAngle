#include <d3d12.h>
#include <dxgi1_6.h>
#include <iostream>
#include <chrono>
#include <directxmath.h>


const float PI = 3.1415926535897932;
const float D2R = PI / 180.0f;

const float epsilon = 0.0001f;

const int maxSteps = 1000000;

DirectX::XMVECTOR warmUp[maxSteps];
DirectX::XMVECTOR storeBenchmark1[maxSteps];
DirectX::XMVECTOR storeBenchmark2[maxSteps];

const DirectX::XMFLOAT3 modelDefaultLookDirection = { 0.0f, 0.0f, 1.0f };
const float modelDefaultRotationAroundDefaultLookDirection = 0.0f * D2R;

int numFails = 0;
int numFailsSlerp = 0;
int numFailsAxisAngle = 0;

int main() {
	// application: source for new target orientation

	DirectX::XMFLOAT3 targetDirection = { -1.0f, 1.0f, -1.0f };			// here only some value
	float targetRotationAroundTargetDirection = 90.0f * D2R;


	// storage of orientation as direction and twist around that

	const DirectX::XMVECTOR modelOrientationDirection = XMLoadFloat3(&modelDefaultLookDirection);
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
		having this code here results in durations of benchmark 2 which are up to around 100 x longer.
		If anything, having this code here should prevent an optimisation away of the baseline and as such a baseline should then be present.
		But a present baseline would reduce the duration delivered for benchmark 2 instead of increasing it because the baseline is subtracted.
		And the presence of this code should have no impact on the runtime of benchmark 2 itself.
		Is this a compiler (optimisation) bug ?
		In release builds when not having this code here performance is like in Debug builds where the presence of this code here has no impact on performance.
	*/
	goto pass;		// prevents issue mentioned in comment above from occurring
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

	// assume current orientation is already storerd in a quaternion, thus not part of duration measurement, when using quaternions slerp
	DirectX::XMVECTOR defaultModelOrientation = DirectX::XMQuaternionRotationNormal(modelOrientationDirection, modelOrientationTwist);


	// BENCHMARK 1: quaternions and slerp

	now = std::chrono::high_resolution_clock::now();

	// new orientation, eg from looking into a certain direction, must be converted to quaternion first
	DirectX::XMVECTOR targetModelOrientation = DirectX::XMQuaternionRotationNormal(targetModelOrientationDirection, targetRotationAroundTargetDirection);

	float f_maxSteps = static_cast<float>(maxSteps);

	for (int i = 1; i <= maxSteps; ++i) {
		storeBenchmark1[i-1] = DirectX::XMQuaternionSlerp(defaultModelOrientation, targetModelOrientation, static_cast<float>(i) / f_maxSteps);

		// in application now quaternion can be used to transform vector (eg. model's vertice)
		// vectorTransformed = quaternionRotate(vector, quaternion) is equal* vectorTransformed = axisAngle(vector, axis, angle)
		// * sans need to compute sin and cos of angle and it has 1 less op in serial context and 2 less op in parallel context
		// additionally memory used is larger for axisAngle
		// vectorTransformed = vector + 2 * cross(quaternion.xyz, cross(quaternion.xyz, vector) + quaternion.w * vector)
		// despite difference don't transform vector here
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

		// in application now quaternion can be used to transform vector (eg. model's vertice)
		// vectorTransformed = quaternionRotate(vector, quaternion) is equal* vectorTransformed = axisAngle(vector, axis, angle)
		// * sans need to compute sin and cos of angle and it has 1 less op in serial context and 2 less op in parallel context
		// additionally memory used is larger for axisAngle
		// vectorAlongAxis = dot(vector, axis) * axis;
		// vectorTransformed = vectorAlongAxis + cos(angle) * (vector - vectorAlongAxis) + sin(angle) * cross(axis, vector)
		// despite difference don't transform vector here
	}

	auto duration2 = std::chrono::high_resolution_clock::now() - now;


	// VALIDATION

	DirectX::XMVECTOR epsilonV = { epsilon, epsilon, epsilon, epsilon };

	for (int i = 1; i <= maxSteps; ++i) {
		DirectX::XMVECTOR rotation = storeBenchmark1[i-1];
		DirectX::XMVECTOR axisAngle = storeBenchmark2[i-1];
		DirectX::XMFLOAT3 axisV3 = { DirectX::XMVectorGetX(axisAngle), DirectX::XMVectorGetY(axisAngle), DirectX::XMVectorGetZ(axisAngle) };
		DirectX::XMVECTOR cQ = DirectX::XMQuaternionRotationAxis(DirectX::XMLoadFloat3(&axisV3), DirectX::XMVectorGetW(axisAngle));
		DirectX::XMVECTOR isDifferentV = DirectX::XMVectorSubtract(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(rotation, cQ)), epsilonV);
		if(
			DirectX::XMVectorGetW(isDifferentV) > 0.0f || DirectX::XMVectorGetX(isDifferentV) > 0.0f || DirectX::XMVectorGetY(isDifferentV) > 0.0f || DirectX::XMVectorGetZ(isDifferentV) > 0.0f
		) {
			++numFails;
			if (abs(DirectX::XMVectorGetW(DirectX::XMVector3Length(DirectX::XMLoadFloat3(&axisV3))) - 1.0f) - epsilon < 0.0f) {
				if (abs(DirectX::XMVectorGetW(DirectX::XMVectorDivide(DirectX::XMVector3AngleBetweenNormals(modelOrientationDirection, DirectX::XMLoadFloat3(&axisV3)), angleRotationMax)) - static_cast<float>(i)/f_maxSteps2) - epsilon < 0.0f) {
					// the construction of axisAngle results in a rotation along the shortest possible max angle, everything is fitting for axisAngle thus slerp is not as I would want a rotation to occur
					++numFailsSlerp;
				}
				else {
					// the angle is not the same proprotion to its max angle like the index is to maxSteps
					++numFailsAxisAngle;
				}
			}
			else {
				// no unit vector
				++numFailsAxisAngle;
			}
		}
	}


	// SANITY

	DirectX::XMVECTOR resultOrientationExpected = DirectX::XMQuaternionRotationNormal(targetModelOrientationDirection, targetRotationAroundTargetDirection);
	DirectX::XMVECTOR slerpResultDiff = DirectX::XMVectorSubtract(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(resultOrientationExpected, storeBenchmark1[maxSteps - 1])), epsilonV);
	bool slerpResultAsExpected = DirectX::XMVectorGetW(slerpResultDiff) < 0.0f && DirectX::XMVectorGetX(slerpResultDiff) < 0.0f && DirectX::XMVectorGetY(slerpResultDiff) < 0.0f && DirectX::XMVectorGetZ(slerpResultDiff) < 0.0f;

	DirectX::XMVECTOR axisResultDiff = DirectX::XMVectorSubtract(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(DirectX::XMVector4Normalize(DirectX::XMVectorSetW(targetModelOrientationDirection, 0.0f)), DirectX::XMVector4Normalize(DirectX::XMVectorSetW(storeBenchmark2[maxSteps - 1], 0.0f)))), epsilonV);
	bool axisResultsAsExpected = DirectX::XMVectorGetW(axisResultDiff) < 0.0f && DirectX::XMVectorGetX(axisResultDiff) < 0.0f && DirectX::XMVectorGetY(axisResultDiff) < 0.0f && DirectX::XMVectorGetZ(axisResultDiff) < 0.0f;
	bool angleResultAsExpected = (abs(targetRotationAroundTargetDirection - DirectX::XMVectorGetW(storeBenchmark2[maxSteps - 1])) - epsilon) < 0.0f;


	// OUTPUT

	std::cout
		<< "computed value for anti compiler-based measurement code optimisation:  " << aO << "\n"
		<< "quaternion slerp result like to be expected:  " << (slerpResultAsExpected ? "true" : "false") << "\n"
		<< "axisAngle axis result like to be expected:    " << (axisResultsAsExpected ? "true" : "false") << "\n"
		<< "axisAngle angle result like to be expected:   " << (angleResultAsExpected ? "true" : "false") << "\n"
		<< "proportion of number of 'orientation in step differs \"too much\" between quaternion slerp and axisAngle' to number of steps:  " << static_cast<float>(numFails) * 100.0f / static_cast<float>(maxSteps) << "%" << "\n"
		<< "thereof proportion of \"slerp is wrong\":      " << static_cast<float>(numFailsSlerp) * 100.0f / static_cast<float>(numFails) << "%" << "\n"
		<< "thereof proportion of \"axisAngle is wrong\":  " << static_cast<float>(numFailsAxisAngle) * 100.0f / static_cast<float>(numFails) << "%" << "\n"
		<< "duration quaternion slerp:  " << duration1 - durationA << "\n"
		<< "duration axisAngle:         " << duration2 - durationB << "\n"
		<< "duration axisAngle compared to duration quaternion slerp:  " << (duration2 - durationB) * 100.0f / (duration1 - durationA) << "%" << "\n";


	std::cout << "\npress enter to exit\n";
	char r = '0';
	std::cin.get() >> r;


	return 0;
}