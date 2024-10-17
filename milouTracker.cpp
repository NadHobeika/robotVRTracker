#include <openvr.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// Convert VR matrix to position and quaternion
void GetPositionAndRotation(const vr::HmdMatrix34_t& matrix, vr::HmdVector3_t& position, vr::HmdQuaternion_t& rotation) {
    // Position
    position.v[0] = matrix.m[0][3];
    position.v[1] = matrix.m[1][3];
    position.v[2] = matrix.m[2][3];

    // Rotation (Convert 3x4 matrix to a quaternion)
    rotation.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    rotation.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    rotation.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    rotation.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    // Adjust signs of quaternion
    rotation.x = copysign(rotation.x, matrix.m[2][1] - matrix.m[1][2]);
    rotation.y = copysign(rotation.y, matrix.m[0][2] - matrix.m[2][0]);
    rotation.z = copysign(rotation.z, matrix.m[1][0] - matrix.m[0][1]);
}

int main() {
    vr::EVRInitError eError = vr::VRInitError_None;

    // Initialize OpenVR
    vr::IVRSystem* vr_system = vr::VR_Init(&eError, vr::VRApplication_Scene);
    if (eError != vr::VRInitError_None) {
        std::cerr << "Error: Unable to initialize VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
        return -1;
    }

    vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    while (true) {
        // Update poses from OpenVR
        vr_system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);

        for (vr::TrackedDeviceIndex_t deviceIndex = 0; deviceIndex < vr::k_unMaxTrackedDeviceCount; ++deviceIndex) {
            if (trackedDevicePose[deviceIndex].bPoseIsValid) {
                vr::ETrackedDeviceClass deviceClass = vr_system->GetTrackedDeviceClass(deviceIndex);

                vr::HmdVector3_t position;
                vr::HmdQuaternion_t rotation;

                // Extract position and rotation
                GetPositionAndRotation(trackedDevicePose[deviceIndex].mDeviceToAbsoluteTracking, position, rotation);

                // Handle different types of tracked devices
                if (deviceClass == vr::TrackedDeviceClass_HMD) {
                    // Headset (HMD) Tracking
                    std::cout << "Headset Position: x = " << position.v[0]
                        << ", y = " << position.v[1] << ", z = " << position.v[2] << std::endl;
                    std::cout << "Headset Rotation: x = " << rotation.x
                        << ", y = " << rotation.y << ", z = " << rotation.z << ", w = " << rotation.w << std::endl;

                }
                else if (deviceClass == vr::TrackedDeviceClass_Controller) {
                    // Controller Tracking
                    std::cout << "Controller " << deviceIndex << " Position: x = " << position.v[0]
                        << ", y = " << position.v[1] << ", z = " << position.v[2] << std::endl;
                    std::cout << "Controller " << deviceIndex << " Rotation: x = " << rotation.x
                        << ", y = " << rotation.y << ", z = " << rotation.z << ", w = " << rotation.w << std::endl;

                }
                else if (deviceClass == vr::TrackedDeviceClass_GenericTracker) {
                    // Generic Tracker Tracking
                    std::cout << "Generic Tracker " << deviceIndex << " Position: x = " << position.v[0]
                        << ", y = " << position.v[1] << ", z = " << position.v[2] << std::endl;
                    std::cout << "Generic Tracker " << deviceIndex << " Rotation: x = " << rotation.x
                        << ", y = " << rotation.y << ", z = " << rotation.z << ", w = " << rotation.w << std::endl;

                }
                else {
                    // Other types of tracked devices can be added here (e.g., base stations, etc.)
                }
            }
        }


        // Sleep for a bit (this is just to simulate a loop in a simple app)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Clean up and shut down
    vr::VR_Shutdown();
    return 0;
}
