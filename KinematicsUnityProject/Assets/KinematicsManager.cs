using Kinematics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Kinematics
{
    /*All the code regarding Kinematics was written by me Eduardo Francio at "https://github.com/HealthReminder"
      Based on the tutorial written by Allan Zucconi at "https://www.alanzucconi.com/2017/04/17/procedural-animations/" */
    public class KinematicsManager : MonoBehaviour
    {
        /* Definitions:
          END EFFECTOR - is a position at the most outboard position of the most outboard link.
          It is the free end of the chain of alternating joints and links. The end effector is not a joint.
          The end effector is merely the position at the end of an articulated body.
          An articulated body can have multiple end effectors
          FORWARD KINEMATICS - is basically how the hierarchy in Unity works, unfortunately to implement
          Inverse Kinematics we need the forward movement not to be realized, only calculated, and that is
          Why we don't use Unity's but implement our own function that does not apply the movement necessarily
          That way, if we have a specific point in space that we want to reach, we can use Forward Kinematics to estimate how close we are
          GRADIENT or DIRECTIONAL DERIVATIVE - of a function is a vector that points in the direction of the steepest ascent, is found using a sampling distance.
          DERIVATIVE - of a function is a single number that indicates how fast a function is rising when moving in the direction of its gradient.
          LEARNING RATE - dictates how fast we move against the gradient. Larger values approach the solution faster, but are also more likely to overshoot it.
        */
        //Joints are assigned manually
        public KinematicsJoint[] joints;
        //An end effector is basically the "hand" of the arm
        public Transform end_effector;
        //Where the "hand" is trying to get to
        public Transform target_transform;
        //The sampling distance defines the rotation on the joints we will be testing to find the less resistant movement
        public float sampling_distance = 5;
        public float learning_rate = 2;
        //This is the minimum distance the arm should be from the target
        //It exists to avoid the wiggly movement when the arm keeps overshooting the target
        public float distance_threshold = 0.1f;

        //The following variables are related to details in how the tentacle moves
        //These variables are used by the ErrorFunction, previously called DistanceFromTarget


        //Public for debug only
        public float[] angles;
        private void Awake()
        {
            angles = GetJointAngles();
        }
        private void Update()
        {
            if (!target_transform)
            {
                Debug.LogError("No target transform found!");
                return;
            }
            InverseKinematics(target_transform.position, angles);
            ApplyAngles(angles);
        }

        void ApplyAngles(float[] joint_angles)
        {
            var c = joints.Length;
            for (int i = 0; i < c; i++)
            {
                Vector3 new_rotation = new Vector3(
                    joints[i].axis.x * joint_angles[i],
                    joints[i].axis.y * joint_angles[i],
                    joints[i].axis.z * joint_angles[i]);
                joints[i].transform.localRotation = Quaternion.Euler(new_rotation);
            }
        }
        float[] GetJointAngles()
        {
            var c = joints.Length;
            float[] joint_angles = new float[c];

            /* Emergency debug code
            for (int i = 0; i < c; i++) {
                if (joints[i].axis.x == 1) {
                    joint_angles[i] = joints[i].transform.localRotation.eulerAngles.x;
                }
                else if (joints[i].axis.y == 1) {
                    joint_angles[i] = joints[i].transform.localRotation.eulerAngles.y;
                }
                else if (joints[i].axis.z == 1) {
                    joint_angles[i] = joints[i].transform.localRotation.eulerAngles.z;
                }
                else {
                    Debug.LogError("Joint has invalid axis");
                }
            }*/

            for (int i = 0; i < c; i++)
            {
                Vector3 local_rot = joints[i].transform.localEulerAngles;
                //Make sure the joints.axis are unit vectors on only one axis
                float angle = (local_rot.x * joints[i].axis.x) + (local_rot.y * joints[i].axis.y) + (local_rot.z * joints[i].axis.z);
                joint_angles[i] = angle;
            }

            return joint_angles;
        }

        public float[] InverseKinematics(Vector3 target, float[] joint_angles)
        {
            //Invoking this function repeatedly move the robotic arm closer to the target point.

            //Avoid arm wiggle by checking if its within the minimum range from the target
            if (ErrorFunction(target, joint_angles) < distance_threshold)
                return joint_angles;

            var c = joint_angles.Length;
            for (int i = 0; i < c; i++)
            {
                float gradient = PartialGradient(target, joint_angles, i);
                joint_angles[i] -= learning_rate * gradient;

                //Constrain the joint to its min and max angles
                joint_angles[i] = Mathf.Clamp(joint_angles[i], joints[i].angle_min, joints[i].angle_max);

                //Early avoid arm wiggle by checking if its within the minimum range from the target
                if (ErrorFunction(target, joint_angles) < distance_threshold)
                    return joint_angles;
            }
            return joint_angles;
        }

        private float PartialGradient(Vector3 target, float[] joint_angles, int joint_index)
        {
            //Previously, when invoked, this function returns a single number that indicates how the distance from our target changes as a function of the joint rotation
            //The least resistant movement is the movement towards the bottom of the curve, the place closest to our target
            //But currently, the ErrorFunction has more complexity other than only distance from the target

            //Store old angle
            float initial_angle = joint_angles[joint_index];

            //Find current distance
            float current_dist = ErrorFunction(target, joint_angles);

            //Find distance using the sampling distance
            joint_angles[joint_index] += sampling_distance;
            float sampled_dist = ErrorFunction(target, joint_angles);

            float gradient = (sampled_dist - current_dist) / sampling_distance;

            //Restore initial angles
            joint_angles[joint_index] = initial_angle;

            //Return this gradient
            return gradient;
        }
        private float ErrorFunction(Vector3 target_pos, float[] joint_angles)
        {
            //This function alters the arm movement by applying penalties towards unwanted behaviour
            //ALL VALUES MUST BE NORMALIZED
            //Inverse Kinematics always wants to minimize the distance from target
            Vector3 current_pos = ForwardKinematics(joint_angles);
            float distance_penalty = Vector3.Distance(current_pos, target_pos);
            //The tip of the tentacle tries to match the rotation of the object we want to reach //It will try to match to the UP vector
            float rotation_penalty = Mathf.Abs(Quaternion.Angle(end_effector.rotation, target_transform.rotation));
            rotation_penalty /= 180f; //Normalization
            //Keeping limbs in unnatural positions is uncomfortable
            float torsion_penalty = 0;
            for (int i = 1; i < joint_angles.Length; i++)
                torsion_penalty += Mathf.Abs(Quaternion.Angle(joints[i-1].transform.localRotation, joints[i].transform.localRotation) );
            torsion_penalty /= joint_angles.Length-1;
            torsion_penalty /= 180f; //Normalization

            Debug.Log("Dist " + distance_penalty + "Rot " + rotation_penalty + "Torsion " + torsion_penalty);
            return distance_penalty*1+rotation_penalty * 10+ torsion_penalty*1f;
        }

        [System.Obsolete("DistanceFromTarget is deprecated. Use ErrorFunction() method for more complex movements.")] 
        private float DistanceFromTarget(Vector3 target_pos, float[] joint_angles)
        {
            //Inverse Kinematics always wants to minimize the value returne by this function
            Vector3 current_pos = ForwardKinematics(joint_angles);
            float dist = Vector3.Distance(current_pos, target_pos);
            return dist;
        }

        public Vector3 ForwardKinematics(float[] joint_angles)
        {
            //This function returns the position of the end effector, in global coordinates
            //Implements the following Forward Kinematics equation: 

            Vector3 point_previous = joints[0].transform.position;
            Quaternion rotation_current = Quaternion.identity;
            var c = joints.Length;
            for (int i = 1; i < c; i++)
            {
                //Basically rotates every joint except for the first one according to its angle and the axis it rotates
                rotation_current *= Quaternion.AngleAxis(joint_angles[i - 1], joints[i - 1].axis);
                Vector3 point_next = point_previous + rotation_current * joints[i].start_offset;
                point_previous = point_next;
            }
            //Return the final point in the chain (The end effector)
            Vector3 end_point = point_previous;
            return end_point;
        }
    }
}
