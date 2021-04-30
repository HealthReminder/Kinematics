using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Kinematics
{
    public class KinematicsJoint : MonoBehaviour
    {
        //Axis in which the joint movement will occur
        public Vector3 axis;
        //Offset joint had at start (T-Pose)
        public Vector3 start_offset;
        //Constraints
        public float angle_min, angle_max;
        private void Awake()
        {
            start_offset = transform.localPosition;
        }
    }
}
