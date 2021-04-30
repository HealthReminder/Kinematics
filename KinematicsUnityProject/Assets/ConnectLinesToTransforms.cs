using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConnectLinesToTransforms : MonoBehaviour
{
    //This script will take lines and connect their points to their parents
    public LineRenderer[] lines;
    private void Update()
    {
        for (int i = 0; i < lines.Length; i++)
        {
            lines[i].SetPosition(0, lines[i].transform.position);
            lines[i].SetPosition(1, lines[i].transform.parent.position);
        }
    }
}
