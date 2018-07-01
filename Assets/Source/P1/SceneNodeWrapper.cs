using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SceneNodeWrapper : MonoBehaviour {

	public Transform gObject;
	public int vertexId;

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		Vector3[] v = gObject.GetComponent<MeshFilter> ().mesh.vertices;
		v [vertexId] = transform.localPosition;
		gObject.GetComponent<MeshFilter> ().mesh.vertices = v;
	}
}
